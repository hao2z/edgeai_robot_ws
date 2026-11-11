#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <string>
#include <vector>
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nlohmann/json.hpp"
#include "lattice_heading_order.hpp"
#include "heading_candidates.hpp"

namespace edgeai_navigation
{
// Jazzy compatibility for upstream ALL_DIRECTION / BIDIRECTIONAL goal headings.
// Compare native, footprint-checked paths rather than returning the first yaw.
class XYLatticePlanner : public nav2_core::GlobalPlanner
{
public:
  XYLatticePlanner() : loader_("nav2_core", "nav2_core::GlobalPlanner") {}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_ = parent.lock(); costmap_ros_ = costmap;
    const auto parameter = [&](const std::string & key, auto value) {
        nav2_util::declare_parameter_if_not_declared(node_,name+"."+key,rclcpp::ParameterValue(value));
      };
    parameter("goal_heading_mode",std::string("ALL_DIRECTION"));
    parameter("goal_heading_count",4);
    parameter("heading_search_budget_s",2.0);
    parameter("heading_reuse",true);
    node_->get_parameter(name+".goal_heading_mode",mode_);
    node_->get_parameter(name+".goal_heading_count",heading_count_);
    node_->get_parameter(name+".heading_search_budget_s",budget_);
    node_->get_parameter(name+".heading_reuse",reuse_);
    if ((mode_!="ALL_DIRECTION" && mode_!="BIDIRECTIONAL" && mode_!="DEFAULT") ||
      heading_count_<0 || !std::isfinite(budget_) || budget_<=0) {throw std::runtime_error("Invalid goal heading search configuration");}
    lattice_ = loader_.createSharedInstance("nav2_smac_planner::SmacPlannerLattice");
    lattice_->configure(parent, name, tf, costmap);
    node_->get_parameter(name + ".allow_unknown", allow_unknown_);
    node_->get_parameter(name+".cost_penalty",cost_penalty_);
    node_->get_parameter(name+".rotation_penalty",rotation_penalty_);
    node_->get_parameter(name+".reverse_penalty",reverse_penalty_);
    std::string filepath;
    node_->get_parameter(name + ".lattice_filepath", filepath);
    std::ifstream stream(filepath);
    nlohmann::json primitives;
    stream >> primitives;
    headings_ = primitives.at("lattice_metadata").at("heading_angles").get<std::vector<double>>();
    if (headings_.empty()) {throw std::runtime_error("Empty lattice heading set");}
    if (static_cast<size_t>(heading_count_)>headings_.size()) {
      throw std::runtime_error("goal_heading_count exceeds available lattice headings");
    }
  }
  void activate() override {lattice_->activate();}
  void deactivate() override {lattice_->deactivate();}
  void cleanup() override
  {lattice_->cleanup(); lattice_.reset(); cached_yaw_.reset(); has_goal_ = false;}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancelled) override
  {
    const auto & q=goal.pose.orientation;
    const double preferred=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>1e-8 ? tf2::getYaw(q) :
      std::atan2(goal.pose.position.y-start.pose.position.y,goal.pose.position.x-start.pose.position.x);
    const bool same_goal = has_goal_ && goal.header.frame_id == cached_goal_.header.frame_id &&
      std::hypot(goal.pose.position.x - cached_goal_.pose.position.x,
      goal.pose.position.y - cached_goal_.pose.position.y) < 0.025;
    if (!same_goal) {cached_yaw_.reset();cached_goal_=goal;has_goal_=true;}
    const auto reusable=reuse_ ? cached_yaw_ : std::nullopt;
    auto headings = selectGoalHeadings(headings_,preferred,heading_count_,reusable);
    auto difference = [preferred](double yaw) {
        return std::abs(std::atan2(std::sin(yaw - preferred), std::cos(yaw - preferred)));
      };
    std::stable_sort(headings.begin(), headings.end(), [&](double a, double b) {
        return difference(a) < difference(b);
      });
    if (mode_=="DEFAULT") {headings={preferred};}
    else if (mode_=="BIDIRECTIONAL") {headings={preferred,std::remainder(preferred+M_PI,2*M_PI)};}
    // Try footprint-clear goal orientations first, before requesting a native
    // search for an endpoint that already intersects a wall.
    unsigned int clear_count = 0;
    {
      auto * map = costmap_ros_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      const auto footprint = costmap_ros_->getRobotFootprint();
      auto split = std::stable_partition(headings.begin(), headings.end(), [&](double yaw) {
          return exactGoalFootprintClear(*map, footprint, goal.pose.position.x,
            goal.pose.position.y, yaw, allow_unknown_);
        });
      clear_count = std::distance(headings.begin(), split);
    }
    if (reusable) {
      const auto found=std::find_if(headings.begin(),headings.end(),[&](double yaw) {
          return std::abs(std::remainder(yaw-*reusable,2*M_PI))<1e-8;
        });
      if (found!=headings.end()) {std::rotate(headings.begin(),found,found+1);}
    }
    const auto begun=std::chrono::steady_clock::now();
    const auto result=chooseHeadingPlan(headings,budget_,reusable,cancelled,
      [&](double yaw,const std::function<bool()> & stop) {
        auto candidate=goal;
        candidate.pose.orientation.x=candidate.pose.orientation.y=0.;
        candidate.pose.orientation.z=std::sin(yaw/2.);candidate.pose.orientation.w=std::cos(yaw/2.);
        return lattice_->createPlan(start,candidate,stop);
      },[&](const nav_msgs::msg::Path & path) {
        auto * map=costmap_ros_->getCostmap();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
        const auto footprint=costmap_ros_->getRobotFootprint();
        for (const auto & p:path.poses) {
          if (!exactGoalFootprintClear(*map,footprint,p.pose.position.x,p.pose.position.y,
              tf2::getYaw(p.pose.orientation),allow_unknown_)) {return std::numeric_limits<double>::infinity();}
        }
        return headingPathCost(path,*map,cost_penalty_,rotation_penalty_,reverse_penalty_,allow_unknown_);
      });
    cached_yaw_=result.yaw;
    RCLCPP_INFO(node_->get_logger(),
      "XY_GOAL_HEADING: (%.2f, %.2f), yaw %.3f; candidates %u/%zu, feasible %u, exact-clear %u, cost %.3f, reused %d, %.1f ms",
      goal.pose.position.x,goal.pose.position.y,result.yaw,result.attempted,headings.size(),result.feasible,
      clear_count,result.cost,result.reused,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begun).count());
    return result.path;
  }
private:
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_;
  nav2_core::GlobalPlanner::Ptr lattice_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  bool allow_unknown_ = true;
  bool has_goal_ = false;
  std::string mode_;
  bool reuse_=true;
  int heading_count_=4;
  double budget_=2.0,cost_penalty_=2.,rotation_penalty_=5.,reverse_penalty_=2.;
  std::optional<double> cached_yaw_;
  std::vector<double> headings_;
  geometry_msgs::msg::PoseStamped cached_goal_;
};
}  // namespace edgeai_navigation
PLUGINLIB_EXPORT_CLASS(edgeai_navigation::XYLatticePlanner, nav2_core::GlobalPlanner)
