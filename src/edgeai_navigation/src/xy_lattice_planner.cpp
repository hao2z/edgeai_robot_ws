#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <set>
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

namespace edgeai_navigation
{
// Jazzy Smac Lattice has no free-goal-heading mode. A cheap 2D guide suggests
// an arrival direction; only the SE2 planner's footprint-checked path is returned.
class XYLatticePlanner : public nav2_core::GlobalPlanner
{
public:
  XYLatticePlanner() : loader_("nav2_core", "nav2_core::GlobalPlanner") {}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_ = parent.lock(); costmap_ros_ = costmap;
    lattice_ = loader_.createSharedInstance("nav2_smac_planner::SmacPlannerLattice");
    lattice_->configure(parent, name, tf, costmap);
    node_->get_parameter(name + ".allow_reverse_expansion", allow_reverse_);
    node_->get_parameter(name + ".allow_unknown", allow_unknown_);
    std::string filepath;
    node_->get_parameter(name + ".lattice_filepath", filepath);
    std::ifstream stream(filepath);
    nlohmann::json primitives;
    stream >> primitives;
    headings_ = primitives.at("lattice_metadata").at("heading_angles").get<std::vector<double>>();
    if (headings_.empty()) {throw std::runtime_error("Empty lattice heading set");}
    guide_ = loader_.createSharedInstance("nav2_navfn_planner::NavfnPlanner");
    nav2_util::declare_parameter_if_not_declared(node_, name + ".guide.use_final_approach_orientation",
      rclcpp::ParameterValue(true));
    guide_->configure(parent, name + ".guide", tf, costmap);
  }
  void activate() override {lattice_->activate(); guide_->activate();}
  void deactivate() override {lattice_->deactivate(); guide_->deactivate();}
  void cleanup() override
  {lattice_->cleanup(); guide_->cleanup(); lattice_.reset(); guide_.reset(); cached_yaw_.reset(); has_goal_ = false; rejected_.clear();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancelled) override
  {
    double preferred = std::atan2(goal.pose.position.y - start.pose.position.y,
      goal.pose.position.x - start.pose.position.x);
    const bool same_goal = has_goal_ && goal.header.frame_id == cached_goal_.header.frame_id &&
      std::hypot(goal.pose.position.x - cached_goal_.pose.position.x,
      goal.pose.position.y - cached_goal_.pose.position.y) < 0.025;
    if (same_goal) {
      preferred = cached_yaw_.value_or(preferred_yaw_);
    } else {
      rejected_.clear(); cached_yaw_.reset(); cached_goal_ = goal; has_goal_ = true;
      try {
        auto guide = guide_->createPlan(start, goal, cancelled);
        if (guide.poses.size() > 1) {preferred = tf2::getYaw(guide.poses.back().pose.orientation);}
        // A rearward arrival can keep vehicle heading instead of demanding a
        // half-turn at a position-only goal. Native SE2 search still checks the
        // whole route and applies reverse_penalty; it may choose forward motion.
        if (allow_reverse_ && std::cos(preferred - tf2::getYaw(start.pose.orientation)) < 0.0) {
          preferred = std::atan2(std::sin(preferred + M_PI), std::cos(preferred + M_PI));
        }
      } catch (const nav2_core::PlannerException &) {
        if (cancelled()) {throw;}
      }
      preferred_yaw_ = preferred;
    }
    auto headings = headings_;
    auto difference = [preferred](double yaw) {
        return std::abs(std::atan2(std::sin(yaw - preferred), std::cos(yaw - preferred)));
      };
    std::stable_sort(headings.begin(), headings.end(), [&](double a, double b) {
        return difference(a) < difference(b);
      });
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
    auto selected = std::find_if(headings.begin(), headings.end(), [&](double yaw) {
        return rejected_.count(yaw) == 0;
      });
    if (selected == headings.end()) {rejected_.clear(); selected = headings.begin();}
    const double yaw = *selected;
    auto candidate = goal;
    candidate.pose.orientation.x = candidate.pose.orientation.y = 0.0;
    candidate.pose.orientation.z = std::sin(yaw / 2.0);
    candidate.pose.orientation.w = std::cos(yaw / 2.0);
    RCLCPP_INFO(node_->get_logger(), "XY_GOAL_HEADING: (%.2f, %.2f), yaw %.3f; exact-footprint-clear %u/%zu",
      goal.pose.position.x, goal.pose.position.y, yaw, clear_count, headings.size());
    // One native search per request. Existing BT owns retries and recoveries.
    try {
      auto path = lattice_->createPlan(start, candidate, cancelled);
      if (path.poses.empty()) {
        throw nav2_core::NoValidPathCouldBeFound("Native lattice returned an empty path for selected heading");
      }
      cached_yaw_ = yaw; rejected_.clear();
      return path;
    } catch (const nav2_core::StartOccupied &) {throw;}
    catch (const nav2_core::StartOutsideMapBounds &) {throw;}
    catch (const nav2_core::GoalOutsideMapBounds &) {throw;}
    catch (const nav2_core::PlannerException &) {
      if (!cancelled()) {rejected_.insert(yaw);}
      throw;
    }
  }
private:
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_;
  nav2_core::GlobalPlanner::Ptr lattice_, guide_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  bool allow_reverse_ = false;
  bool allow_unknown_ = true;
  bool has_goal_ = false;
  double preferred_yaw_ = 0.0;
  std::set<double> rejected_;
  std::optional<double> cached_yaw_;
  std::vector<double> headings_;
  geometry_msgs::msg::PoseStamped cached_goal_;
};
}  // namespace edgeai_navigation
PLUGINLIB_EXPORT_CLASS(edgeai_navigation::XYLatticePlanner, nav2_core::GlobalPlanner)
