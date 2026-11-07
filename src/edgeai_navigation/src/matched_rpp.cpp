#include <cmath>
#include <limits>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "recovery_history.hpp"

namespace edgeai_navigation
{
// Reuse native RPP tracking and collision checks. Choose its speed budget from
// the same carrot/curvature before native compute, rather than scaling afterwards.
class MatchedRPP : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    RegulatedPurePursuitController::configure(parent, name, tf, map);
    auto node = parent.lock();
    auto read = [&](const std::string & key, double & value) {
        nav2_util::declare_parameter_if_not_declared(node, name + key, rclcpp::ParameterValue(value));
        node->get_parameter(name + key, value);
        if (!std::isfinite(value) || value <= 0.0) {
          throw std::runtime_error(name + key + " must be finite and positive");
        }
      };
    read(".motion_forward_limit", forward_);
    read(".motion_reverse_limit", reverse_);
    read(".motion_angular_limit", angular_);
    status_pub_ = node->create_publisher<std_msgs::msg::String>("navigation/rpp_tracking", 1);
  }

  void activate() override {RegulatedPurePursuitController::activate(); status_pub_->on_activate();}
  void deactivate() override {status_pub_->on_deactivate(); RegulatedPurePursuitController::deactivate();}
  void cleanup() override {status_pub_.reset(); RegulatedPurePursuitController::cleanup();}
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!path.poses.empty()) {
      const auto & next = path.poses.back();
      if (path.header.frame_id != goal_.header.frame_id ||
        std::hypot(next.pose.position.x - goal_.pose.position.x, next.pose.position.y - goal_.pose.position.y) > .1)
      {progress_.failed();}
      goal_ = next; goal_.header.frame_id = path.header.frame_id;
    }
    RegulatedPurePursuitController::setPlan(path);
  }

  void setSpeedLimit(const double & limit, const bool & percentage) override
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!std::isfinite(limit) || limit < 0.0) {
      RCLCPP_WARN(logger_, "Ignored invalid external speed limit");
      return;
    }
    external_limit_ = limit;
    external_percentage_ = percentage;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override
  {
    std::lock_guard<std::mutex> motion_lock(motion_mutex_);
    double budget;
    reference_.clear();
    try {
    {
      std::lock_guard<std::mutex> params_lock(param_handler_->getMutex());
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap_->getMutex());
      auto path = path_handler_->transformGlobalPlan(
        pose, params_->max_robot_pose_search_dist, params_->interpolate_curvature_after_goal);
      double lookahead = getLookAheadDistance(velocity);
      double curve_lookahead = params_->curvature_lookahead_dist;
      if (params_->allow_reversing) {
        const double cusp = findVelocitySignChange(path);
        lookahead = std::min(lookahead, cusp);
        curve_lookahead = std::min(curve_lookahead, cusp);
      }
      const auto carrot = getLookAheadPoint(lookahead, path);
      const double yaw = tf2::getYaw(pose.pose.orientation);
      reference_ = {pose.pose.position.x + std::cos(yaw) * carrot.pose.position.x - std::sin(yaw) * carrot.pose.position.y,
        pose.pose.position.y + std::sin(yaw) * carrot.pose.position.x + std::cos(yaw) * carrot.pose.position.y};
      const auto curve = params_->use_fixed_curvature_lookahead ?
        getLookAheadPoint(curve_lookahead, path, params_->interpolate_curvature_after_goal) : carrot;
      const auto & p = curve.pose.position;
      const double squared = p.x * p.x + p.y * p.y;
      const double curvature = squared > 0.001 ? 2.0 * p.y / squared : 0.0;
      const bool reverse = params_->allow_reversing && carrot.pose.position.x < 0.0;
      budget = std::min(params_->base_desired_linear_vel, reverse ? reverse_ : forward_);
      if (std::abs(curvature) > 1e-9) {
        budget = std::min(budget, angular_ / std::abs(curvature));
      }
      if (external_limit_ != nav2_costmap_2d::NO_SPEED_LIMIT) {
        const double external = external_percentage_ ?
          params_->base_desired_linear_vel * external_limit_ / 100.0 : external_limit_;
        budget = std::min(budget, std::max(0.0, external));
      }
    }
    RegulatedPurePursuitController::setSpeedLimit(budget, false);
    // Native RPP now regulates, calculates angular velocity and predicts
    // collisions using the matched budget. No velocity scaling after the check.
    auto command = RegulatedPurePursuitController::computeVelocityCommands(pose, velocity, checker);
    progress_.observe(rclcpp::Time(pose.header.stamp).seconds(),
      {pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)},
      std::abs(velocity.linear.x) > .01 && std::abs(command.twist.linear.x) > .01);
    publishTracking(pose, "OK");
    return command;
    } catch (const std::exception &) {
      progress_.failed(); publishTracking(pose, "BLOCKED"); throw;
    }
  }
private:
  void publishTracking(const geometry_msgs::msg::PoseStamped & pose, const std::string & state)
  {
    const double now = rclcpp::Time(pose.header.stamp).seconds();
    if (state == last_state_ && now >= last_report_ && now - last_report_ < .2) {return;}
    bool reversing;
    {std::lock_guard<std::mutex> lock(param_handler_->getMutex()); reversing = params_->allow_reversing;}
    nlohmann::json data{{"stamp", now}, {"controller", plugin_name_}, {"status", state},
      {"progress_epoch", progress_.epoch()}, {"frame", pose.header.frame_id},
      {"allow_reversing", reversing}, {"goal_frame", goal_.header.frame_id},
      {"goal_x", goal_.pose.position.x}, {"goal_y", goal_.pose.position.y}};
    if (reference_.size() == 2) {data["carrot_x"] = reference_[0]; data["carrot_y"] = reference_[1];}
    std_msgs::msg::String message; message.data = data.dump(); status_pub_->publish(message);
    last_report_ = now; last_state_ = state;
  }
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  TrackingProgress progress_;
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<double> reference_;
  double last_report_ = -1.;
  std::string last_state_;
  double forward_ = 0.6, reverse_ = 0.25, angular_ = 0.9;
  double external_limit_ = nav2_costmap_2d::NO_SPEED_LIMIT;
  bool external_percentage_ = false;
  std::mutex motion_mutex_;
};
}  // namespace edgeai_navigation
PLUGINLIB_EXPORT_CLASS(edgeai_navigation::MatchedRPP, nav2_core::Controller)
