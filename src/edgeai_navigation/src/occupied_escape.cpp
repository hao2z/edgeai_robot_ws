#include <chrono>
#include <cstring>
#include <mutex>
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "recovery_geometry.hpp"

namespace edgeai_navigation
{
class OccupiedEscape : public nav2_behaviors::TimedBehavior<nav2_msgs::action::DriveOnHeading>
{
  using Action = nav2_msgs::action::DriveOnHeading;
  using Result = nav2_behaviors::ResultStatus;
  using Status = nav2_behaviors::Status;
  using Steady = std::chrono::steady_clock;
public:
  nav2_core::CostmapInfoType getResourceInfo() override {return nav2_core::CostmapInfoType::LOCAL;}
  void onConfigure() override
  {
    auto node = node_.lock();
    std::string text;
    nav2_util::declare_parameter_if_not_declared(node, behavior_name_ + ".actual_footprint", rclcpp::ParameterValue("[]"));
    nav2_util::declare_parameter_if_not_declared(node, behavior_name_ + ".footprint_padding", rclcpp::ParameterValue(.01));
    node->get_parameter(behavior_name_ + ".actual_footprint", text);
    double padding; node->get_parameter(behavior_name_ + ".footprint_padding", padding);
    if (text != "[]") {nav2_costmap_2d::makeFootprintFromString(text, actual_);}
    padded_ = actual_;
    if (padding >= 0.0 && std::isfinite(padding)) {nav2_costmap_2d::padFootprint(padded_, padding);}
    else {actual_.clear(); padded_.clear();}
    map_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>("local_costmap/costmap_raw", 1,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr message) {std::lock_guard<std::mutex> lock(mutex_); map_ = message;});
  }
  Result onRun(const std::shared_ptr<const Action::Goal> command) override
  {
    const double distance = std::abs(command->target.x);
    if (actual_.size() < 3 || distance <= 0.0 || distance > .100001 ||
      !std::isfinite(distance) || !std::isfinite(command->speed) || command->speed == 0.0 ||
      command->target.y != 0.0 || command->target.z != 0.0 ||
      rclcpp::Duration(command->time_allowance).seconds() <= 0.0)
    {return stop(Action::Result::INVALID_INPUT);}
    speed_ = std::copysign(std::min(.03, std::abs(static_cast<double>(command->speed))), command->target.x);
    motion_ = {command->target.x < 0.0 ? "backup" : "forward", distance};
    nav2_costmap_2d::Costmap2D grid;
    if (!snapshot(grid, initial_)) {return stop(Action::Result::TF_ERROR);}
    allowed_ = occupiedOverlap(grid, padded_, initial_);
    actual_allowed_ = occupiedOverlap(grid, actual_, initial_);
    if (!occupiedEscapeFree(grid, actual_, padded_, initial_, motion_, allowed_)) {
      RCLCPP_WARN(logger_, "OCCUPIED_ESCAPE_REJECTED: require known cells, no new obstacles, nonincreasing actual and padded overlap, reduced overlap or clear endpoint");
      return stop(Action::Result::COLLISION_AHEAD);
    }
    previous_area_ = allowed_.area;
    previous_actual_area_ = occupiedOverlap(grid, actual_, initial_).area; last_distance_ = 0.;
    start_wall_ = last_motion_ = Steady::now();
    end_ = clock_->now() + rclcpp::Duration(command->time_allowance);
    RCLCPP_INFO(logger_, "OCCUPIED_ESCAPE_STARTED: signed distance %.3f m, speed %.3f m/s; padded overlap %.6f m2, actual overlap %.6f m2",
      command->target.x, speed_, allowed_.area, previous_actual_area_);
    return {Status::SUCCEEDED, Action::Result::NONE};
  }
  Result onCycleUpdate() override
  {
    if (clock_->now() > end_ || std::chrono::duration<double>(Steady::now() - start_wall_).count() > 24.) {
      return stop(Action::Result::TIMEOUT);
    }
    nav2_costmap_2d::Costmap2D grid; RecoveryPose pose;
    if (!snapshot(grid, pose)) {return stop(Action::Result::TF_ERROR);}
    const double distance = std::hypot(pose.x - initial_.x, pose.y - initial_.y);
    auto feedback = std::make_shared<Action::Feedback>(); feedback->distance_traveled = distance;
    action_server_->publish_feedback(feedback);
    if (poseFree(grid, actual_, pose) && poseFree(grid, padded_, pose)) {
      stopRobot(); RCLCPP_INFO(logger_, "OCCUPIED_ESCAPE_COMPLETED: padded footprint is clear");
      return {Status::SUCCEEDED, Action::Result::NONE};
    }
    const auto current = occupiedOverlap(grid, padded_, pose);
    const auto physical = occupiedOverlap(grid, actual_, pose);
    if (distance >= motion_.amount) {
      if (current.valid && physical.valid && current.area < allowed_.area - 1e-8 &&
        current.area <= previous_area_ + 1e-8 && physical.area <= previous_actual_area_ + 1e-8 &&
        std::includes(allowed_.cells.begin(), allowed_.cells.end(), current.cells.begin(), current.cells.end()) &&
        std::includes(actual_allowed_.cells.begin(), actual_allowed_.cells.end(), physical.cells.begin(), physical.cells.end())) {
        stopRobot();
        RCLCPP_INFO(logger_, "OCCUPIED_ESCAPE_PROGRESS: overlap reduced %.6f -> %.6f m2; replan from measured pose",
          allowed_.area, current.area);
        return {Status::SUCCEEDED, Action::Result::NONE};
      }
      return stop(Action::Result::COLLISION_AHEAD);
    }
    RecoveryMotion remaining{motion_.kind, motion_.amount - distance};
    if (!current.valid || !physical.valid || physical.area > previous_actual_area_ + 1e-8 || current.area > previous_area_ + 1e-8 ||
      !occupiedEscapeFree(grid, actual_, padded_, pose, remaining, allowed_, &actual_allowed_)) {
      return stop(Action::Result::COLLISION_AHEAD);
    }
    previous_area_ = current.area;
    previous_actual_area_ = physical.area;
    if (distance > last_distance_ + .002) {last_distance_ = distance; last_motion_ = Steady::now();}
    if (std::chrono::duration<double>(Steady::now() - last_motion_).count() > 1.5) {
      RCLCPP_WARN(logger_, "OCCUPIED_ESCAPE_STOPPED: no measured translation; downstream safety may be stopping motion");
      return stop(Action::Result::TIMEOUT);
    }
    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header.stamp = clock_->now(); command->header.frame_id = robot_base_frame_;
    command->twist.linear.x = speed_; vel_pub_->publish(std::move(command));
    return {Status::RUNNING, Action::Result::NONE};
  }
private:
  Result stop(uint16_t code) {stopRobot(); return {Status::FAILED, code};}
  bool snapshot(nav2_costmap_2d::Costmap2D & grid, RecoveryPose & pose)
  {
    nav2_msgs::msg::Costmap::ConstSharedPtr map;
    {std::lock_guard<std::mutex> lock(mutex_); map = map_;}
    if (!map) {return false;}
    const auto & m = map->metadata;
    const double age = (clock_->now() - rclcpp::Time(map->header.stamp)).seconds();
    if (age < 0.0 || age > .35 || m.resolution <= 0.0 || m.size_x == 0 || m.size_y == 0 ||
      map->data.size() != static_cast<size_t>(m.size_x) * m.size_y) {return false;}
    try {
      const auto current = tf_->lookupTransform(map->header.frame_id, robot_base_frame_, tf2::TimePointZero);
      const double tf_age = (clock_->now() - rclcpp::Time(current.header.stamp)).seconds();
      if (tf_age < 0.0 || tf_age > .35) {return false;}
      pose = {current.transform.translation.x, current.transform.translation.y, tf2::getYaw(current.transform.rotation)};
    } catch (const tf2::TransformException &) {return false;}
    grid.resizeMap(m.size_x, m.size_y, m.resolution, m.origin.position.x, m.origin.position.y);
    std::memcpy(grid.getCharMap(), map->data.data(), map->data.size()); return true;
  }
  std::mutex mutex_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr map_sub_;
  nav2_msgs::msg::Costmap::ConstSharedPtr map_;
  Polygon actual_, padded_;
  RecoveryPose initial_{};
  RecoveryMotion motion_;
  OccupancyOverlap allowed_, actual_allowed_;
  double speed_ = 0.0, previous_area_ = 0.0, previous_actual_area_ = 0.0, last_distance_ = 0.0;
  rclcpp::Time end_{0, 0, RCL_ROS_TIME};
  Steady::time_point start_wall_, last_motion_;
};
}  // namespace edgeai_navigation
PLUGINLIB_EXPORT_CLASS(edgeai_navigation::OccupiedEscape, nav2_core::Behavior)
