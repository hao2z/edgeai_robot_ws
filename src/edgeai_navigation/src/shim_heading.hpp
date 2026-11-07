#pragma once
#include "nav2_core/controller_exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"

namespace edgeai_navigation
{
// Heading alignment only: use the latest common frame transform, with an
// explicit freshness bound, instead of asking for a not-yet-published future TF.
inline double shimHeadingError(tf2_ros::Buffer & buffer,
  const geometry_msgs::msg::PoseStamped & path_pose,
  const geometry_msgs::msg::PoseStamped & robot_pose, double tolerance)
{
  auto local = path_pose;
  if (path_pose.header.frame_id != robot_pose.header.frame_id) {
    try {
      const auto transform = buffer.lookupTransform(
        robot_pose.header.frame_id, path_pose.header.frame_id, tf2::TimePointZero);
      const auto & stamp = transform.header.stamp;
      if (stamp.sec != 0 || stamp.nanosec != 0) {
        const double age = rclcpp::Time(robot_pose.header.stamp).seconds() - rclcpp::Time(stamp).seconds();
        if (std::abs(age) > tolerance) {
          throw nav2_core::NoValidControl("Lattice Shim: frame transform outside freshness tolerance");
        }
      }
      tf2::doTransform(path_pose, local, transform);
    } catch (const tf2::TransformException & error) {
      // ControllerServer stops and retries NoValidControl within failure_tolerance.
      throw nav2_core::NoValidControl(std::string("Lattice Shim: waiting for frame transform: ") + error.what());
    }
  }
  return angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(local.pose.orientation));
}
}  // namespace edgeai_navigation
