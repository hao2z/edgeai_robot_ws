#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "shim_heading.hpp"

namespace edgeai_navigation
{
// Keep native Shim acceleration/collision checks, but align vehicle orientation
// from the feasible path, rather than facing a point that may be behind us.
// Unlike Jazzy's orientation branch, both yaws are transformed into one frame.
class LatticeRotationShim : public nav2_rotation_shim_controller::RotationShimController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    RotationShimController::configure(parent, name, tf, map);
    auto node = parent.lock();
    nav2_util::declare_parameter_if_not_declared(node, name + ".transform_tolerance", rclcpp::ParameterValue(0.3));
    node->get_parameter(name + ".transform_tolerance", frame_tolerance_);
    if (!(frame_tolerance_ > 0.0) || !std::isfinite(frame_tolerance_)) {
      throw std::runtime_error("Lattice Shim transform_tolerance must be finite and positive");
    }
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * checker) override
  {
    if (path_updated_) {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(
        *costmap_ros_->getCostmap()->getMutex());
      std::lock_guard<std::mutex> lock(mutex_);
      auto sampled = getSampledPathPt();
      const double error = shimHeadingError(*tf_, sampled, pose, frame_tolerance_);
      const double threshold = in_rotation_ ? angular_disengage_threshold_ : angular_dist_threshold_;
      if (std::abs(error) > threshold) {
        in_rotation_ = true;
        auto command = computeRotateToHeadingCommand(error, pose, velocity);
        last_angular_vel_ = command.twist.angular.z;
        return command;
      }
      path_updated_ = false;
    }
    in_rotation_ = false;
    auto command = primary_controller_->computeVelocityCommands(pose, velocity, checker);
    last_angular_vel_ = command.twist.angular.z;
    return command;
  }
private:
  double frame_tolerance_ = 0.3;
};
}  // namespace edgeai_navigation
PLUGINLIB_EXPORT_CLASS(edgeai_navigation::LatticeRotationShim, nav2_core::Controller)
