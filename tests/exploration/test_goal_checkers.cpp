// Test the installed Nav2 plugins, including their actual yaw/stop semantics.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_core/goal_checker.hpp>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

void require(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int result=0;
  try {
    auto options=rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("ordinary.xy_goal_tolerance",0.1),
      rclcpp::Parameter("heading.xy_goal_tolerance",0.1),
      rclcpp::Parameter("heading.yaw_goal_tolerance",0.25),
      rclcpp::Parameter("final.xy_goal_tolerance",0.1),
      rclcpp::Parameter("final.yaw_goal_tolerance",0.25),
      rclcpp::Parameter("final.trans_stopped_velocity",0.02),
      rclcpp::Parameter("final.rot_stopped_velocity",0.05)});
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_checker_test","",options);
    pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core","nav2_core::GoalChecker");
    auto ordinary=loader.createSharedInstance("nav2_controller::PositionGoalChecker");
    auto heading=loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
    auto rescue=loader.createSharedInstance("nav2_controller::StoppedGoalChecker");
    ordinary->initialize(node,"ordinary",nullptr);
    heading->initialize(node,"heading",nullptr);
    rescue->initialize(node,"final",nullptr);
    geometry_msgs::msg::Pose actual,goal;
    geometry_msgs::msg::Twist stopped,moving;
    actual.position.x=0.05; actual.orientation.w=1.; goal.orientation.z=1.;
    require(ordinary->isGoalReached(actual,goal,stopped),"Ordinary goal must ignore 180 degree yaw error");
    require(!heading->isGoalReached(actual,goal,stopped),"Heading goal must reject wrong yaw");
    require(!rescue->isGoalReached(actual,goal,stopped),"Final rescue must reject wrong yaw");
    ordinary->reset();actual.position.x=0.11;
    require(!ordinary->isGoalReached(actual,goal,stopped),"Ordinary goal must still enforce XY tolerance");
    actual.position.x=0.05;goal.orientation.z=0.;goal.orientation.w=1.;moving.linear.x=0.1;
    heading->reset();
    require(heading->isGoalReached(actual,goal,moving),"Heading goal must accept matching pose without rescue stop requirement");
    rescue->reset();
    require(!rescue->isGoalReached(actual,goal,moving),"Final rescue must require stopping");
    require(rescue->isGoalReached(actual,goal,stopped),"Final rescue must accept correct stopped pose");
    std::cout << "PASS installed Nav2 goal checkers: position, heading, rescue yaw/stop" << std::endl;
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl; result=1;
  }
  rclcpp::shutdown();
  return result;
}
