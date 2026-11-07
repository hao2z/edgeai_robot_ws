#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <set>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "recovery_geometry.hpp"
#include "recovery_history.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "angles/angles.h"

namespace edgeai_navigation
{
class InvalidatePath : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {BT::OutputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus tick() override
  {setOutput("path", nav_msgs::msg::Path{}); return BT::NodeStatus::SUCCESS;}
};

class PathNearRobot : public BT::ConditionNode
{
public:
  PathNearRobot(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config), tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer")) {}
  static BT::PortsList providedPorts()
  {return {BT::InputPort<nav_msgs::msg::Path>("path"), BT::InputPort<double>("max_distance", .25, "Path deviation limit"), BT::InputPort<std::string>("robot_frame", "base_footprint", "Robot axle frame")};}
  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput("path", path) || path.poses.empty()) {return BT::NodeStatus::FAILURE;}
    try {
      auto pose = tf_->lookupTransform(path.header.frame_id, getInput<std::string>("robot_frame").value(), tf2::TimePointZero);
      double distance = INFINITY;
      for (const auto & p : path.poses) {
        distance = std::min(distance, std::hypot(p.pose.position.x - pose.transform.translation.x,
            p.pose.position.y - pose.transform.translation.y));
      }
      return distance <= getInput<double>("max_distance").value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (const tf2::TransformException &) {return BT::NodeStatus::FAILURE;}
  }
private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

class PlannerStartOccupied : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<uint16_t>("error_code")};}
  BT::NodeStatus tick() override
  {auto error = getInput<uint16_t>("error_code"); return error && error.value() == 205 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;}
};

class SelectRecovery : public BT::StatefulActionNode
{
public:
  SelectRecovery(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config), node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
    tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"))
  {
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options; options.callback_group = group_;
    map_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>("local_costmap/costmap_raw", 1,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {map_ = msg;}, options);
    footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>("local_costmap/published_footprint", 1,
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {footprint_ = msg;}, options);
    rpp_sub_ = node_->create_subscription<std_msgs::msg::String>("navigation/rpp_tracking", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        try {
          auto parsed = nlohmann::json::parse(msg->data);
          if (!parsed.is_object()) {return;}
          (void)parsed.value<double>("stamp", -100.); (void)parsed.value<uint64_t>("progress_epoch", 0);
          (void)parsed.value<std::string>("frame", ""); (void)parsed.value<std::string>("goal_frame", "");
          (void)parsed.value<std::string>("controller", "");
          (void)parsed.value<double>("goal_x", -100.); (void)parsed.value<double>("goal_y", -100.);
          (void)parsed.value<double>("carrot_x", 0.); (void)parsed.value<double>("carrot_y", 0.);
          (void)parsed.value<bool>("allow_reversing", false); rpp_ = std::move(parsed);
        } catch (const nlohmann::json::exception &) {rpp_ = nlohmann::json::object();}
      }, options);
    const auto actual_text = getInput<std::string>("actual_footprint").value();
    if (actual_text != "[]") {nav2_costmap_2d::makeFootprintFromString(actual_text, actual_);}
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("recovery_candidate_path", 1);
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("robot_frame", "base_footprint", "Axle frame"),
      BT::InputPort<std::string>("distances", "0.05;0.10", "Short translations in metres"),
      BT::InputPort<std::string>("angles", "0.20;0.35", "Short rotations in radians"),
      BT::InputPort<double>("max_age", .35, "Maximum map/footprint/pose age"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::string>("controller_id", "", "Expected RPP feedback source"),
      BT::InputPort<std::string>("actual_footprint", "[]", "Unpadded physical body from runtime config"),
      BT::OutputPort<std::string>("kind"), BT::OutputPort<double>("distance"), BT::OutputPort<double>("angle")};
  }
  BT::NodeStatus onStart() override
  {started_ = std::chrono::steady_clock::now(); return onRunning();}
  BT::NodeStatus onRunning() override
  {
    executor_.spin_some();
    const double max_age = getInput<double>("max_age").value();
    auto fresh = [&](const builtin_interfaces::msg::Time & stamp) {
        const double age = (node_->now() - rclcpp::Time(stamp)).seconds();
        return age >= 0.0 && age <= max_age;
      };
    if (!map_ || !footprint_ || !fresh(map_->header.stamp) || !fresh(footprint_->header.stamp)) {return waiting();}
    if (map_->metadata.resolution <= 0 || map_->metadata.size_x == 0 || map_->metadata.size_y == 0 ||
      map_->data.size() != static_cast<size_t>(map_->metadata.size_x) * map_->metadata.size_y)
    {return BT::NodeStatus::FAILURE;}
    try {
      const auto robot_frame = getInput<std::string>("robot_frame").value();
      const auto current = tf_->lookupTransform(map_->header.frame_id, robot_frame, tf2::TimePointZero);
      if (!fresh(current.header.stamp)) {return waiting();}
      const auto old = tf_->lookupTransform(footprint_->header.frame_id, robot_frame,
        rclcpp::Time(footprint_->header.stamp));
      const double yaw = tf2::getYaw(old.transform.rotation), c = std::cos(yaw), s = std::sin(yaw);
      Polygon body;
      for (const auto & point : footprint_->polygon.points) {
        geometry_msgs::msg::Point local;
        const double dx = point.x - old.transform.translation.x, dy = point.y - old.transform.translation.y;
        local.x = c * dx + s * dy; local.y = -s * dx + c * dy; body.push_back(local);
      }
      const auto & m = map_->metadata;
      nav2_costmap_2d::Costmap2D grid(m.size_x, m.size_y, m.resolution, m.origin.position.x, m.origin.position.y, 255);
      std::memcpy(grid.getCharMap(), map_->data.data(), map_->data.size());
      const RecoveryPose pose{current.transform.translation.x, current.transform.translation.y,
        tf2::getYaw(current.transform.rotation)};
      geometry_msgs::msg::PoseStamped goal;
      getInput("goal", goal);
      if (!has_goal_ || goal.header.frame_id != previous_goal_.header.frame_id ||
        std::hypot(goal.pose.position.x - previous_goal_.pose.position.x, goal.pose.position.y - previous_goal_.pose.position.y) > .1 ||
        std::abs(angles::shortest_angular_distance(tf2::getYaw(previous_goal_.pose.orientation), tf2::getYaw(goal.pose.orientation))) > .35)
      {
        history_.clear(); previous_goal_ = goal; has_goal_ = true; progress_epoch_.reset();
      }
      const double rpp_age = node_->now().seconds() - rpp_.value("stamp", -100.);
      const bool rpp_current = rpp_.is_object() && rpp_age >= 0.0 && rpp_age <= 2.0 &&
        (getInput<std::string>("controller_id").value().empty() ||
        rpp_.value("controller", "") == getInput<std::string>("controller_id").value()) &&
        rpp_.value("goal_frame", "") == goal.header.frame_id &&
        std::hypot(rpp_.value("goal_x", -100.) - goal.pose.position.x,
        rpp_.value("goal_y", -100.) - goal.pose.position.y) <= .15;
      if (rpp_current) {
        const auto epoch = rpp_.value<uint64_t>("progress_epoch", 0);
        if (progress_epoch_ && epoch > *progress_epoch_) {
          RCLCPP_INFO(node_->get_logger(), "RECOVERY_HISTORY_RESET: sustained measured RPP tracking; previous attempts %zu", history_.size());
          history_.clear();
        }
        progress_epoch_ = epoch;
      }
      const bool reference = rpp_current && rpp_.value("frame", "") == map_->header.frame_id &&
        rpp_.contains("carrot_x") && rpp_.contains("carrot_y");
      const bool occupied = !poseFree(grid, body, pose);
      const auto overlap = occupiedOverlap(grid, body, pose);
      if (occupied && (actual_.size() < 3 || !overlap.valid)) {
        RCLCPP_WARN(node_->get_logger(), "RECOVERY_NO_SAFE_MOTION: %s",
          actual_.size() < 3 ? "actual footprint unavailable" :
          "footprint intersects unknown cells or map boundary");
        return BT::NodeStatus::FAILURE;
      }
      std::vector<RecoveryMotion> candidates;
      for (const auto distance : BT::convertFromString<std::vector<double>>(getInput<std::string>("distances").value())) {
        if (distance <= 0.0 || distance > .2) {throw std::runtime_error("Recovery translation must be in (0,0.2] m");}
        candidates.push_back({"backup", distance}); candidates.push_back({"forward", distance});
      }
      for (const auto angle : BT::convertFromString<std::vector<double>>(getInput<std::string>("angles").value())) {
        if (angle <= 0.0 || angle > .6) {throw std::runtime_error("Recovery rotation must be in (0,0.6] rad");}
        candidates.push_back({"spin", angle}); candidates.push_back({"spin", -angle});
      }
      std::optional<RecoveryMotion> best; double best_score = -INFINITY; bool best_connection = false;
      for (const auto & motion : candidates) {
        if (history_.tried(pose, motion)) {continue;}
        if (occupied ? (motion.amount > .10 || !occupiedEscapeFree(grid, actual_, body, pose, motion, overlap)) :
          !motionFree(grid, body, pose, motion)) {continue;}
        const auto end = advanced(pose, motion, 1.0);
        const bool connection = reference && rppConnectionFree(grid, body, end,
          rpp_["carrot_x"].get<double>(), rpp_["carrot_y"].get<double>(), rpp_.value("allow_reversing", false));
        const double score = clearance(grid, body, end) + (motion.kind == "spin" ? 0.0 : .02) +
          (motion.kind == "backup" ? .01 : 0.0) + std::min(.1, std::abs(motion.amount)) * .01;
        if (!best || (connection && !best_connection) || (connection == best_connection && score > best_score)) {
          best = motion; best_score = score; best_connection = connection;
        }
      }
      if (!best) {
        RCLCPP_WARN(node_->get_logger(), "RECOVERY_NO_SAFE_MOTION: no untried swept-footprint candidate");
        return BT::NodeStatus::FAILURE;
      }
      history_.add(pose, *best);
      setOutput("kind", occupied ? std::string("occupied") : best->kind);
      setOutput("distance", best->kind == "spin" ? 0.0 : (occupied && best->kind == "backup" ? -best->amount : best->amount));
      setOutput("angle", best->kind == "spin" ? best->amount : 0.0);
      nav_msgs::msg::Path path; path.header = map_->header; path.header.stamp = node_->now();
      for (int i = 0; i <= 20; ++i) {
        const auto point = advanced(pose, *best, i / 20.0); geometry_msgs::msg::PoseStamped p; p.header = path.header;
        p.pose.position.x = point.x; p.pose.position.y = point.y;
        p.pose.orientation.z = std::sin(point.yaw / 2); p.pose.orientation.w = std::cos(point.yaw / 2);
        path.poses.push_back(p);
      }
      path_pub_->publish(path);
      RCLCPP_INFO(node_->get_logger(), "RECOVERY_SELECTED: %s %s %.3f; history %zu; RPP short-arc %s; map age %.3f s",
        occupied ? (poseFree(grid, actual_, pose) ? "padding_escape" : "body_overlap_escape") : "free", best->kind.c_str(), best->amount, history_.size(),
        !reference ? "unavailable" : best_connection ? "clear" : "blocked",
        (node_->now() - rclcpp::Time(map_->header.stamp)).seconds());
      return BT::NodeStatus::SUCCESS;
    } catch (const tf2::TransformException &) {return waiting();}
  }
  void onHalted() override {}
private:
  BT::NodeStatus waiting()
  {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count() < .6) {return BT::NodeStatus::RUNNING;}
    RCLCPP_WARN(node_->get_logger(), "RECOVERY_NO_SAFE_MOTION: fresh local map, footprint or paired TF unavailable");
    return BT::NodeStatus::FAILURE;
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpp_sub_;
  nlohmann::json rpp_ = nlohmann::json::object();
  Polygon actual_;
  nav2_msgs::msg::Costmap::ConstSharedPtr map_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::chrono::steady_clock::time_point started_;
  std::optional<uint64_t> progress_epoch_;
  bool has_goal_ = false;
  geometry_msgs::msg::PoseStamped previous_goal_;
  RecoveryHistory history_;
};
}  // namespace edgeai_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<edgeai_navigation::InvalidatePath>("InvalidatePath");
  factory.registerNodeType<edgeai_navigation::PathNearRobot>("PathNearRobot");
  factory.registerNodeType<edgeai_navigation::PlannerStartOccupied>("PlannerStartOccupied");
  factory.registerNodeType<edgeai_navigation::SelectRecovery>("SelectRecovery");
}
