"""Keep perception alive during an authority pause, without dispatching motion."""
from pathlib import Path
import shutil


def apply(output, replace):
    header='include/frontier_exploration_ros2/frontier_explorer_node.hpp'
    replace(output,header,'  FrontierExplorerCoreParams params_;',
        '  bool paused_map_processing_enabled_{false};\n'
        '  int last_monitor_count_{-2};\n'
        '  std::optional<std::chrono::steady_clock::time_point> last_monitor_publish_;\n'
        '  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr monitor_pub_;\n'
        '  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidates_pub_;\n'
        '  FrontierExplorerCoreParams params_;')
    replace(output,header,'#pragma once','#pragma once\n#include <std_msgs/msg/int32.hpp>\n'
        '#include <std_msgs/msg/string.hpp>\n#include <sstream>\n#include <locale>\n#include <cmath>')
    core='include/frontier_exploration_ros2/frontier_explorer_core.hpp'
    shutil.copyfile(Path(__file__).with_name('frontier_endpoint_clearance.hpp'),
                    output/'include/frontier_exploration_ros2/frontier_endpoint_clearance.hpp')
    replace(output,core,'  void try_send_next_goal();',
        '  void try_send_next_goal();\n'
        '  void diagnose_frontier_exhaustion(const geometry_msgs::msg::Pose & pose);\n'
        '''  FrontierEndpointClearance endpoint_clearance;
  FrontierSequence monitored_frontiers(const geometry_msgs::msg::Pose & pose) {
    commit_deferred_costmap_search_input_updates();
    // Optimization may expand free cells into raw unknown space. Passive
    // exporters need endpoints verified against the actual shared map.
    const auto search = callbacks.frontier_search(pose, *map, *costmap,
      local_costmap, 0.0, true);
    const auto frontiers = filter_frontiers_for_suppression(
      to_frontier_sequence(search.frontiers));
    FrontierSequence safe;
    // Use the exact projection used by send_frontier_goal, beginning with the
    // search's reachable neighbor. Inflated costmaps encode robot clearance.
    for (const auto & frontier : frontiers) {
      if (!frontier.goal_point || frontier.size <= 0) {continue;}
      const auto goal = build_dispatch_goal_pose(frontier, pose);
      const std::pair<double, double> point{goal.pose.position.x, goal.pose.position.y};
      if (!std::isfinite(point.first) || !std::isfinite(point.second)) {continue;}
      int x = 0, y = 0;
      if (!map->worldToMapNoThrow(point.first, point.second, x, y) ||
          map->getCost(x, y) != 0) {continue;}
      if (!costmap->worldToMapNoThrow(point.first, point.second, x, y)) {continue;}
      const int global_cost = costmap->getCost(x, y);
      if (global_cost < 0 || global_cost >= params.occ_threshold) {continue;}
      if (local_costmap && local_costmap->worldToMapNoThrow(point.first, point.second, x, y)) {
        const int local_cost = local_costmap->getCost(x, y);
        if (local_cost < 0 || local_cost >= params.occ_threshold) {continue;}
      }
      if (!endpoint_clearance.allows(*map, point.first, point.second, params.occ_threshold)) {continue;}
      auto projected = frontier;
      projected.goal_point = point;
      safe.push_back(projected);
    }
    return safe;
  }''')
    replace(output,core,'#pragma once','#pragma once\n#include <cmath>\n'
        '#include "frontier_exploration_ros2/frontier_endpoint_clearance.hpp"')
    node='src/frontier_explorer_node.cpp'
    replace(output,node,'  core_ = std::make_unique<FrontierExplorerCore>(params_, callbacks);',
        '  core_ = std::make_unique<FrontierExplorerCore>(params_, callbacks);\n'
        '  const auto footprint = declare_parameter("frontier_footprint", std::vector<double>{});\n'
        '  const auto headings = declare_parameter("frontier_heading_angles", std::vector<double>{});\n'
        '  core_->endpoint_clearance.configure(footprint, headings);')
    replace(output,node,': Node("frontier_explorer", options)\n{',
        ': Node("frontier_explorer", options)\n{\n'
        '  paused_map_processing_enabled_ = declare_parameter<bool>("paused_map_processing_enabled", false);\n'
        '  monitor_pub_ = create_publisher<std_msgs::msg::Int32>("/explore/available_frontiers", 1);\n'
        '  candidates_pub_ = create_publisher<std_msgs::msg::String>("explore/candidates", 1);')
    # An autostart=false detector has never entered startExplorationRuntime().
    # Subscribe directly without starting a session or enabling goal dispatch.
    replace(output,node,
        '    core_->stop_exploration_session("Frontier exploration initialized in cold idle mode");',
        '''    core_->stop_exploration_session("Frontier exploration initialized in cold idle mode");
    if (paused_map_processing_enabled_) {
      createMapSubscription(topic_qos_profiles_.map_durability);
      costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        params_.costmap_topic, topic_qos_profiles_.make_costmap_qos(),
        std::bind(&FrontierExplorerNode::costmapCallback, this, std::placeholders::_1));
      local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        params_.local_costmap_topic, topic_qos_profiles_.make_local_costmap_qos(),
        std::bind(&FrontierExplorerNode::localCostmapCallback, this, std::placeholders::_1));
      if (params_.map_processing_rate_hz > 0.0) {resetMapProcessingCalibrationWindow();}
    }''')
    replace(output,node,'  map_sub_.reset();\n  costmap_sub_.reset();\n  local_costmap_sub_.reset();\n'
        '  map_autodetect_timer_.reset();\n  map_processing_timer_.reset();',
        '  if (!paused_map_processing_enabled_) {\n'
        '    map_sub_.reset(); costmap_sub_.reset(); local_costmap_sub_.reset();\n'
        '    map_processing_timer_.reset();\n  }\n  map_autodetect_timer_.reset();')
    # Only sensor processing accepts cold idle; watchdog/control semantics stay unchanged.
    path=output/node;text=path.read_text()
    for method in ('occupancyGridCallback','costmapCallback','localCostmapCallback','mapProcessingTimerCallback'):
        begin=text.index('void FrontierExplorerNode::'+method+'(')
        end=text.index('\n}',begin)+2
        block=text[begin:end]
        before='runtime_state_ != RuntimeState::RUNNING'
        if block.count(before)!=1:raise RuntimeError('Pinned monitor gate changed: '+method)
        block=block.replace(before,'(runtime_state_ != RuntimeState::RUNNING &&\n'
            '      !(paused_map_processing_enabled_ && runtime_state_ == RuntimeState::COLD_IDLE))')
        text=text[:begin]+block+text[end:]
    path.write_text(text)
    replace(output,node,'  core_->processPendingMapUpdate();\n}',
        '  core_->processPendingMapUpdate();\n'
        '  if (paused_map_processing_enabled_) {\n'
        '    const auto monitor_now = std::chrono::steady_clock::now();\n'
        '    if (last_monitor_publish_ && monitor_now - *last_monitor_publish_ < std::chrono::milliseconds(500)) {return;}\n'
        '    std_msgs::msg::Int32 status; status.data = -1;\n'
        '    FrontierSequence frontiers;\n'
        '    bool available = false;\n'
        '    const auto pose = core_->callbacks.get_current_pose();\n'
        '    if (pose && core_->map && core_->costmap && !core_->goal_in_progress) {\n'
        '      try {\n'
        '        frontiers = core_->monitored_frontiers(*pose);\n'
        '        status.data = static_cast<int>(frontiers.size());\n'
        '        available = runtime_state_ == RuntimeState::COLD_IDLE;\n'
        '        core_->publish_frontier_markers(frontiers);\n'
        '      } catch (const std::exception &) {}\n'
        '    }\n'
        '    if (status.data > 0) {\n'
        '      completion_event_published_ = false;\n'
        '      if (runtime_state_ == RuntimeState::RUNNING && core_->return_to_start_completed &&\n'
        '          !core_->goal_in_progress) {\n'
        '        core_->return_to_start_completed = false;\n'
        '        RCLCPP_INFO(get_logger(), "FRONTIER_REOPENED: shared map exposes new candidates; resuming automatic dispatch");\n'
        '        core_->try_send_next_goal();\n'
        '      }\n    }\n'
        '''    std::ostringstream json;
    json.imbue(std::locale::classic()); json.precision(17);
    json << "{\\"frame_id\\":\\"" << params_.global_frame << "\\",\\"stamp_ns\\":"
         << get_clock()->now().nanoseconds() << ",\\"available\\":"
         << (available ? "true" : "false") << ",\\"candidates\\":[";
    bool first = true;
    if (available) {
      for (const auto & frontier : frontiers) {
        if (!first) {json << ',';} first = false;
        const auto point = *frontier.goal_point;
        json << "{\\"x\\":" << point.first << ",\\"y\\":" << point.second
             << ",\\"size\\":" << frontier.size << '}';
      }
    }
    json << "]}";
    std_msgs::msg::String snapshot; snapshot.data = json.str();
    candidates_pub_->publish(snapshot);
    last_monitor_publish_ = std::chrono::steady_clock::now();
'''
        '    monitor_pub_->publish(status);\n'
        '    if (status.data != last_monitor_count_) {\n'
        '      RCLCPP_INFO(get_logger(), "FRONTIER_MONITOR: available=%d; active_goal=%d", status.data, core_->goal_in_progress);\n'
        '      last_monitor_count_ = status.data;\n'
        '    }\n  }\n}')
    dispatch='src/frontier_explorer_core_dispatch.cpp'
    replace(output,dispatch,'      callbacks.log_info("No more frontiers found");',
        '      diagnose_frontier_exhaustion(*current_pose);\n'
        '      callbacks.log_info("No more frontiers found");')
    path=output/dispatch
    text=path.read_text()
    text='#include <filesystem>\n#include <fstream>\n#include <cstdlib>\n'+text
    text+='\n'+Path(__file__).with_name('frontier_diagnostics.cpp.inc').read_text()
    path.write_text(text)
