"""Keep perception alive during an authority pause, without dispatching motion."""
from pathlib import Path


def apply(output, replace):
    header='include/frontier_exploration_ros2/frontier_explorer_node.hpp'
    replace(output,header,'  FrontierExplorerCoreParams params_;',
        '  bool paused_map_processing_enabled_{false};\n'
        '  int last_monitor_count_{-2};\n'
        '  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr monitor_pub_;\n'
        '  FrontierExplorerCoreParams params_;')
    replace(output,header,'#pragma once','#pragma once\n#include <std_msgs/msg/int32.hpp>')
    core='include/frontier_exploration_ros2/frontier_explorer_core.hpp'
    replace(output,core,'  void try_send_next_goal();',
        '  void try_send_next_goal();\n'
        '  void diagnose_frontier_exhaustion(const geometry_msgs::msg::Pose & pose);\n'
        '  FrontierSequence monitored_frontiers(const geometry_msgs::msg::Pose & pose) {\n'
        '    return filter_frontiers_for_suppression(get_frontier_snapshot(pose, 0.0).frontiers);\n'
        '  }')
    node='src/frontier_explorer_node.cpp'
    replace(output,node,': Node("frontier_explorer", options)\n{',
        ': Node("frontier_explorer", options)\n{\n'
        '  paused_map_processing_enabled_ = declare_parameter<bool>("paused_map_processing_enabled", false);\n'
        '  monitor_pub_ = create_publisher<std_msgs::msg::Int32>("/explore/available_frontiers", 1);')
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
        '  if (paused_map_processing_enabled_ && runtime_state_ == RuntimeState::COLD_IDLE) {\n'
        '    std_msgs::msg::Int32 status; status.data = -1;\n'
        '    const auto pose = core_->callbacks.get_current_pose();\n'
        '    if (pose && core_->map && core_->costmap && !core_->goal_in_progress) {\n'
        '      try {\n'
        '        const auto frontiers = core_->monitored_frontiers(*pose);\n'
        '        status.data = static_cast<int>(frontiers.size());\n'
        '        core_->publish_frontier_markers(frontiers);\n'
        '      } catch (const std::out_of_range &) {}\n'
        '    }\n'
        '    monitor_pub_->publish(status);\n'
        '    if (status.data != last_monitor_count_) {\n'
        '      RCLCPP_INFO(get_logger(), "FRONTIER_MONITOR: paused navigation dispatch; available=%d", status.data);\n'
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
