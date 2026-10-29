"""Generate a pinned upstream build with an optional outer progress watchdog."""
from pathlib import Path
import shutil
import sys


def replace_once(root, name, before, after):
    path = root / name
    text = path.read_text(encoding='utf-8')
    if text.count(before) != 1:
        raise RuntimeError(f'Pinned upstream changed: {name}')
    path.write_text(text.replace(before, after), encoding='utf-8', newline='\n')


def prepare(source, output):
    source, output = source.resolve(), output.resolve()
    if source == output or source in output.parents or output in source.parents:
        raise ValueError('Source and generated build source must be separate directories')
    shutil.copytree(source, output, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('.git', '__pycache__'))
    replace_once(output, 'src/frontier_explorer_core_dispatch.cpp',
                 '      if (goal_kind == "frontier") {\n'
                 '        record_failed_frontier_attempt(context.has_value() ? context->frontier : std::optional<FrontierLike>{});\n'
                 '      }\n'
                 '      callbacks.log_warn(',
                 '      if (goal_kind == "frontier") {\n'
                 '        record_failed_frontier_attempt(context.has_value() ? context->frontier : std::optional<FrontierLike>{});\n'
                 '      }\n'
                 '      callbacks.log_warn(')
    # Low information gain must pass the existing replacement-membership and
    # equivalent-sequence checks before canceling an accepted navigation task.
    dispatch = output / 'src/frontier_explorer_core_dispatch.cpp'
    text = dispatch.read_text()
    start = text.index('    const std::string visible_gain_preemption_reason =')
    end = text.index('  // THIS PART MIGHT BE PROBLEMATIC FOR CPU USAGE', start)
    text = text[:start] + '  }\n\n' + text[end:]
    dispatch.write_text(text, encoding='utf-8', newline='\n')
    replace_once(output, 'src/frontier_explorer_core_init.cpp',
                 'params.frontier_suppression_no_progress_timeout_s = std::max(\n    0.1,',
                 'params.frontier_suppression_no_progress_timeout_s = std::max(\n    0.0,')
    replace_once(output, 'src/frontier_explorer_core_suppression.cpp',
                 'bool FrontierExplorerCore::evaluate_active_goal_progress_timeout()\n{\n',
                 'bool FrontierExplorerCore::evaluate_active_goal_progress_timeout()\n{\n'
                 '  // Zero delegates progress and recovery to Nav2 until a terminal result.\n'
                 '  if (params.frontier_suppression_no_progress_timeout_s <= 0.0) {\n'
                 '    return false;\n  }\n')
    test = '''TEST(FrontierSuppressionCoreTests, Nav2OwnsRecoveryUntilTerminalResult)
{
  int64_t now_ns = 0;
  int dispatch_calls = 0;
  auto core = make_suppression_core(&now_ns, &dispatch_calls);
  core->params.frontier_suppression_no_progress_timeout_s = 0.0;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_complete_if_within_m = 0.0;
  core->try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  const int dispatch = core->current_dispatch_id;
  auto handle = std::make_shared<FakeGoalHandle>();
  core->goal_response_callback(dispatch, handle, true, "");
  core->feedback_callback(10.0, dispatch);
  now_ns = 900'000'000'000;
  core->feedback_callback(11.0, dispatch);
  EXPECT_FALSE(core->evaluate_active_goal_progress_timeout());
  auto blocked = build_grid(20, 20, 100);
  core->localCostmapCallback(OccupancyGrid2d(blocked));
  core->costmapCallback(OccupancyGrid2d(blocked));
  core->consider_preempt_active_goal("map");
  EXPECT_EQ(handle->cancel_calls, 0);
  EXPECT_EQ(core->current_dispatch_id, dispatch);
  EXPECT_EQ(dispatch_calls, 1);
  core->get_result_callback(dispatch, action_msgs::msg::GoalStatus::STATUS_ABORTED,
                            208, "recovery exhausted");
  EXPECT_EQ(core->suppressed_region_count(), 1U);
}

'''
    test += '''TEST(FrontierSuppressionCoreTests, OccupiedStartDoesNotPauseExploration)
{
  int64_t now_ns = 0;
  int dispatch_calls = 0;
  auto core = make_suppression_core(&now_ns, &dispatch_calls);
  core->try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  const int dispatch = core->current_dispatch_id;
  core->goal_response_callback(dispatch, std::make_shared<FakeGoalHandle>(), true, "");
  core->get_result_callback(dispatch, action_msgs::msg::GoalStatus::STATUS_ABORTED,
                            205, "Start occupied");
  EXPECT_TRUE(core->exploration_enabled);
  EXPECT_EQ(core->suppressed_region_count(), 1U);
  core->try_send_next_goal();
}

'''
    anchor = 'TEST(FrontierSuppressionCoreTests, NoProgressTimeoutCanceledGoalCreatesSuppressedRegion)'
    replace_once(output, 'test/test_frontier_suppression.cpp', anchor, test + anchor)


if __name__ == '__main__':
    prepare(*map(Path, sys.argv[1:3]))
