TEST(RepeatGuardTests, SuccessAndCancelWithoutGainBackOffAcrossCooldownExpiry)
{
  int64_t now = 0;
  int sends = 0;
  std::vector<std::string> logs;
  auto core = make_suppression_core(&now, &sends, &logs);
  core->params.repeat_guard_enabled = true;
  core->params.frontier_suppression_enabled = false;
  core->params.post_goal_min_settle = 1.0;
  core->try_send_next_goal();
  ASSERT_EQ(sends, 1);
  core->goal_response_callback(core->current_dispatch_id, std::make_shared<FakeGoalHandle>(), true, "");
  core->get_result_callback(core->current_dispatch_id, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED, 0, "", "");
  now = 2'000'000'000;
  core->try_send_next_goal();
  EXPECT_EQ(sends, 1);
  now = 33'000'000'000;
  core->try_send_next_goal();
  ASSERT_EQ(sends, 2);
  core->goal_response_callback(core->current_dispatch_id, std::make_shared<FakeGoalHandle>(), true, "");
  core->get_result_callback(core->current_dispatch_id, action_msgs::msg::GoalStatus::STATUS_CANCELED, 0, "", "");
  now = 35'000'000'000;
  core->try_send_next_goal();
  now = 66'000'000'000;
  core->try_send_next_goal();
  EXPECT_EQ(sends, 2);  // Second cooldown lasts 60 s, rather than expiring after 30 s.
  EXPECT_TRUE(std::any_of(logs.begin(), logs.end(), [](const auto & s) {
    return s.find("cooldown_s=60") != std::string::npos;
  }));
  now = 96'000'000'000;
  core->try_send_next_goal();
  EXPECT_EQ(sends, 3);
}

TEST(RepeatGuardTests, LocalInformationGainDoesNotSuppressSuccessfulGoal)
{
  int64_t now = 0;
  int sends = 0;
  std::vector<std::string> logs;
  auto core = make_suppression_core(&now, &sends, &logs);
  core->params.repeat_guard_enabled = true;
  core->params.frontier_suppression_enabled = false;
  core->params.post_goal_min_settle = 1.0;
  auto map_msg = build_grid(20, 20, -1);
  core->map = OccupancyGrid2d(map_msg);
  core->try_send_next_goal();
  ASSERT_EQ(sends, 1);
  core->goal_response_callback(core->current_dispatch_id, std::make_shared<FakeGoalHandle>(), true, "");
  core->get_result_callback(core->current_dispatch_id, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED, 0, "", "");
  // Simulate SLAM revealing the local area during post-goal settling.
  map_msg.data[4*20+4] = 0;
  core->map = OccupancyGrid2d(map_msg);
  now = 2'000'000'000;
  core->try_send_next_goal();
  EXPECT_TRUE(std::any_of(logs.begin(), logs.end(), [](const auto & s) {
    return s.find("REPEAT_GUARD_GAIN") != std::string::npos;
  }));
  EXPECT_EQ(sends, 2);
}
