#include <gtest/gtest.h>
#include "shim_heading.hpp"
#include "lattice_heading_order.hpp"

class ShimFrames : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    buffer = std::make_shared<tf2_ros::Buffer>(clock);
    robot.header.frame_id = "odom";
    robot.header.stamp = rclcpp::Time(110801000000LL);
    robot.pose.orientation.w = 1.0;
    path.header.frame_id = "map"; path.header.stamp = robot.header.stamp;
    path.pose.position.x = -0.3;  // A rearward point must not force a half turn.
    path.pose.orientation.z = std::sin(M_PI / 4.0);
    path.pose.orientation.w = std::cos(M_PI / 4.0);
  }
  void transform(int64_t stamp, bool is_static = false)
  {
    geometry_msgs::msg::TransformStamped frame;
    frame.header.frame_id = "odom"; frame.child_frame_id = "map";
    frame.header.stamp = rclcpp::Time(stamp);
    frame.transform.rotation.z = -std::sin(M_PI / 4.0);
    frame.transform.rotation.w = std::cos(M_PI / 4.0);
    ASSERT_TRUE(buffer->setTransform(frame, "test", is_static));
  }
  rclcpp::Clock::SharedPtr clock;
  std::shared_ptr<tf2_ros::Buffer> buffer;
  geometry_msgs::msg::PoseStamped robot, path;
};

TEST_F(ShimFrames, OneMillisecondTFGapUsesFreshTransformAndCorrectFrame)
{
  transform(110800000000LL);
  EXPECT_NEAR(edgeai_navigation::shimHeadingError(*buffer, path, robot, .3), 0.0, 1e-8);
}

TEST_F(ShimFrames, MissingTransformIsRetryableAndLaterRecovers)
{
  EXPECT_THROW(edgeai_navigation::shimHeadingError(*buffer, path, robot, .3), nav2_core::NoValidControl);
  transform(110800000000LL);
  EXPECT_NEAR(edgeai_navigation::shimHeadingError(*buffer, path, robot, .3), 0.0, 1e-8);
}

TEST_F(ShimFrames, OldTransformStillStopsControl)
{
  transform(110000000000LL);
  EXPECT_THROW(edgeai_navigation::shimHeadingError(*buffer, path, robot, .3), nav2_core::NoValidControl);
}

TEST_F(ShimFrames, StaticFrameDoesNotExpire)
{
  transform(0, true);
  EXPECT_NEAR(edgeai_navigation::shimHeadingError(*buffer, path, robot, .3), 0.0, 1e-8);
}

TEST(GoalHeading, NarrowSlotPrefersBodyAlignedWithFreeSpace)
{
  nav2_costmap_2d::Costmap2D map(100, 100, .05, 0., 0., 0);
  for (unsigned int y = 0; y < 100; ++y) {
    map.setCost(34, y, 254); map.setCost(46, y, 254);
  }
  nav2_costmap_2d::Footprint footprint;
  for (const auto & xy : std::vector<std::pair<double, double>>{
      {.1325, .185}, {-.3325, .185}, {-.3325, -.185}, {.1325, -.185}})
  {
    geometry_msgs::msg::Point p; p.x = xy.first; p.y = xy.second; footprint.push_back(p);
  }
  EXPECT_FALSE(edgeai_navigation::exactGoalFootprintClear(map, footprint, 2., 2., 0., true));
  EXPECT_FALSE(edgeai_navigation::exactGoalFootprintClear(map, footprint, 2., 2., M_PI, true));
  EXPECT_TRUE(edgeai_navigation::exactGoalFootprintClear(map, footprint, 2., 2., M_PI / 2., true));
  std::vector<double> candidates{0., M_PI, M_PI / 2., 3. * M_PI / 2.};
  std::stable_partition(candidates.begin(), candidates.end(), [&](double yaw) {
      return edgeai_navigation::exactGoalFootprintClear(map, footprint, 2., 2., yaw, true);
    });
  EXPECT_NEAR(candidates.front(), M_PI / 2., 1e-8);
  EXPECT_EQ(candidates.size(), 4u);  // Retain approximate-goal alternatives.
}
