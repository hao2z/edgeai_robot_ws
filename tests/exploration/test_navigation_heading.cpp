#include <gtest/gtest.h>
#include "shim_heading.hpp"
#include "lattice_heading_order.hpp"
#include "heading_candidates.hpp"
#include <thread>

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

namespace
{
nav_msgs::msg::Path testPath(double length)
{
  nav_msgs::msg::Path path;geometry_msgs::msg::PoseStamped a,b;
  a.pose.orientation.w=b.pose.orientation.w=1.;b.pose.position.x=length;
  path.poses={a,b};return path;
}
}

TEST(HeadingCandidates, CountSamplesWholeCircleAndKeepsCachedHeadingWithinCap)
{
  std::vector<double> available;
  for (int i=0;i<16;++i) {available.push_back(i*M_PI/8.);}
  auto selected=edgeai_navigation::selectGoalHeadings(available,.03,4,std::nullopt);
  ASSERT_EQ(selected.size(),4u);
  for (size_t i=0;i<4;++i) {EXPECT_NEAR(selected[i],i*M_PI/2.,1e-8);}
  selected=edgeai_navigation::selectGoalHeadings(available,.03,4,M_PI/8.);
  EXPECT_EQ(selected.size(),4u);
  EXPECT_NE(std::find(selected.begin(),selected.end(),M_PI/8.),selected.end());
  EXPECT_EQ(edgeai_navigation::selectGoalHeadings(available,.03,0,std::nullopt).size(),16u);
  EXPECT_EQ(edgeai_navigation::selectGoalHeadings(available,.03,8,std::nullopt).size(),8u);
  selected=edgeai_navigation::selectGoalHeadings({0.,.3,1.5,3.,4.8},6.2,4,std::nullopt);
  auto unique=selected;std::sort(unique.begin(),unique.end());
  EXPECT_EQ(std::unique(unique.begin(),unique.end()),unique.end());
}

TEST(HeadingCandidates, ComparesFeasiblePathsInsteadOfReturningFirst)
{
  auto chosen=edgeai_navigation::chooseHeadingPlan(std::vector<double>{0.,1.,2.},1.,std::nullopt,[]{return false;},
    [](double yaw,auto) {return testPath(yaw==0 ? 5. : yaw==1 ? 1. : 3.);},
    [](const auto & p) {return p.poses.back().pose.position.x;});
  EXPECT_DOUBLE_EQ(chosen.yaw,1.);EXPECT_EQ(chosen.feasible,3u);EXPECT_EQ(chosen.attempted,3u);
}

TEST(HeadingCandidates, InvalidFirstHeadingDoesNotRequireNavigationRecovery)
{
  auto chosen=edgeai_navigation::chooseHeadingPlan(std::vector<double>{0.,1.},1.,std::nullopt,[]{return false;},
    [](double yaw,auto) {if (yaw==0) {throw nav2_core::GoalOccupied("blocked heading");}return testPath(1.);},
    [](const auto &) {return 1.;});
  EXPECT_DOUBLE_EQ(chosen.yaw,1.);EXPECT_EQ(chosen.feasible,1u);
}

TEST(HeadingCandidates, ReusesSuccessfulHeadingButReconsidersAfterFailure)
{
  for (const bool valid:{true,false}) {
    auto chosen=edgeai_navigation::chooseHeadingPlan(std::vector<double>{0.,1.},1.,0.,[]{return false;},
      [valid](double yaw,auto) {if (!valid && yaw==0) {throw nav2_core::GoalOccupied("new obstacle");}return testPath(1.);},
      [](const auto &) {return 1.;});
    EXPECT_EQ(chosen.reused,valid);EXPECT_EQ(chosen.attempted,valid ? 1u : 2u);
    EXPECT_DOUBLE_EQ(chosen.yaw,valid ? 0. : 1.);
  }
}

TEST(HeadingCandidates, ExternalCancelAndInvalidStartAreNeverHiddenByAlternatives)
{
  bool cancel=false;
  EXPECT_THROW(edgeai_navigation::chooseHeadingPlan(std::vector<double>{0.,1.},1.,std::nullopt,[&]{return cancel;},
    [&](double,auto) {cancel=true;return testPath(1.);},[](const auto &) {return 1.;}),nav2_core::PlannerCancelled);
  unsigned int calls=0;
  EXPECT_THROW(edgeai_navigation::chooseHeadingPlan(std::vector<double>{0.,1.},1.,std::nullopt,[]{return false;},
    [&](double,auto) -> nav_msgs::msg::Path {++calls;throw nav2_core::StartOccupied("start collision");},
    [](const auto &) {return 1.;}),nav2_core::StartOccupied);
  EXPECT_EQ(calls,1u);
}

TEST(HeadingCandidates, TimeSlicesShareOneBudgetAndAreNotExternalCancellation)
{
  auto begin=std::chrono::steady_clock::now();
  EXPECT_THROW(edgeai_navigation::chooseHeadingPlan(std::vector<double>{0.,1.,2.},.015,std::nullopt,[]{return false;},
    [](double,auto stop) -> nav_msgs::msg::Path {
      while (!stop()) {std::this_thread::yield();}
      throw nav2_core::PlannerCancelled("candidate time slice");
    },[](const auto &) {return 1.;}),nav2_core::PlannerTimedOut);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(),.5);
}

TEST(HeadingCandidates, PathScoringAccountsForCostReverseAndRotation)
{
  nav2_costmap_2d::Costmap2D map(40,40,.1,-1.,-1.,0);
  const auto cost=[&](const auto & p) {return edgeai_navigation::headingPathCost(p,map,2.,5.,2.,false);};
  auto forward=testPath(1.),backward=forward,spin=testPath(0.);
  std::reverse(backward.poses.begin(),backward.poses.end());
  EXPECT_GT(cost(backward),cost(forward));
  spin.poses.back().pose.orientation.w=0.;spin.poses.back().pose.orientation.z=1.;
  EXPECT_GT(cost(spin),0.);
  const double free=cost(forward);map.setCost(20,10,200);
  EXPECT_GT(cost(forward),free);
  map.setCost(20,10,254);EXPECT_FALSE(std::isfinite(cost(forward)));
}
