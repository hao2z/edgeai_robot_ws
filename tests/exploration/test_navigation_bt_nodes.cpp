#include <chrono>
#include <thread>
#include <atomic>
#include <gtest/gtest.h>
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"

class NavigationBT : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node=rclcpp::Node::make_shared("navigation_bt_test");
    tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    bb=BT::Blackboard::create();bb->set("node",node);bb->set("tf_buffer",tf);
    geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="odom";goal.pose.position.x=3.;goal.pose.orientation.w=1.;bb->set("goal",goal);
    factory.registerFromPlugin(TEST_PLUGIN_PATH);
    map_pub=node->create_publisher<nav2_msgs::msg::Costmap>("local_costmap/costmap_raw",1);
    foot_pub=node->create_publisher<geometry_msgs::msg::PolygonStamped>("local_costmap/published_footprint",1);
  }
  void TearDown() override
  {if(server_thread.joinable()){server_executor.cancel();server_thread.join();}}
  void pathValidationService()
  {
    bb->set("server_timeout",std::chrono::milliseconds(200));
    bb->set("wait_for_service_timeout",std::chrono::milliseconds(1000));
    bb->set("bt_loop_duration",std::chrono::milliseconds(10));
    server=rclcpp::Node::make_shared("path_validation_test_server");
    validity=server->create_service<nav2_msgs::srv::IsPathValid>("is_path_valid",
      [this](const nav2_msgs::srv::IsPathValid::Request::SharedPtr,nav2_msgs::srv::IsPathValid::Response::SharedPtr response){response->is_valid=path_valid.load();});
    server_executor.add_node(server);server_thread=std::thread([this]{server_executor.spin();});
    factory.registerFromPlugin(std::string(TEST_NAV2_BT_DIR)+"/libnav2_globally_updated_goal_condition_bt_node.so");
    factory.registerFromPlugin(std::string(TEST_NAV2_BT_DIR)+"/libnav2_path_expiring_timer_condition.so");
    factory.registerFromPlugin(std::string(TEST_NAV2_BT_DIR)+"/libnav2_is_path_valid_condition_bt_node.so");
    factory.registerSimpleAction("ComputeTestPath",[this](BT::TreeNode &){
      ++plan_calls;nav_msgs::msg::Path path;path.header.frame_id="odom";path.header.stamp=node->now();
      geometry_msgs::msg::PoseStamped start;start.header=path.header;start.pose.position.x=2.;start.pose.position.y=2.;start.pose.orientation.w=1.;
      path.poses={start,bb->get<geometry_msgs::msg::PoseStamped>("goal")};bb->set("path",path);path_valid=true;
      return BT::NodeStatus::SUCCESS;
    });
    bb->set("path",nav_msgs::msg::Path{});
  }
  BT::Tree stableBranch(double expiry=10.)
  {
    return factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><Fallback><ReactiveSequence>"
      "<Inverter><PathExpiringTimer seconds=\""+std::to_string(expiry)+"\" path=\"{path}\"/></Inverter>"
      "<Inverter><GlobalUpdatedGoal/></Inverter><PathNearRobot path=\"{path}\" max_distance=\"0.25\"/>"
      "<IsPathValid path=\"{path}\" server_timeout=\"200\"/></ReactiveSequence><ComputeTestPath/></Fallback></BehaviorTree></root>",bb);
  }
  void publish(bool stale=false,bool occupied=false)
  {
    const auto now=node->now();const builtin_interfaces::msg::Time stamp=now-rclcpp::Duration::from_seconds(stale?2.:0.);
    geometry_msgs::msg::TransformStamped t;t.header.frame_id="odom";t.child_frame_id="base_footprint";
    t.header.stamp=stamp;t.transform.translation.x=robot_x;t.transform.translation.y=2.;t.transform.rotation.w=1.;tf->setTransform(t,"test",false);
    nav2_msgs::msg::Costmap map;map.header.frame_id="odom";map.header.stamp=stamp;
    map.metadata.resolution=.05;map.metadata.size_x=map.metadata.size_y=100;map.metadata.origin.orientation.w=1.;map.data.assign(10000,0);
    // A rear wall blocks both backup candidates but leaves a forward exit.
    for(unsigned y=0;y<100;++y){map.data[y*100+32]=254;}
    if(occupied){map.data[40*100+40]=254;}
    map_pub->publish(map);
    geometry_msgs::msg::PolygonStamped foot;foot.header=map.header;
    for(const auto & xy:std::vector<std::pair<double,double>>{{.1225,.175},{-.3225,.175},{-.3225,-.175},{.1225,-.175}}){
      geometry_msgs::msg::Point32 p;p.x=robot_x+xy.first;p.y=2.+xy.second;foot.polygon.points.push_back(p);
    }
    foot_pub->publish(foot);
  }
  BT::NodeStatus run(BT::Tree & tree,bool stale=false,bool occupied=false)
  {
    BT::NodeStatus state=BT::NodeStatus::RUNNING;
    for(int i=0;i<100 && state==BT::NodeStatus::RUNNING;++i){
      publish(stale,occupied);rclcpp::spin_some(node);state=tree.tickOnce();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return state;
  }
  BT::Tree selector()
  {return factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><SelectRecovery goal="{goal}" kind="{kind}" distance="{distance}" angle="{angle}"/></BehaviorTree></root>)",bb);}
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  BT::Blackboard::Ptr bb;
  BT::BehaviorTreeFactory factory;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr map_pub;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr foot_pub;
  rclcpp::Node::SharedPtr server;
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr validity;
  rclcpp::executors::SingleThreadedExecutor server_executor;
  std::thread server_thread;
  std::atomic<bool> path_valid{true};
  int plan_calls=0;
  double robot_x=2.;
};

TEST_F(NavigationBT, SelectsForwardWhenBackupSweepBlocked)
{
  auto tree=selector();ASSERT_EQ(run(tree),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("kind"),"forward");EXPECT_GT(bb->get<double>("distance"),0.);
}

TEST_F(NavigationBT, RejectedRecoveryTriesAnotherMotionWithoutWaiting)
{
  std::vector<std::pair<std::string,double>> attempts;
  factory.registerSimpleAction("TestRecovery",[&](BT::TreeNode &){
    attempts.push_back({bb->get<std::string>("kind"),bb->get<double>("distance")});
    return attempts.size()==1 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  });
  auto tree=factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><RetryUntilSuccessful num_attempts="8"><Sequence><SelectRecovery goal="{goal}" kind="{kind}" distance="{distance}" angle="{angle}"/><TestRecovery/></Sequence></RetryUntilSuccessful></BehaviorTree></root>)",bb);
  ASSERT_EQ(run(tree),BT::NodeStatus::SUCCESS);
  ASSERT_EQ(attempts.size(),2u);EXPECT_NE(attempts[0],attempts[1]);
}

TEST_F(NavigationBT, AllRejectedRecoveriesTerminateWithBoundedAttempts)
{
  size_t attempts=0;
  factory.registerSimpleAction("RejectRecovery",[&](BT::TreeNode &){++attempts;return BT::NodeStatus::FAILURE;});
  auto tree=factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><RetryUntilSuccessful num_attempts="8"><Sequence><SelectRecovery goal="{goal}" kind="{kind}" distance="{distance}" angle="{angle}"/><RejectRecovery/></Sequence></RetryUntilSuccessful></BehaviorTree></root>)",bb);
  ASSERT_EQ(run(tree),BT::NodeStatus::FAILURE);
  EXPECT_GT(attempts,1u);EXPECT_LE(attempts,8u);
}
TEST_F(NavigationBT, StaleDataCannotAuthorizeMotion)
{
  auto tree=selector();EXPECT_EQ(run(tree,true),BT::NodeStatus::FAILURE);
}
TEST_F(NavigationBT, OccupiedStartCannotAuthorizeMotion)
{
  auto tree=selector();EXPECT_EQ(run(tree,false,true),BT::NodeStatus::FAILURE);
}
TEST_F(NavigationBT, RecoveryHistorySurvivesTenCentimetreTranslation)
{
  auto tree=selector();ASSERT_EQ(run(tree),BT::NodeStatus::SUCCESS);
  const auto first_kind=bb->get<std::string>("kind");const auto first_distance=bb->get<double>("distance");
  tree.haltTree();robot_x=2.1;
  ASSERT_EQ(run(tree),BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(bb->get<std::string>("kind")!=first_kind || bb->get<double>("distance")!=first_distance);
}
TEST_F(NavigationBT, NewRppProgressEpochResetsHistory)
{
  auto pub=node->create_publisher<std_msgs::msg::String>("/navigation/rpp_tracking",1);
  auto tree=selector();
  auto feedback=[&](unsigned epoch){
      std_msgs::msg::String msg;msg.data="{\"stamp\":"+std::to_string(node->now().seconds())+
      ",\"goal_frame\":\"odom\",\"goal_x\":3.0,\"goal_y\":0.0,\"progress_epoch\":"+std::to_string(epoch)+"}";
      pub->publish(msg);
    };
  for(int i=0;i<100 && pub->get_subscription_count()==0;++i){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  ASSERT_GT(pub->get_subscription_count(),0u);feedback(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_EQ(run(tree),BT::NodeStatus::SUCCESS);
  const auto first_kind=bb->get<std::string>("kind");const auto first_distance=bb->get<double>("distance");
  tree.haltTree();feedback(1);std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_EQ(run(tree),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("kind"),first_kind);EXPECT_EQ(bb->get<double>("distance"),first_distance);
}
TEST_F(NavigationBT, PaddingOnlyOccupancySelectsDedicatedSignedExit)
{
  auto tree=factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><SelectRecovery goal="{goal}" actual_footprint="[[0.1125,0.165],[-0.3125,0.165],[-0.3125,-0.165],[0.1125,-0.165]]" kind="{kind}" distance="{distance}" angle="{angle}"/></BehaviorTree></root>)",bb);
  BT::NodeStatus result=BT::NodeStatus::RUNNING;
  for(int i=0;i<100 && result==BT::NodeStatus::RUNNING;++i){
    const auto now=node->now();geometry_msgs::msg::TransformStamped t;
    t.header.frame_id="odom";t.child_frame_id="base_footprint";t.header.stamp=now;
    t.transform.translation.x=2.;t.transform.translation.y=2.;t.transform.rotation.w=1.;tf->setTransform(t,"test",false);
    nav2_msgs::msg::Costmap map;map.header.frame_id="odom";map.header.stamp=now;
    map.metadata.resolution=.005;map.metadata.size_x=map.metadata.size_y=600;map.metadata.origin.orientation.w=1.;map.data.assign(360000,0);
    for(unsigned y=0;y<600;++y){map.data[y*600+423]=254;}
    map_pub->publish(map);
    geometry_msgs::msg::PolygonStamped foot;foot.header=map.header;
    for(const auto & xy:std::vector<std::pair<double,double>>{{.1225,.175},{-.3225,.175},{-.3225,-.175},{.1225,-.175}}){
      geometry_msgs::msg::Point32 p;p.x=2.+xy.first;p.y=2.+xy.second;foot.polygon.points.push_back(p);
    }
    foot_pub->publish(foot);rclcpp::spin_some(node);result=tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(result,BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("kind"),"occupied");EXPECT_LT(bb->get<double>("distance"),0.);
  EXPECT_EQ(bb->get<double>("angle"),0.);
}
TEST_F(NavigationBT, ActualBodyOccupancySelectsDedicatedSignedExit)
{
  auto tree=factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><SelectRecovery goal="{goal}" actual_footprint="[[0.1125,0.165],[-0.3125,0.165],[-0.3125,-0.165],[0.1125,-0.165]]" kind="{kind}" distance="{distance}" angle="{angle}"/></BehaviorTree></root>)",bb);
  BT::NodeStatus result=BT::NodeStatus::RUNNING;
  for(int i=0;i<100 && result==BT::NodeStatus::RUNNING;++i){
    const auto now=node->now();geometry_msgs::msg::TransformStamped t;
    t.header.frame_id="odom";t.child_frame_id="base_footprint";t.header.stamp=now;
    t.transform.translation.x=2.;t.transform.translation.y=2.;t.transform.rotation.w=1.;tf->setTransform(t,"test",false);
    nav2_msgs::msg::Costmap map;map.header.frame_id="odom";map.header.stamp=now;
    map.metadata.resolution=.005;map.metadata.size_x=map.metadata.size_y=600;map.metadata.origin.orientation.w=1.;map.data.assign(360000,0);
    for(unsigned y=0;y<600;++y){map.data[y*600+421]=254;}
    map_pub->publish(map);
    geometry_msgs::msg::PolygonStamped foot;foot.header=map.header;
    for(const auto & xy:std::vector<std::pair<double,double>>{{.1225,.175},{-.3225,.175},{-.3225,-.175},{.1225,-.175}}){
      geometry_msgs::msg::Point32 p;p.x=2.+xy.first;p.y=2.+xy.second;foot.polygon.points.push_back(p);
    }
    foot_pub->publish(foot);rclcpp::spin_some(node);result=tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(result,BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("kind"),"occupied");EXPECT_LT(bb->get<double>("distance"),0.);
  EXPECT_EQ(bb->get<double>("angle"),0.);
}
TEST_F(NavigationBT, InvalidationForcesEmptyTypedPath)
{
  nav_msgs::msg::Path path;path.poses.resize(2);bb->set("path",path);
  auto tree=factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><InvalidatePath path="{path}"/></BehaviorTree></root>)",bb);
  ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);EXPECT_TRUE(bb->get<nav_msgs::msg::Path>("path").poses.empty());
}
TEST_F(NavigationBT, ReusesValidPathButReplansOnObstacleGoalChangeAndInvalidation)
{
  pathValidationService();publish();auto tree=stableBranch();
  ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plan_calls,1);
  for(int i=0;i<10;++i){publish();ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);}
  EXPECT_EQ(plan_calls,1);
  path_valid=false;ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plan_calls,2);
  auto goal=bb->get<geometry_msgs::msg::PoseStamped>("goal");goal.pose.position.y=2.5;bb->set("goal",goal);
  ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plan_calls,3);
  bb->set("path",nav_msgs::msg::Path{});
  ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plan_calls,4);
}
TEST_F(NavigationBT, ExpiringTimerTriggersPeriodicRefresh)
{
  pathValidationService();publish();auto tree=stableBranch(.1);
  ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);
  ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plan_calls,1);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));publish();
  ASSERT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plan_calls,2);
}
TEST_F(NavigationBT, PlannerErrorCodeMatchesNativeUint16Port)
{
  bb->set<uint16_t>("code",205);
  auto tree=factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><PlannerStartOccupied error_code="{code}"/></BehaviorTree></root>)",bb);
  EXPECT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);bb->set<uint16_t>("code",208);
  EXPECT_EQ(tree.tickOnce(),BT::NodeStatus::FAILURE);
}
