#include <chrono>
#include <array>
#include <iostream>
#include <sstream>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <optional>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "edgeai_mission/action/execute_task.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nlohmann/json.hpp"

#include "mission_process.hpp"
class Coordinator : public rclcpp::Node
{
public:
  Json c;
  fs::path directory, config;
  bool exploration_done = false, cancel = false, seed_enabled = true;
  std::string reason, phase = "STARTING";
  Child observation;
  Child search_task_server, single_task_server, joint_task_server;
  Json supervised_processes;
  Clock::time_point next_health_check{};
  std::optional<Pose> initial1, initial2, start_map;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr completion;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odoms;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service;
  rclcpp::TimerBase::SharedPtr seed_timer;

  Coordinator() : Node("mission_coordinator")
  {
    config = declare_parameter<std::string>("config", "");
    c = readJson(config); directory = c.at("directory").get<std::string>();
    fs::create_directories(directory);
    state = create_publisher<std_msgs::msg::String>("/mission/state", rclcpp::QoS(1).transient_local());
    completion = create_subscription<std_msgs::msg::Empty>("/exploration_complete", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Empty::ConstSharedPtr) {exploration_done = true;});
    stop_service = create_service<std_srvs::srv::Trigger>("/mission/stop",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        cancel = true; response->success = true; response->message = "Cancellation requested; wait for mission result";
      });
    buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener = std::make_unique<tf2_ros::TransformListener>(*buffer, this, false);
    broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    for (const std::string ns : {"", "robot2/"}) {
      odoms.push_back(create_subscription<nav_msgs::msg::Odometry>("/"+ns+"odom", rclcpp::SensorDataQoS(),
        [this, ns](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          auto & p = ns.empty() ? initial1 : initial2;
          if (!p) {p = Pose{m->pose.pose.position.x, m->pose.pose.position.y, tf2::getYaw(m->pose.pose.orientation)};}
        }));
    }
    seed_timer = create_wall_timer(std::chrono::milliseconds(200), [this] {seed();});
    event("STARTING", { {"scheduler", "fixed_rules"}, {"cooperative", c.at("cooperative")} });
  }
  void event(const std::string & name, Json extra = Json::object())
  {
    phase = name; extra["phase"] = name; extra["sim_time"] = now().seconds();
    std_msgs::msg::String m; m.data = extra.dump(); state->publish(m);
    std::ofstream(directory/"mission-events.jsonl", std::ios::app) << m.data << '\n';
    RCLCPP_INFO(get_logger(), "MISSION_%s: %s", name.c_str(), m.data.c_str());
  }
  void seed()
  {
    if (!c.at("cooperative").get<bool>() || !seed_enabled || !initial1 || !initial2 || now().nanoseconds() == 0) {return;}
    if (!start_map) {
      try {
        const auto t = buffer->lookupTransform("map", "odom", tf2::TimePointZero).transform;
        start_map = compose({t.translation.x, t.translation.y, tf2::getYaw(t.rotation)}, *initial1);
        writeJson(directory/"mission-reference.json", {{"mapping_start_pose", *start_map}, {"initial_odom2", *initial2}});
        event("INITIAL_REFERENCE", {{"robot1_start_map", *start_map}});
      } catch (const tf2::TransformException &) {return;}
    }
    const Pose prior = c.at("initial_relative_pose").get<Pose>();
    const auto p = compose(compose(*start_map, prior), inverse(*initial2));
    geometry_msgs::msg::TransformStamped t; t.header.frame_id = "map"; t.child_frame_id = "robot2/odom";
    t.header.stamp = now(); t.transform.translation.x = p[0]; t.transform.translation.y = p[1];
    t.transform.rotation.z = std::sin(p[2]/2); t.transform.rotation.w = std::cos(p[2]/2);
    broadcaster->sendTransform(t);
  }
  bool healthy()
  {
    if (Clock::now() < next_health_check) {return true;}
    next_health_check = Clock::now()+std::chrono::milliseconds(250);
    if(c.at("rescue_enabled").get<bool>() && search_task_server.done()) {reason="Target search task server exited";return false;}
    if(c.at("rescue_enabled").get<bool>() && single_task_server.done()) {reason="S1 task server exited";return false;}
    if(c.at("rescue_enabled").get<bool>() && c.at("cooperative").get<bool>() && joint_task_server.done()) {reason="C1 task server exited";return false;}
    auto path = directory/"mission-processes.json";
    if (supervised_processes.is_null()) {
      if (!fs::exists(path)) {return true;}
      supervised_processes = readJson(path);  // Atomic, immutable launch identities; not read on every tick.
    }
    for (const auto & item : supervised_processes.items()) {
      if (item.key() == "explorer" && exploration_done) {continue;}
      auto proc = fs::path("/proc")/std::to_string(item.value().at("pid").get<int>())/"stat";
      std::ifstream f(proc); std::string line;
      if (!std::getline(f, line)) {reason = "Required process exited: " + item.key(); return false;}
      // Fields after the final ')' start at stat field 3 (comm may contain spaces).
      std::istringstream fields(line.substr(line.rfind(')')+2)); std::string value;
      for (int i = 3; i <= 22; ++i) {
        fields >> value;
        if (i == 3 && value == "Z") {reason = "Required process exited: " + item.key(); return false;}
      }
      if (value != item.value().at("start_ticks").get<std::string>()) {
        reason = "Required process identity changed: " + item.key(); return false;
      }
    }
    return true;
  }
  void stopOwnedProcess(const fs::path & path,const std::string & event_name)
  {
    if (!fs::exists(path)) {return;}
    const auto record = readJson(path); const auto pid = record.at("pid").get<int>();
    std::ifstream f(fs::path("/proc")/std::to_string(pid)/"stat"); std::string line;
    if (!std::getline(f, line) || getpgid(pid) != pid) {return;}
    std::istringstream fields(line.substr(line.rfind(')')+2)); std::string tick;
    for (int i = 3; i <= 22; ++i) {fields >> tick;}
    if (tick != record.at("start_ticks").get<std::string>()) {return;}
    event(event_name, {{"pid", pid}});
    kill(-pid, SIGTERM);
    const auto end = Clock::now()+std::chrono::seconds(5);
    while (kill(pid, 0) == 0 && Clock::now() < end) {std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    if (kill(pid, 0) == 0) {kill(-pid, SIGKILL);}
  }
};

class AwaitExploration : public BT::StatefulActionNode
{
  Coordinator & n;
  Clock::time_point launched, started;
  bool ready = false, finishing = false;
public:
  AwaitExploration(const std::string & name, const BT::NodeConfig & config, Coordinator & node)
  : StatefulActionNode(name, config), n(node) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override
  {launched = Clock::now(); n.event("WAITING_EXPLORATION"); return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override
  {
    if (!ready) {
      if (!fs::exists(n.directory/"ready")) {
        if (Clock::now()-launched > std::chrono::seconds(90)) {n.reason = "Startup readiness timeout"; return BT::NodeStatus::FAILURE;}
        return BT::NodeStatus::RUNNING;
      }
      ready = true; started = Clock::now(); n.event("EXPLORING");
      if (n.c.at("observation").get<bool>()) {
        fs::path root = n.c.at("project_root").get<std::string>();
        n.observation.launch({"python3", "-u", (root/"src/edgeai_perception/scripts/observation_scan.py").string(),
          "--ros-args", "--params-file", (n.directory/"observation-scan-runtime.yaml").string()}, n.directory/"observation.log");
      }
    }
    if (n.c.at("backend") == "lite" && !n.exploration_done) {
      std::ifstream f(n.directory/"explore.log");
      std::string text((std::istreambuf_iterator<char>(f)), {});
      n.exploration_done = text.find("No frontiers found, stopping") != std::string::npos;
      if (text.find("All frontiers traversed/tried out, stopping") != std::string::npos) {
        n.reason = "Exploration exhausted attempted/blacklisted frontiers"; return BT::NodeStatus::FAILURE;
      }
    }
    if (n.exploration_done) {
      if (!finishing) {finishing = true; n.observation.requestStop(); n.event("EXPLORATION_COMPLETE");}
      return n.observation.stopped() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }
    if (std::chrono::duration<double>(Clock::now()-started).count() > n.c.at("exploration_timeout_s").get<double>()) {
      n.reason = "Exploration time limit; not exploration success"; return BT::NodeStatus::FAILURE;
    }
    if (n.c.at("observation").get<bool>() && n.observation.done()) {
      n.reason = "Observation scheduler exited unexpectedly"; return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {n.observation.stop();}
};

class MissionStage : public BT::StatefulActionNode
{
  Coordinator & n;
  Child child;
  std::string step;
public:
  MissionStage(const std::string & name, const BT::NodeConfig & config, Coordinator & node)
  : StatefulActionNode(name, config), n(node) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("step")};}
  BT::NodeStatus onStart() override
  {
    step = getInput<std::string>("step").value();
    if (step != "quiesce" && step != "save_map") {throw std::runtime_error("Unregistered preparation step");}
    // Never consume a result left by a previous attempt.
    fs::remove(n.directory/("stage-"+step+"-result.json"));
    n.event("STAGE_STARTED", {{"step", step}});
    const auto log = "mission-stage.log";
    child.launch({"python3", "-u", n.c.at("worker_script").get<std::string>(), "--step", step, "--config", n.config.string()}, n.directory/log);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if (!child.done()) {return BT::NodeStatus::RUNNING;}
    auto path = n.directory/("stage-"+step+"-result.json");
    Json result = fs::exists(path) ? readJson(path) : Json{{"success", false}, {"reason", "Stage exited without confirmed result"}};
    const bool okay = child.success() && result.value("success", false);
    n.event("STAGE_FINISHED", {{"step", step}, {"success", okay}, {"result", result}});
    if (!okay) {n.reason = step+": "+result.value("reason", "Worker failed");}
    return okay ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  void onHalted() override {child.stop();}
};

class ExecuteTask : public BT::StatefulActionNode
{
  using Action=edgeai_mission::action::ExecuteTask;
  using Handle=rclcpp_action::ClientGoalHandle<Action>;
  Coordinator & n;
  rclcpp_action::Client<Action>::SharedPtr client;
  std::shared_future<Handle::SharedPtr> sent;
  std::shared_future<Handle::WrappedResult> result;
  Handle::SharedPtr handle;
  Clock::time_point started;
  std::string task;
public:
  ExecuteTask(const std::string & name,const BT::NodeConfig & config,Coordinator & node):StatefulActionNode(name,config),n(node) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("task")};}
  BT::NodeStatus onStart() override
  {
    task=getInput<std::string>("task").value();started=Clock::now();sent={};result={};handle.reset();
    if(task!="find_targets" && task!="s1" && task!="c1") {throw std::runtime_error("Unregistered task");}
    if(task=="c1") {
      if(!n.start_map) {n.reason="Missing mapping-start reference";return BT::NodeStatus::FAILURE;}
      n.seed_enabled=false;
    }
    client=rclcpp_action::create_client<Action>(&n,"/mission/tasks/"+task+"/execute");
    n.event("TASK_REQUESTED",{{"task",task}});return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if(!sent.valid()) {
      if(!client->action_server_is_ready()) {
        if(Clock::now()-started>std::chrono::seconds(15)) {n.reason="Task server unavailable: "+task;return BT::NodeStatus::FAILURE;}
        return BT::NodeStatus::RUNNING;
      }
      Action::Goal goal;goal.task_id=n.directory.filename().string()+":"+task;
      auto options=rclcpp_action::Client<Action>::SendGoalOptions();
      options.feedback_callback=[this](auto,std::shared_ptr<const Action::Feedback> feedback) {
        n.event("TASK_FEEDBACK",{{"task",task},{"unit",feedback->phase},
          {"details",feedback->details_json.empty()?Json::object():Json::parse(feedback->details_json)}});
      };
      sent=client->async_send_goal(goal,options);return BT::NodeStatus::RUNNING;
    }
    if(!handle) {
      if(sent.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {return BT::NodeStatus::RUNNING;}
      handle=sent.get();if(!handle) {n.reason="Task rejected: "+task;return BT::NodeStatus::FAILURE;}
      result=client->async_get_result(handle);
    }
    if(result.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {return BT::NodeStatus::RUNNING;}
    const auto terminal=result.get();bool okay=terminal.code==rclcpp_action::ResultCode::SUCCEEDED && terminal.result->success;
    n.event("TASK_FINISHED",{{"task",task},{"success",okay},{"reason",terminal.result->reason}});
    if(!okay) {n.reason=task+": "+terminal.result->reason;}
    return okay?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
  }
  void onHalted() override
  {
    const auto end=Clock::now()+std::chrono::seconds(20);
    if(!handle && sent.valid()) {
      while(sent.wait_for(std::chrono::seconds(0))!=std::future_status::ready && Clock::now()<end) {rclcpp::spin_some(n.shared_from_this());std::this_thread::sleep_for(std::chrono::milliseconds(20));}
      if(sent.wait_for(std::chrono::seconds(0))==std::future_status::ready) {handle=sent.get();if(handle) {result=client->async_get_result(handle);}}
    }
    if(handle && result.valid() && result.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {
      client->async_cancel_goal(handle);
      while(result.wait_for(std::chrono::seconds(0))!=std::future_status::ready && Clock::now()<end) {rclcpp::spin_some(n.shared_from_this());std::this_thread::sleep_for(std::chrono::milliseconds(20));}
      if(result.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {n.event("TASK_STOP_UNCONFIRMED",{{"task",task}});}
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, [](int) {interrupted = 1;});
  std::signal(SIGTERM, [](int) {interrupted = 1;});
  std::shared_ptr<Coordinator> n;
  std::optional<BT::Tree> tree;
  bool success = false;
  try {
    n = std::make_shared<Coordinator>();
    if(n->c.at("rescue_enabled").get<bool>()) {
      auto binary=n->c.value("task_server_executable",(fs::read_symlink("/proc/self/exe").parent_path()/"task_executor").string());
      n->search_task_server.launch({binary,"--ros-args","-p","config:="+n->config.string(),"-p","task_name:=find_targets","-r","__node:=find_targets_task_executor"},n->directory/"find-targets-task.log");
      n->single_task_server.launch({binary,"--ros-args","-p","config:="+n->config.string(),"-p","task_name:=s1","-r","__node:=s1_task_executor"},n->directory/"s1-task.log");
      if(n->c.at("cooperative").get<bool>()) {n->joint_task_server.launch({binary,"--ros-args","-p","config:="+n->config.string(),"-p","task_name:=c1","-r","__node:=c1_task_executor"},n->directory/"c1-task.log");}
    }
    BT::BehaviorTreeFactory f;
    f.registerBuilder<AwaitExploration>("AwaitExploration", [&n](auto & name, auto & config) {return std::make_unique<AwaitExploration>(name, config, *n);});
    f.registerBuilder<MissionStage>("MissionStage", [&n](auto & name, auto & config) {return std::make_unique<MissionStage>(name, config, *n);});
    f.registerBuilder<ExecuteTask>("ExecuteTask",[&n](const auto & name,const auto & config){return std::make_unique<ExecuteTask>(name,config,*n);});
    f.registerSimpleCondition("RescueEnabled", [&n](BT::TreeNode &) {return n->c.at("rescue_enabled").get<bool>() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    f.registerSimpleCondition("CooperativeEnabled", [&n](BT::TreeNode &) {return n->c.at("cooperative").get<bool>() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    tree.emplace(f.createTreeFromFile(n->c.at("tree").get<std::string>()));
    auto begin = Clock::now();
    while (rclcpp::ok()) {
      rclcpp::spin_some(n);
      if (interrupted || n->cancel) {n->reason = "Mission canceled"; break;}
      if (!n->healthy()) {break;}
      if (std::chrono::duration<double>(Clock::now()-begin).count() > n->c.at("wall_timeout_s").get<double>()) {
        n->reason = "Mission wall budget exhausted"; break;
      }
      auto status = tree->tickExactlyOnce();
      if (status == BT::NodeStatus::SUCCESS) {success = true; break;}
      if (status == BT::NodeStatus::FAILURE) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } catch (const std::exception & error) {
    if (n) {n->reason = error.what();} else {std::cerr << error.what() << std::endl;}
  }
  if (tree) {tree->haltTree();}
  if (n) {
    n->observation.stop();
    n->search_task_server.stop();n->single_task_server.stop();n->joint_task_server.stop();
    for(const std::string task:{"find_targets","s1","c1"}) {n->stopOwnedProcess(n->directory/("task-"+task+"-worker-process.json"),"CLEANUP_ORPHAN_TASK_BACKEND");}
    n->stopOwnedProcess(n->directory/"mission-owned-navigation.json","CLEANUP_OWNED_NAVIGATION");
    // Cancellation of the mission also stops the AUTO explorer through authority.
    for (const std::string ns : {"", "robot2/"}) {
      if (!ns.empty() && !n->c.at("cooperative").get<bool>()) {continue;}
      auto client = n->create_client<std_srvs::srv::Trigger>("/"+ns+"navigation_authority/stop");
      auto until = Clock::now()+std::chrono::seconds(2);
      while (rclcpp::ok() && !client->service_is_ready() && Clock::now() < until) {
        rclcpp::spin_some(n); std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (client->service_is_ready()) {
        auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        while (rclcpp::ok() && future.wait_for(std::chrono::seconds(0)) != std::future_status::ready && Clock::now() < until) {
          rclcpp::spin_some(n); std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      }
    }
    Json result{{"success", success}, {"reason", success ? "Completed" : n->reason}, {"scheduler", "fixed_rules"}, {"sim_time", n->now().seconds()}};
    n->event("RESULT", result); writeJson(n->directory/"mission-result.json", result);
  }
  tree.reset(); n.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return success ? 0 : 1;
}
