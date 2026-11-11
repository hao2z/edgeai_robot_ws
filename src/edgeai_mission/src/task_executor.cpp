#include <iostream>
#include "mission_process.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "edgeai_mission/action/execute_task.hpp"

using Task = edgeai_mission::action::ExecuteTask;
using TaskHandle = rclcpp_action::ServerGoalHandle<Task>;

class TaskExecutor : public rclcpp::Node
{
public:
  Json c;
  fs::path config, directory;
  std::string task, reason;
  std::shared_ptr<TaskHandle> active;
  rclcpp_action::Server<Task>::SharedPtr server;
  Child worker;
  std::string feedback_phase;
  std::string logged_operation;
  Clock::time_point feedback_at{};
  void feedback(const std::string & phase, const Json & details=Json::object())
  {
    if(phase==feedback_phase && details.empty()) {return;}
    if(phase==feedback_phase && Clock::now()-feedback_at<std::chrono::seconds(1) &&
       !details.value("complete",false) && details.value("success",true)) {return;}
    feedback_phase=phase;feedback_at=Clock::now();
    auto message=std::make_shared<Task::Feedback>();message->phase=phase;message->details_json=details.dump();active->publish_feedback(message);
  }
  TaskExecutor() : Node("task_executor")
  {
    config=declare_parameter<std::string>("config", "");
    task=declare_parameter<std::string>("task_name", "s1");
    if(task!="find_targets" && task!="s1" && task!="c1") {throw std::runtime_error("Unregistered task");}
    c=readJson(config);directory=c.at("directory").get<std::string>();
    server=rclcpp_action::create_server<Task>(this,"/mission/tasks/"+task+"/execute",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Task::Goal> goal) {
        return active || goal->task_id.empty() ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<TaskHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<TaskHandle> handle) {active=handle;});
  }
};

class TaskUnit : public BT::StatefulActionNode
{
  TaskExecutor & n;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
  std::optional<rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture> pending;
  Clock::time_point started;
  std::string op;
public:
  TaskUnit(const std::string & name,const BT::NodeConfig & config,TaskExecutor & node)
  : StatefulActionNode(name,config),n(node) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("operation")};}
  BT::NodeStatus onStart() override
  {
    op=getInput<std::string>("operation").value();started=Clock::now();pending.reset();
    client=n.create_client<std_srvs::srv::Trigger>("/mission/tasks/"+n.task+"/units/"+op);
    n.feedback(op);
    if(op!=n.logged_operation) {
      RCLCPP_INFO(n.get_logger(),"TASK_UNIT_STARTED: %s/%s",n.task.c_str(),op.c_str());n.logged_operation=op;
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if(!pending) {
      if(!client->service_is_ready()) {
        if(Clock::now()-started>std::chrono::seconds(15)) {
          n.reason="Unit unavailable: "+op;return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
      }
      pending=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()).future;
      return BT::NodeStatus::RUNNING;
    }
    if(pending->wait_for(std::chrono::seconds(0))!=std::future_status::ready) {return BT::NodeStatus::RUNNING;}
    auto response=pending->get();pending.reset();auto data=Json::parse(response->message);
    n.feedback(op,data);
    for(const std::string key:{"complete","needs_correction"}) {
      if(data.contains(key)) {config().blackboard->set(key,data.at(key).get<bool>());}
    }
    config().blackboard->set("retry_kind",data.value("category",std::string("failure")));
    if(!response->success) {n.reason=data.value("reason",op+" failed");}
    return response->success?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
  }
  void onHalted() override
  {
    // Services do not support cancel. The owning task halts and reaps the
    // entire backend before releasing its Action slot; no next unit is sent.
    if(client) {client->prune_pending_requests();}pending.reset();
  }
};

class UntilDone : public BT::DecoratorNode
{
public:
  UntilDone(const std::string & name,const BT::NodeConfig & config):DecoratorNode(name,config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    const auto status=child_node_->executeTick();
    if(status==BT::NodeStatus::RUNNING) {return status;}
    resetChild();
    if(status==BT::NodeStatus::FAILURE) {return status;}
    return config().blackboard->get<bool>("complete")?BT::NodeStatus::SUCCESS:BT::NodeStatus::RUNNING;
  }
};

class RetryTask : public BT::DecoratorNode
{
  unsigned failures=0;
public:
  RetryTask(const std::string & name,const BT::NodeConfig & config):DecoratorNode(name,config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<unsigned>("attempts",3u, "Maximum attempts")};}
  BT::NodeStatus tick() override
  {
    if(status()==BT::NodeStatus::IDLE) {failures=0;}
    setStatus(BT::NodeStatus::RUNNING);
    auto result=child_node_->executeTick();
    if(result!=BT::NodeStatus::FAILURE) {return result;}
    resetChild();
    const auto kind=config().blackboard->get<std::string>("retry_kind");
    if(kind=="fatal") {return BT::NodeStatus::FAILURE;}
    if(kind!="target_updated") {++failures;}
    return failures>=getInput<unsigned>("attempts").value()?BT::NodeStatus::FAILURE:BT::NodeStatus::RUNNING;
  }
};

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  std::signal(SIGINT,[](int){interrupted=1;});std::signal(SIGTERM,[](int){interrupted=1;});
  auto n=std::make_shared<TaskExecutor>();
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<TaskUnit>("TaskUnit",[&n](const auto & name,const auto & config){return std::make_unique<TaskUnit>(name,config,*n);});
  factory.registerNodeType<UntilDone>("UntilDone");factory.registerNodeType<RetryTask>("RetryTask");
  factory.registerSimpleCondition("TaskDone",[](BT::TreeNode & node){return std::as_const(node).config().blackboard->get<bool>("complete")?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;});
  factory.registerSimpleCondition("CorrectionNeeded",[](BT::TreeNode & node){return std::as_const(node).config().blackboard->get<bool>("needs_correction")?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;});
  factory.registerSimpleCondition("TaskErrorIs",[](BT::TreeNode & node){
    return node.getInput<std::string>("category").value()==std::as_const(node).config().blackboard->get<std::string>("retry_kind")?
      BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
  },{BT::InputPort<std::string>("category")});
  std::optional<BT::Tree> tree;
  Clock::time_point begin;
  bool finished=false,okay=false;
  while(rclcpp::ok() && !interrupted) {
    rclcpp::spin_some(n);
    if(n->active) {
      try {
        if(!tree) {
          n->reason.clear();finished=false;okay=false;begin=Clock::now();
          fs::remove(n->directory/("task-"+n->task+"-worker-result.json"));
          n->worker.launch({"python3","-u",n->c.at("unit_worker_script").get<std::string>(),
              "--task",n->task,"--config",n->config.string()},n->directory/(n->task=="find_targets"?"target-search.log":n->task=="s1"?"rescue.log":"cooperative.log"));
          writeJson(n->directory/("task-"+n->task+"-worker-process.json"),{{"pid",n->worker.pid},{"start_ticks",processStart(n->worker.pid)}});
          tree.emplace(factory.createTreeFromFile(n->c.at("task_trees").at(n->task).get<std::string>()));
          tree->rootBlackboard()->set("complete",false);tree->rootBlackboard()->set("needs_correction",false);
          tree->rootBlackboard()->set("retry_kind",std::string("failure"));
          const auto attempts=n->c.value("task_attempts",Json::object()).value(n->task,n->task=="s1"?3u:4u);
          if(attempts<1 || attempts>20) {throw std::runtime_error("Task attempts must be in 1..20");}
          tree->rootBlackboard()->set("task_attempts",attempts);
          const auto nav_attempts=n->c.value("task_navigation_attempts",Json::object()).value(n->task,3u);
          if(nav_attempts<1 || nav_attempts>20) {throw std::runtime_error("Navigation attempts must be in 1..20");}
          tree->rootBlackboard()->set("navigation_attempts",nav_attempts);
        }
        if(n->active->is_canceling()) {n->reason="Task canceled";finished=true;}
        const auto budgets=n->c.value("task_wall_timeout_s",Json::object());
        const auto budget=budgets.value(n->task,n->c.at("wall_timeout_s").get<double>());
        if(std::chrono::duration<double>(Clock::now()-begin).count()>budget) {n->reason="Task wall budget exhausted";finished=true;okay=false;}
        if(!finished && n->worker.done()) {n->reason="Unit backend exited unexpectedly";finished=true;}
        if(!finished) {
          auto status=tree->tickExactlyOnce();finished=status!=BT::NodeStatus::RUNNING;okay=status==BT::NodeStatus::SUCCESS;
        }
      } catch(const std::exception & e) {n->reason=e.what();finished=true;okay=false;}
      if(finished) {
        if(tree) {tree->haltTree();}n->worker.stop();
        fs::remove(n->directory/("task-"+n->task+"-worker-process.json"));
        const auto path=n->directory/("task-"+n->task+"-worker-result.json");
        auto data=fs::exists(path)?readJson(path):Json{{"success",false},{"reason","No worker cleanup result"}};
        okay=okay && n->worker.success() && data.value("success",false);
        auto result=std::make_shared<Task::Result>();result->success=okay;
        result->reason=okay?"Completed":n->reason.empty()?data.value("reason","Task failed"):n->reason;
        result->result_json=data.dump();
        if(n->active->is_canceling()) {n->active->canceled(result);}else if(okay) {n->active->succeed(result);}else {n->active->abort(result);}
        n->active.reset();tree.reset();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if(tree) {tree->haltTree();}n->worker.stop();
  fs::remove(n->directory/("task-"+n->task+"-worker-process.json"));
  if(n->active) {auto result=std::make_shared<Task::Result>();result->reason="Task server stopped";n->active->abort(result);}
  n.reset();if(rclcpp::ok()) {rclcpp::shutdown();}return 0;
}
