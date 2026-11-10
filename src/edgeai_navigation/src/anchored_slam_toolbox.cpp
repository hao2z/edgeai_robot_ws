// Stock async SLAM with a read-only, timestamped keyframe snapshot for landmarks.
#include <chrono>
#include <cmath>
#include <string>
#include <nlohmann/json.hpp>
#include <slam_toolbox/slam_toolbox_async.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class AnchoredSlam : public slam_toolbox::AsynchronousSlamToolbox
{
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graph_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finalize_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string instance_=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  std::string origin_;
  unsigned epoch_=0,revision_=0;
  bool finalized_=false;

  nlohmann::json snapshot(bool finalized)
  {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    nlohmann::json nodes=nlohmann::json::array();
    const auto * graph=smapper_->getMapper()->GetGraph();
    // Karto creates its graph only after the first accepted laser scan.
    // Gazebo may deliver that scan seconds after lifecycle activation.
    if(graph) {for(const auto & sensor:graph->GetVertices()) {
      for(const auto & item:sensor.second) {
        const auto * scan=item.second->GetObject();
        const auto & odom=scan->GetOdometricPose();const auto & pose=scan->GetCorrectedPose();
        nodes.push_back({{"id",scan->GetUniqueId()},
          {"stamp_ns",static_cast<int64_t>(std::llround(scan->GetTime()*1e9))},
          {"odom",{odom.GetX(),odom.GetY(),odom.GetHeading()}},
          {"map",{pose.GetX(),pose.GetY(),pose.GetHeading()}}});
      }
    }}
    const auto origin=nodes.empty()?std::string("empty"):
      std::to_string(nodes[0]["id"].get<int>())+":"+std::to_string(nodes[0]["stamp_ns"].get<int64_t>());
    if(origin!=origin_) {
      if(!origin_.empty() && origin_!="empty") {++epoch_;}
      origin_=origin;
    }
    return {{"frame_id",map_frame_},{"epoch",instance_+":"+std::to_string(epoch_)},
      {"revision",++revision_},{"stamp_ns",now().nanoseconds()},{"finalized",finalized},{"nodes",nodes}};
  }
  void publish(const nlohmann::json & graph)
  {std_msgs::msg::String msg;msg.data=graph.dump();graph_pub_->publish(msg);}

public:
  explicit AnchoredSlam(const rclcpp::NodeOptions & options):AsynchronousSlamToolbox(options)
  {
    graph_pub_=create_publisher<std_msgs::msg::String>("slam_toolbox/anchor_graph",rclcpp::QoS(1).transient_local());
    timer_=create_wall_timer(std::chrono::seconds(1),[this] {
      if(get_current_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && smapper_) {
        publish(snapshot(finalized_));
      }
    });
    finalize_=create_service<std_srvs::srv::Trigger>("slam_toolbox/finalize_anchors",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response) {
        if(get_current_state().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE || !smapper_) {
          response->message="SLAM is not active";return;
        }
        // Same executor as laserCallback: pause before the last map raster and
        // graph snapshot. No further scan can change the handover geometry.
        state_.set(toolbox_types::NEW_MEASUREMENTS,true);
        set_parameter(rclcpp::Parameter("paused_new_measurements",true));
        if(!updateMap()) {response->message="Final map unavailable";return;}
        finalized_=true;auto graph=snapshot(true);publish(graph);
        response->success=!graph["nodes"].empty();response->message=graph.dump();
      });
  }
};

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);auto node=std::make_shared<AnchoredSlam>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());rclcpp::shutdown();return 0;
}
