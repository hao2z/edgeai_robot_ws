// Central two-sensor Karto graph using the installed slam_toolbox/Ceres backend.
// All ROS callbacks use the default mutually-exclusive group and a single-thread
// executor. TF lookup never holds smapper_mutex_; that mutex protects Karto only.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <slam_toolbox/slam_toolbox_common.hpp>
#include <slam_toolbox/visualization_utils.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace edgeai_mapping
{
using Json = nlohmann::json;
using Scan = sensor_msgs::msg::LaserScan;
using Pose = karto::Pose2;
using CallbackReturn = toolbox_types::CallbackReturn;

class JointSlam : public slam_toolbox::SlamToolbox
{
  struct Pending
  {
    Scan::ConstSharedPtr scan;
    std::chrono::steady_clock::time_point received;
  };
  struct Source
  {
    std::string robot, topic, base, odom, sensor;
    rclcpp::Subscription<Scan>::SharedPtr subscription;
    std::unique_ptr<laser_utils::LaserAssistant> assistant;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
    std::deque<Pending> pending;
    karto::LocalizedRangeScan * last = nullptr;  // Dataset owns accepted scans.
    Pose last_local;
    karto::Matrix3 covariance;
    tf2::Transform map_from_odom = tf2::Transform::getIdentity();
    int64_t received_stamp = 0, accepted_stamp = 0, tf_stamp = 0;
    uint64_t received = 0, accepted = 0, dropped = 0, candidates = 0;
    std::string reason;
  };
  struct NodeInfo {size_t source; Pose local; int64_t stamp;};
  std::array<Source, 2> sources_;
  std::unordered_map<int, NodeInfo> node_info_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr alignment_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr graph_pub_, status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finalize_;
  rclcpp::TimerBase::SharedPtr timer_;
  const std::string instance_ = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  uint64_t epoch_ = 0, revision_ = 0;
  bool active_ = false, aligned_ = false, frozen_ = false, finalized_ = false;
  bool publish_poses_ = true;
  tf2::Transform odom1_from_odom2_ = tf2::Transform::getIdentity();
  size_t queue_size_ = 5;
  double max_age_ = 2., tf_wait_ = .5, future_tolerance_ = .1;
  double graph_period_ = .5, tf_period_ = .05, map_period_ = 5.;
  double min_distance_ = 0., min_heading_ = 0.;
  int64_t map_stamp_ = 0, graph_stamp_ = 0, optimizer_stamp_ = 0;
  uint64_t edges_ = 0, cross_edges_ = 0;
  std::string failure_;
  std::chrono::steady_clock::time_point last_tf_{}, last_graph_{}, last_map_{};

  template<class T> T param(const std::string & name, const T & value)
  {
    if (!has_parameter(name)) {declare_parameter<T>(name, value);}
    return get_parameter(name).get_value<T>();
  }
  static bool finitePose(const Pose & p)
  {return std::isfinite(p.GetX()) && std::isfinite(p.GetY()) && std::isfinite(p.GetHeading());}
  static tf2::Transform transform(const Pose & p)
  {
    tf2::Quaternion q; q.setRPY(0., 0., p.GetHeading());
    return tf2::Transform(q, tf2::Vector3(p.GetX(), p.GetY(), 0.));
  }
  static Pose pose(const tf2::Transform & t)
  {return Pose(t.getOrigin().x(), t.getOrigin().y(), tf2::getYaw(t.getRotation()));}
  static Json xyz(const Pose & p) {return {p.GetX(), p.GetY(), p.GetHeading()};}
  static bool elapsed(std::chrono::steady_clock::time_point t, double period)
  {return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count() >= period;}
  void reject(Source & s, const std::string & reason)
  {++s.dropped; s.reason = reason;}

  // The stock helper returns a laser named "Custom Described Lidar" for every
  // call. Object::GetName is immutable: copy its calibrated properties into a
  // uniquely named sensor BEFORE registration, rather than alias both robots.
  karto::LaserRangeFinder * registerLaser(Source & s, const Scan::ConstSharedPtr & scan)
  {
    auto found = lasers_.find(s.sensor);
    if (found != lasers_.end()) {return found->second.getLaser();}
    auto metadata = s.assistant->toLaserMetadata(*scan);
    std::unique_ptr<karto::LaserRangeFinder> calibrated(metadata.getLaser());
    auto laser = std::unique_ptr<karto::LaserRangeFinder>(
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,
      karto::Name(s.sensor)));
    laser->SetOffsetPose(calibrated->GetOffsetPose());
    laser->SetMinimumRange(calibrated->GetMinimumRange());
    laser->SetMaximumRange(calibrated->GetMaximumRange());
    laser->SetMinimumAngle(calibrated->GetMinimumAngle());
    laser->SetMaximumAngle(calibrated->GetMaximumAngle());
    laser->SetAngularResolution(calibrated->GetAngularResolution());
    laser->SetRangeThreshold(calibrated->GetRangeThreshold());
    laser->SetIs360Laser(calibrated->GetIs360Laser());
    auto * result = laser.get();
    dataset_->Add(result);  // Dataset owns it from here; no override of sensor names.
    laser.release();
    lasers_.emplace(s.sensor, laser_utils::LaserMetadata(result, metadata.isInverted()));
    return result;
  }

  bool validScan(const Scan & scan) const
  {
    if (scan.ranges.size() < 2 || scan.ranges.size() > 100000 ||
      !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_max) ||
      !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0. ||
      !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0. || scan.range_max <= scan.range_min ||
      scan.angle_max <= scan.angle_min || scan.angle_max - scan.angle_min > 2. * M_PI + .01)
    {return false;}
    const double count = (scan.angle_max - scan.angle_min) / scan.angle_increment + 1.;
    if (std::abs(count - static_cast<double>(scan.ranges.size())) > 1.1) {return false;}
    return std::any_of(scan.ranges.begin(), scan.ranges.end(), [&scan](float range) {
      return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
    });
  }
  void enqueue(size_t index, Scan::ConstSharedPtr scan)
  {
    auto & s = sources_[index]; ++s.received;
    if (!active_ || frozen_ || isPaused(toolbox_types::NEW_MEASUREMENTS) ||
      get_parameter("paused_new_measurements").as_bool())
    {reject(s, "inactive_or_paused"); return;}
    if (scan->header.stamp.sec < 0 || scan->header.stamp.nanosec >= 1000000000u)
    {reject(s, "invalid_stamp_encoding"); return;}
    const int64_t stamp = rclcpp::Time(scan->header.stamp).nanoseconds();
    const double age = (now().nanoseconds() - stamp) / 1.e9;
    if (stamp <= 0 || stamp <= s.received_stamp || age > max_age_ || age < -future_tolerance_)
    {reject(s, "invalid_stale_future_or_out_of_order_stamp"); return;}
    if (scan->header.frame_id != s.sensor || !validScan(*scan))
    {reject(s, "invalid_scan_or_sensor_frame"); return;}
    s.received_stamp = stamp;
    if (s.pending.size() >= queue_size_) {s.pending.pop_front(); reject(s, "queue_overflow");}
    s.pending.push_back({std::move(scan), std::chrono::steady_clock::now()});
  }

  void alignment(const std_msgs::msg::String & message)
  {
    try {
      const auto data = Json::parse(message.data);
      if (!data.at("accepted").get<bool>()) {return;}
      const auto & a = data.at("odom1_from_odom2");
      if (!a.is_array() || a.size() != 3) {throw std::invalid_argument("expected [x,y,yaw]");}
      const Pose p(a.at(0).get<double>(), a.at(1).get<double>(), a.at(2).get<double>());
      if (!finitePose(p)) {throw std::invalid_argument("nonfinite alignment");}
      if (aligned_) {
        const Pose old = pose(odom1_from_odom2_);
        if (old.SquaredDistance(p) > 1.e-12 ||
          std::abs(karto::math::NormalizeAngle(old.GetHeading() - p.GetHeading())) > 1.e-6)
        {failure_ = "alignment_change_requires_cleanup_and_reconfigure"; publishStatus();}
        return;
      }
      odom1_from_odom2_ = transform(p); aligned_ = true; failure_.clear();
      sources_[0].map_from_odom.setIdentity();
      sources_[1].map_from_odom = odom1_from_odom2_;
      // Bootstrap masking before the first scan, with current ROS timestamps.
      if (active_) {broadcast(); publishStatus();}
    } catch (const std::exception & e) {
      failure_ = std::string("invalid_alignment: ") + e.what(); publishStatus();
    }
  }

  // Must hold smapper_mutex_. A loop closure caused by EITHER sensor may move
  // the OTHER sensor's last node. Use stored LOCAL odom, never current TF or
  // the common-odom pose, when computing each map->local_odom transform.
  void refreshTransforms(bool count_edges = false)
  {
    for (auto & s : sources_) {
      if (s.last) {
        s.map_from_odom = transform(s.last->GetCorrectedPose()) * transform(s.last_local).inverse();
      }
    }
    if (!sources_[0].last && sources_[1].last) {
      sources_[0].map_from_odom = sources_[1].map_from_odom * odom1_from_odom2_.inverse();
    } else if (sources_[0].last && !sources_[1].last) {
      sources_[1].map_from_odom = sources_[0].map_from_odom * odom1_from_odom2_;
    }
    if (!count_edges) {return;}
    const auto * graph = smapper_->getMapper()->GetGraph();
    edges_ = cross_edges_ = 0;
    if (graph) {
      edges_ = graph->GetEdges().size();
      for (const auto * edge : graph->GetEdges()) {
        if (edge->GetSource()->GetObject()->GetSensorName() !=
          edge->GetTarget()->GetObject()->GetSensorName()) {++cross_edges_;}
      }
    }
  }

  // Return false only when waiting for TF (bounded by tf_wait_ in pump()).
  bool process(size_t index, const Scan::ConstSharedPtr & original)
  {
    auto & s = sources_[index];
    tf2::Transform local_tf;
    try {
      const auto tf = tf_->lookupTransform(s.odom, s.base, original->header.stamp);
      tf2::fromMsg(tf.transform, local_tf);
      // Check the mount at the scan timestamp even after device registration.
      (void)tf_->lookupTransform(s.base, s.sensor, original->header.stamp);
    } catch (const tf2::TransformException & e) {
      s.reason = std::string("waiting_for_scan_timestamp_tf: ") + e.what(); return false;
    }
    const Pose local = pose(local_tf);
    if (!finitePose(local)) {reject(s, "nonfinite_odometry"); return true;}
    const int64_t stamp = rclcpp::Time(original->header.stamp).nanoseconds();
    ++s.candidates;
    if (s.last && (s.candidates % static_cast<uint64_t>(throttle_scans_) != 0 ||
      stamp - s.accepted_stamp < minimum_time_interval_.nanoseconds() ||
      (local.SquaredDistance(s.last_local) < min_distance_ * min_distance_ &&
      std::abs(karto::math::NormalizeAngle(local.GetHeading() - s.last_local.GetHeading())) < min_heading_)))
    {reject(s, "per_source_rate_or_motion_gate"); return true;}

    // Masked NaN is UNOBSERVED, not a max-range clearing ray. Karto's
    // OccupancyGrid::AddScan explicitly skips NaN; GridIndexLookup also marks
    // NaN/inf INVALID_SCAN. Below-minimum finite readings would be skipped by
    // occupancy insertion but can still enter the matcher's unfiltered lookup,
    // so retain NaN instead. Preserve genuine +infinity unchanged (stock Karto
    // ignores it in occupancy insertion and matching; it does not clear a ray).
    auto cleaned = std::make_shared<Scan>(*original);
    for (auto & range : cleaned->ranges) {
      if (range < cleaned->range_min) {
        range = std::numeric_limits<float>::quiet_NaN();
      }
    }
    Scan::ConstSharedPtr scan = cleaned;
    karto::LaserRangeFinder * laser;
    try {laser = registerLaser(s, scan);}
    catch (const tf2::TransformException & e) {s.reason = e.what(); return false;}
    // A source's angular/range geometry is fixed for a configured graph.
    if (std::abs(laser->GetMinimumAngle() - scan->angle_min) > 1.e-5 ||
      std::abs(laser->GetMaximumAngle() - scan->angle_max) > 1.e-5 ||
      std::abs(laser->GetAngularResolution() - scan->angle_increment) > 1.e-6 ||
      std::abs(laser->GetMinimumRange() - scan->range_min) > 1.e-5 ||
      std::abs(laser->GetMaximumRange() - scan->range_max) > 1.e-4)
    {reject(s, "sensor_geometry_changed"); return true;}

    Pose common = pose((index == 0 ? tf2::Transform::getIdentity() : odom1_from_odom2_) * local_tf);
    boost::mutex::scoped_lock lock(smapper_mutex_);
    std::unique_ptr<karto::LocalizedRangeScan> range(getLocalizedRangeScan(laser, scan, common));
    karto::Matrix3 covariance; covariance.SetToIdentity();
    // Karto 2.8.5 Mapper::Process uses per-sensor last/running scans, AddEdges
    // links other sensors' chains, and TryCloseLoop iterates all sensor names.
    if (!smapper_->getMapper()->Process(range.get(), &covariance))
    {reject(s, "karto_rejected_scan"); return true;}
    const int id = range->GetUniqueId();  // Globally assigned by MapperSensorManager.
    dataset_->Add(range.get());
    s.last = range.release();
    node_info_.emplace(id, NodeInfo{index, local, stamp});
    s.last_local = local; s.accepted_stamp = stamp; s.covariance = covariance;
    ++s.accepted; s.reason.clear();
    if (stamp >= rclcpp::Time(scan_header.stamp).nanoseconds()) {scan_header = scan->header;}
    optimizer_stamp_ = now().nanoseconds();
    refreshTransforms(true);
    return true;
  }

  void broadcast()
  {
    if (!active_ || !aligned_) {return;}
    // Also refresh here to cover any future optimizer entry point. No external
    // graph editing is enabled; Process is currently the only such entry point.
    {boost::mutex::scoped_lock lock(smapper_mutex_); refreshTransforms();}
    const auto stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (auto & s : sources_) {
      geometry_msgs::msg::TransformStamped message;
      message.header.stamp = stamp; message.header.frame_id = map_frame_;
      message.child_frame_id = s.odom;
      message.transform = tf2::toMsg(s.map_from_odom);
      transforms.push_back(message); s.tf_stamp = stamp.nanoseconds();
    }
    tfB_->sendTransform(transforms); last_tf_ = std::chrono::steady_clock::now();
  }

  Json snapshot()
  {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    Json nodes = Json::array();
    const auto * graph = smapper_->getMapper()->GetGraph();
    if (graph) {
      for (const auto & sensor : graph->GetVertices()) {
        for (const auto & item : sensor.second) {
          const auto * scan = item.second->GetObject();
          const auto & info = node_info_.at(scan->GetUniqueId());
          const auto & s = sources_[info.source];
          nodes.push_back({{"id", scan->GetUniqueId()}, {"robot_id", s.robot},
            {"sensor_id", s.sensor}, {"stamp_ns", info.stamp},
            // Exporter binds with robot-local odom. Common odom is separate.
            {"odom", xyz(info.local)}, {"global_odom", xyz(scan->GetOdometricPose())},
            {"map", xyz(scan->GetCorrectedPose())}, {"sensor_map", xyz(scan->GetSensorPose())}});
        }
      }
    }
    std::sort(nodes.begin(), nodes.end(), [](const Json & a, const Json & b) {
      return a.at("id").get<int>() < b.at("id").get<int>();
    });
    // Export raw constraints; a separate diagnostic node evaluates jumps and
    // residuals. Do not put reporting or task policy inside the mapper lock.
    Json constraints = Json::array();
    if (graph) {
      for (auto * edge : graph->GetEdges()) {
        auto * label = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
        if (!label) {continue;}
        Json covariance = Json::array();
        for (size_t r = 0; r < 3; ++r) {
          for (size_t c = 0; c < 3; ++c) {covariance.push_back(label->GetCovariance()(r, c));}
        }
        constraints.push_back({{"source", edge->GetSource()->GetObject()->GetUniqueId()},
          {"target", edge->GetTarget()->GetObject()->GetUniqueId()},
          {"delta", xyz(label->GetPoseDifference())}, {"covariance", covariance}});
      }
    }
    graph_stamp_ = now().nanoseconds();
    return {{"frame_id", map_frame_}, {"epoch", instance_ + ":" + std::to_string(epoch_)},
      {"revision", ++revision_}, {"stamp_ns", graph_stamp_}, {"finalized", finalized_},
      {"frozen", frozen_}, {"nodes", nodes}, {"constraints", constraints}};
  }
  void publishGraph(const Json & graph)
  {std_msgs::msg::String message; message.data = graph.dump(); graph_pub_->publish(message);}

  // Unlike stock updateMap(), always rasterize and publish, even with zero
  // subscribers, so finalize cannot report success without a latched final map.
  bool publishMap()
  {
    if (node_info_.empty()) {return false;}
    nav_msgs::msg::OccupancyGrid message;
    // toNavMap copies cells/dimensions/XY offset only; the stock visualization
    // thread normally initializes these fields. This backend has no such thread.
    message.info.resolution = resolution_;
    message.info.origin.orientation.w = 1.;
    message.info.map_load_time = now();
    {
      boost::mutex::scoped_lock lock(smapper_mutex_);
      std::unique_ptr<karto::OccupancyGrid> grid(smapper_->getOccupancyGrid(resolution_));
      if (!grid) {return false;}
      vis_utils::toNavMap(grid.get(), message);
    }
    message.header.frame_id = map_frame_; message.header.stamp = scan_header.stamp;
    map_.map = message;
    sst_->publish(message); sstm_->publish(message.info);
    map_stamp_ = now().nanoseconds(); return true;
  }
  void publishPoses()
  {
    if (!publish_poses_) {return;}
    for (const auto & s : sources_) {
      if (!s.last) {continue;}
      geometry_msgs::msg::PoseWithCovarianceStamped message;
      message.header.frame_id = map_frame_;
      message.header.stamp = rclcpp::Time(s.accepted_stamp, get_clock()->get_clock_type());
      message.pose.pose.position.x = s.last->GetCorrectedPose().GetX();
      message.pose.pose.position.y = s.last->GetCorrectedPose().GetY();
      message.pose.pose.orientation = tf2::toMsg(transform(s.last->GetCorrectedPose()).getRotation());
      const std::array<size_t, 3> axis{0, 1, 5};
      for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
          const double scale_r = r == 2 ? yaw_covariance_scale_ : position_covariance_scale_;
          const double scale_c = c == 2 ? yaw_covariance_scale_ : position_covariance_scale_;
          message.pose.covariance[6 * axis[r] + axis[c]] =
            s.covariance(r, c) * std::sqrt(scale_r * scale_c);
        }
      }
      s.pose_pub->publish(message);
    }
  }
  void publishStatus()
  {
    if (!active_ || !status_pub_->is_activated()) {return;}
    Json robots = Json::object();
    for (const auto & s : sources_) {
      Json mount = nullptr;
      auto laser = lasers_.find(s.sensor);
      if (laser != lasers_.end()) {mount = xyz(laser->second.getLaser()->GetOffsetPose());}
      robots[s.robot] = {{"sensor_id", s.sensor}, {"scan_topic", s.topic},
        {"odom_frame", s.odom}, {"base_frame", s.base}, {"received", s.received},
        {"accepted", s.accepted}, {"dropped", s.dropped}, {"queued", s.pending.size()},
        {"last_scan_stamp_ns", s.received_stamp}, {"last_accepted_stamp_ns", s.accepted_stamp},
        {"tf_stamp_ns", s.tf_stamp}, {"provisional_tf", s.last == nullptr},
        {"map_from_odom", xyz(pose(s.map_from_odom))}, {"laser_mount", mount},
        {"failure_reason", s.reason}};
    }
    Json data = {{"active", active_}, {"alignment_accepted", aligned_},
      {"frozen", frozen_}, {"finalized", finalized_}, {"frame_id", map_frame_},
      {"global_odom_reference", sources_[0].odom}, {"stamp_ns", now().nanoseconds()},
      {"map_update_stamp_ns", map_stamp_}, {"graph_update_stamp_ns", graph_stamp_},
      {"optimizer_update_stamp_ns", optimizer_stamp_}, {"nodes", node_info_.size()},
      {"edges", edges_}, {"cross_robot_edges", cross_edges_},
      {"failure_reason", failure_}, {"sources", robots}};
    std_msgs::msg::String message; message.data = data.dump(); status_pub_->publish(message);
  }
  void pump()
  {
    if (!active_) {return;}
    try {
      if (aligned_ && !frozen_ && !isPaused(toolbox_types::NEW_MEASUREMENTS) &&
        !get_parameter("paused_new_measurements").as_bool())
      {
        // At most one sample per source per tick; TF waits cannot grow queues.
        for (size_t index = 0; index < sources_.size(); ++index) {
          auto & s = sources_[index];
          if (s.pending.empty()) {continue;}
          const auto & pending = s.pending.front();
          const double age = (now().nanoseconds() -
            rclcpp::Time(pending.scan->header.stamp).nanoseconds()) / 1.e9;
          if (age > max_age_ || age < -future_tolerance_ || elapsed(pending.received, max_age_)) {
            reject(s, "queued_scan_expired_or_clock_reversed"); s.pending.pop_front(); continue;
          }
          if (process(index, pending.scan)) {s.pending.pop_front();}
          else if (elapsed(pending.received, tf_wait_)) {++s.dropped; s.pending.pop_front();}
        }
      }
      if (elapsed(last_tf_, tf_period_)) {broadcast();}
      if (!frozen_ && elapsed(last_map_, map_period_)) {
        if (!node_info_.empty() && !publishMap()) {failure_ = "map_rasterization_failed";}
        last_map_ = std::chrono::steady_clock::now();
      }
      if (elapsed(last_graph_, graph_period_)) {
        publishGraph(snapshot()); publishPoses(); publishStatus();
        last_graph_ = std::chrono::steady_clock::now();
      }
    } catch (const karto::Exception & e) {
      freezeFailure(std::string("karto_exception: ") + e.GetErrorMessage());
    } catch (const std::exception & e) {
      // A mapper/ownership exception can leave partial graph mutation. Freeze
      // rather than continue adding scans into a potentially inconsistent graph.
      freezeFailure(std::string("backend_exception: ") + e.what());
    }
  }
  void freezeFailure(const std::string & reason)
  {
    frozen_ = true; failure_ = reason;
    for (auto & s : sources_) {s.pending.clear();}
    publishStatus(); RCLCPP_ERROR(get_logger(), "%s", failure_.c_str());
  }
  void finalize(std_srvs::srv::Trigger::Response & response)
  {
    if (!active_) {response.message = "SLAM is not active"; return;}
    frozen_ = true;
    for (auto & s : sources_) {s.pending.clear();}
    state_.set(toolbox_types::NEW_MEASUREMENTS, true);
    set_parameter(rclcpp::Parameter("paused_new_measurements", true));
    publishStatus();  // Runtime must stop peer publication before AMCL handover.
    try {
      if (sources_[0].accepted == 0 || sources_[1].accepted == 0) {
        failure_ = "finalize_requires_accepted_scans_from_both_robots";
      } else if (!publishMap()) {failure_ = "final_map_unavailable";}
      else {
        broadcast(); finalized_ = true; failure_.clear();
        auto graph = snapshot(); publishGraph(graph); publishPoses(); publishStatus();
        response.success = true; response.message = graph.dump(); return;
      }
    } catch (const karto::Exception & e) {
      finalized_ = false; failure_ = std::string("finalize_karto_exception: ") + e.GetErrorMessage();
    } catch (const std::exception & e) {
      finalized_ = false; failure_ = std::string("finalize_exception: ") + e.what();
    }
    publishStatus(); response.message = failure_;
  }

protected:
  // No stock scan path, graph import/reset, or interactive optimizer: those
  // lack per-node local odom metadata required by this backend's TF/export.
  void laserCallback(Scan::ConstSharedPtr) override {}
  bool deserializePoseGraphCallback(const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request>,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response>) override
  {return false;}
  bool resetCallback(const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<slam_toolbox::srv::Reset::Request>,
    std::shared_ptr<slam_toolbox::srv::Reset::Response>) override
  {return false;}

public:
  explicit JointSlam(const rclcpp::NodeOptions & options) : SlamToolbox(options) {}
  void loadPoseGraphByParams() override
  {
    if (!param<std::string>("map_file_name", "").empty()) {
      throw std::invalid_argument("Joint SLAM cannot restore graphs without local odom metadata");
    }
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    // Reject conflicting options before constructing the base graph/helpers.
    if (param<double>("transform_publish_period", 0.) != 0. ||
      param<bool>("enable_interactive_mode", false) ||
      param<std::string>("map_name", "/map") != "/map" ||
      !param<bool>("use_scan_matching", true))
    {
      RCLCPP_ERROR(get_logger(), "Require transform_publish_period=0, enable_interactive_mode=false, "
        "map_name=/map, use_scan_matching=true"); return CallbackReturn::FAILURE;
    }
    queue_size_ = static_cast<size_t>(param<int>("joint_scan_queue_size", 5));
    max_age_ = param<double>("joint_scan_max_age_s", 2.);
    tf_wait_ = param<double>("joint_tf_wait_s", .5);
    future_tolerance_ = param<double>("joint_scan_future_tolerance_s", .1);
    graph_period_ = param<double>("anchor_graph_publish_period_s", .5);
    tf_period_ = param<double>("joint_tf_publish_period_s", .05);
    map_period_ = param<double>("map_update_interval", 5.);
    publish_poses_ = param<bool>("joint_publish_poses", true);
    for (double value : {max_age_, tf_wait_, graph_period_, tf_period_, map_period_}) {
      if (!std::isfinite(value) || value <= 0.) {return CallbackReturn::FAILURE;}
    }
    if (queue_size_ == 0 || queue_size_ > 1000 || !std::isfinite(future_tolerance_) ||
      future_tolerance_ < 0.) {return CallbackReturn::FAILURE;}
    for (size_t i = 0; i < sources_.size(); ++i) {
      auto & s = sources_[i]; s = Source{}; s.robot = "robot" + std::to_string(i + 1);
      s.topic = param<std::string>(s.robot + "_scan_topic", "/" + s.robot + "/scan_slam");
      s.base = param<std::string>(s.robot + "_base_frame", s.robot + "/base_link");
      s.odom = param<std::string>(s.robot + "_odom_frame", s.robot + "/odom");
      s.sensor = param<std::string>(s.robot + "_sensor_frame", s.robot + "/laser");
      for (const auto & frame : {s.base, s.odom, s.sensor}) {
        if (frame.empty() || frame.front() == '/' || frame == param<std::string>("map_frame", "map"))
        {return CallbackReturn::FAILURE;}
      }
      if (s.base == s.odom || s.base == s.sensor || s.odom == s.sensor || s.topic.empty())
      {return CallbackReturn::FAILURE;}
    }
    if (sources_[0].topic == sources_[1].topic) {return CallbackReturn::FAILURE;}
    for (const auto & a : {sources_[0].base, sources_[0].odom, sources_[0].sensor}) {
      for (const auto & b : {sources_[1].base, sources_[1].odom, sources_[1].sensor}) {
        if (a == b) {return CallbackReturn::FAILURE;}
      }
    }
    const auto result = SlamToolbox::on_configure(state);
    if (result != CallbackReturn::SUCCESS) {return result;}
    if (throttle_scans_ < 1 || minimum_time_interval_.nanoseconds() < 0 ||
      !std::isfinite(resolution_) || resolution_ <= 0. ||
      !std::isfinite(position_covariance_scale_) || position_covariance_scale_ < 0. ||
      !std::isfinite(yaw_covariance_scale_) || yaw_covariance_scale_ < 0.)
    {SlamToolbox::on_cleanup(state); return CallbackReturn::FAILURE;}
    min_distance_ = smapper_->getMapper()->getParamMinimumTravelDistance();
    min_heading_ = smapper_->getMapper()->getParamMinimumTravelHeadingInRadians();
    if (!std::isfinite(min_distance_) || min_distance_ < 0. ||
      !std::isfinite(min_heading_) || min_heading_ < 0.)
    {SlamToolbox::on_cleanup(state); return CallbackReturn::FAILURE;}
    ++epoch_; revision_ = 0; node_info_.clear();
    state_.set(toolbox_types::NEW_MEASUREMENTS, false);
    map_ = nav_msgs::srv::GetMap::Response();
    aligned_ = frozen_ = finalized_ = false; failure_.clear();
    map_stamp_ = graph_stamp_ = optimizer_stamp_ = 0; edges_ = cross_edges_ = 0;
    scan_header = std_msgs::msg::Header();
    for (size_t i = 0; i < sources_.size(); ++i) {
      auto & s = sources_[i];
      s.assistant = std::make_unique<laser_utils::LaserAssistant>(shared_from_this(), tf_.get(), s.base);
      s.subscription = create_subscription<Scan>(s.topic,
        rclcpp::SensorDataQoS().keep_last(queue_size_),
        [this, i](Scan::ConstSharedPtr scan) {enqueue(i, std::move(scan));});
      s.pose_pub = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/" + s.robot + "/slam_pose", rclcpp::QoS(1));
    }
    graph_pub_ = create_publisher<std_msgs::msg::String>(
      "/slam_toolbox/anchor_graph", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/joint_mapping/status", rclcpp::QoS(1).transient_local());
    alignment_sub_ = create_subscription<std_msgs::msg::String>(
      param<std::string>("alignment_topic", "/joint_mapping/alignment"),
      rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {alignment(*msg);});
    finalize_ = create_service<std_srvs::srv::Trigger>("/slam_toolbox/finalize_anchors",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {finalize(*response);});
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {pump();});
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Use base ROS/map services but not its single-robot TF/visualization
    // threads or global shouldProcessScan gate. One serialized timer owns work.
    setROSInterfaces(); scan_filter_.reset(); scan_filter_sub_.reset();
    ssDesserialize_.reset(); ssReset_.reset();
    sst_->on_activate(); sstm_->on_activate(); pose_pub_->on_activate();
    graph_pub_->on_activate(); status_pub_->on_activate();
    for (auto & s : sources_) {s.pose_pub->on_activate();}
    reprocessing_transform_.setIdentity();
    active_ = true;
    last_tf_ = last_graph_ = last_map_ = std::chrono::steady_clock::time_point{};
    if (aligned_) {broadcast();}
    publishStatus();
    if (use_lifecycle_manager_) {createBond();}
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    // Publish inactive before deactivating the latched status publisher.
    const bool was_frozen = frozen_; frozen_ = true; active_ = false; publishStatus();
    frozen_ = was_frozen;
    for (auto & s : sources_) {s.pending.clear(); s.pose_pub->on_deactivate();}
    graph_pub_->on_deactivate(); status_pub_->on_deactivate();
    return SlamToolbox::on_deactivate(state);
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    active_ = false; timer_.reset(); alignment_sub_.reset(); finalize_.reset();
    graph_pub_.reset(); status_pub_.reset();
    for (auto & s : sources_) {s = Source{};}
    node_info_.clear();
    return SlamToolbox::on_cleanup(state);
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    if (active_) {on_deactivate(state);}
    return SlamToolbox::on_shutdown(state);
  }
};
}  // namespace edgeai_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<edgeai_mapping::JointSlam>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
