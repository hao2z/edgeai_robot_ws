#include <algorithm>
#include <cmath>
#include <mutex>
#include <optional>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "peer_raster.hpp"

namespace edgeai_navigation
{
class PeerFootprintLayer : public nav2_costmap_2d::Layer
{
  using Pose = geometry_msgs::msg::PoseStamped;
  struct Body {double x, y, c, s, reach;};

public:
  void onInitialize() override
  {
    auto node = node_.lock();
    declareParameter("pose_topic", rclcpp::ParameterValue(std::string("peer_pose")));
    declareParameter("bounds", rclcpp::ParameterValue(std::vector<double>{-.3125, .1125, -.165, .165}));
    declareParameter("padding_m", rclcpp::ParameterValue(.03));
    declareParameter("max_age_s", rclcpp::ParameterValue(.5));
    std::string topic;
    node->get_parameter(getFullName("pose_topic"), topic);
    node->get_parameter(getFullName("bounds"), bounds_);
    node->get_parameter(getFullName("padding_m"), padding_);
    node->get_parameter(getFullName("max_age_s"), max_age_);
    if (bounds_.size() != 4 ||
      !std::all_of(bounds_.begin(), bounds_.end(), [](double v) {return std::isfinite(v);}) ||
      bounds_[0] >= bounds_[1] || bounds_[2] >= bounds_[3] ||
      !std::isfinite(padding_) || padding_ < 0 || !std::isfinite(max_age_) || max_age_ <= 0)
    {
      throw std::runtime_error("Invalid peer footprint layer parameters");
    }
    reach_ = std::hypot(std::max(std::abs(bounds_[0]), std::abs(bounds_[1])) + padding_,
      std::max(std::abs(bounds_[2]), std::abs(bounds_[3])) + padding_);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    sub_ = node->create_subscription<Pose>(topic, rclcpp::QoS(1),
      [this](Pose::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = msg;
      }, options);
    current_ = enabled_ = true;
  }

  void updateBounds(double, double, double, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    const auto include = [&](const Body & b) {
        *min_x = std::min(*min_x, b.x - b.reach);
        *min_y = std::min(*min_y, b.y - b.reach);
        *max_x = std::max(*max_x, b.x + b.reach);
        *max_y = std::max(*max_y, b.y + b.reach);
      };
    // Reset the old region through the layered map, preserving static/scan costs.
    if (body_) {include(*body_);}
    body_.reset();
    Pose::ConstSharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = latest_;
    }
    if (!msg || msg->header.frame_id.empty()) {return;}
    const auto age = (clock_->now() - rclcpp::Time(msg->header.stamp, clock_->get_clock_type())).seconds();
    if (age < -.05 || age > max_age_) {return;}
    Pose pose = *msg;
    try {
      const auto & frame = layered_costmap_->getGlobalFrameID();
      if (frame != pose.header.frame_id) {pose = tf_->transform(pose, frame);}
    } catch (const tf2::TransformException &) {return;}
    const double x = pose.pose.position.x, y = pose.pose.position.y, yaw = tf2::getYaw(pose.pose.orientation);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {return;}
    body_ = Body{x, y, std::cos(yaw), std::sin(yaw), reach_ + layered_costmap_->getCostmap()->getResolution()};
    include(*body_);
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & grid, int min_i, int min_j,
    int max_i, int max_j) override
  {
    if (!body_) {return;}
    const auto & b = *body_;
    int x0, y0, x1, y1;
    grid.worldToMapEnforceBounds(b.x - b.reach, b.y - b.reach, x0, y0);
    grid.worldToMapEnforceBounds(b.x + b.reach, b.y + b.reach, x1, y1);
    for (int y = std::max(y0, min_j); y < std::min(y1 + 1, max_j); ++y) {
      for (int x = std::max(x0, min_i); x < std::min(x1 + 1, max_i); ++x) {
        double wx, wy;
        grid.mapToWorld(x, y, wx, wy);
        const double dx = wx - b.x, dy = wy - b.y;
        if (peerCellIntersects(dx, dy, b.c, b.s, bounds_[0], bounds_[1],
          bounds_[2], bounds_[3], padding_, grid.getResolution()))
        {
          grid.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

  void reset() override {current_ = true;}
  bool isClearable() override {return false;}

private:
  std::mutex mutex_;
  Pose::ConstSharedPtr latest_;
  rclcpp::Subscription<Pose>::SharedPtr sub_;
  std::optional<Body> body_;
  std::vector<double> bounds_;
  double padding_, max_age_, reach_;
};
}  // namespace edgeai_navigation

PLUGINLIB_EXPORT_CLASS(edgeai_navigation::PeerFootprintLayer, nav2_costmap_2d::Layer)
