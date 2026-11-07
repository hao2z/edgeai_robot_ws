#pragma once
#include <cstdint>
#include "recovery_geometry.hpp"

namespace edgeai_navigation
{
class RecoveryHistory
{
public:
  bool tried(RecoveryPose pose, const RecoveryMotion & motion) const
  {
    for (const auto & item : attempts_) {
      const double yaw = std::atan2(std::sin(pose.yaw - item.pose.yaw), std::cos(pose.yaw - item.pose.yaw));
      if (item.motion.kind == motion.kind && std::abs(item.motion.amount - motion.amount) < .001 &&
        std::hypot(pose.x - item.pose.x, pose.y - item.pose.y) < .20 && std::abs(yaw) < .35) {return true;}
    }
    return false;
  }
  void add(RecoveryPose pose, RecoveryMotion motion)
  {attempts_.push_back({pose, motion}); if (attempts_.size() > 64) {attempts_.erase(attempts_.begin());}}
  void clear() {attempts_.clear();}
  size_t size() const {return attempts_.size();}
private:
  struct Attempt {RecoveryPose pose; RecoveryMotion motion;};
  std::vector<Attempt> attempts_;
};

// Counts sustained, measured translation, never just a successful action result.
class TrackingProgress
{
public:
  void failed() {active_ = false;}
  void observe(double now, RecoveryPose pose, bool moving)
  {
    if (!moving) {failed(); return;}
    const bool gap = now < last_time_ || now - last_time_ > .5;
    last_time_ = now;
    if (!active_ || gap) {anchor_ = pose; since_ = now; active_ = true; return;}
    if (now - since_ >= 2.0 && std::hypot(pose.x - anchor_.x, pose.y - anchor_.y) >= .30) {
      ++epoch_; anchor_ = pose; since_ = now;
    }
  }
  uint64_t epoch() const {return epoch_;}
private:
  bool active_ = false;
  double since_ = 0.0;
  double last_time_ = -1.;
  RecoveryPose anchor_{};
  uint64_t epoch_ = 0;
};
}  // namespace edgeai_navigation
