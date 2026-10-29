"""Prepare a generated upstream source copy with opt-in scoring diagnostics."""
import pathlib
import shutil
import sys

source, target = map(pathlib.Path, sys.argv[1:])
shutil.copytree(source, target, dirs_exist_ok=True)


def patch(relative, old, new):
    path = target / relative
    text = path.read_text()
    if text.count(old) != 1:
        raise RuntimeError(f"Unexpected upstream source: {relative}")
    path.write_text(text.replace(old, new))


patch("src/critic_manager.cpp", '#include "nav2_mppi_controller/critic_manager.hpp"',
      '#include "nav2_mppi_controller/critic_manager.hpp"\n#include <cstdlib>\n#include <limits>')
patch("src/critic_manager.cpp", "  for (const auto & critic : critics_) {", """  static size_t cycle = 0;
  const bool probe = std::getenv("EDGEAI_MPPI_SCORE_PROBE") && (++cycle % 10 == 1);
  size_t index = 0;
  for (const auto & critic : critics_) {""")
patch("src/critic_manager.cpp", "    critic->score(data);", """    const auto before = data.costs;
    critic->score(data);
    if (probe) {
      size_t forward = 0, reverse = 0;
      float fc = std::numeric_limits<float>::infinity();
      float rc = fc;
      const auto last = data.trajectories.x.shape(1) - 1;
      const auto yaw = tf2::getYaw(data.state.pose.pose.orientation);
      for (size_t k = 0; k < data.costs.size(); ++k) {
        const double progress =
          (data.trajectories.x(k,last)-data.state.pose.pose.position.x)*std::cos(yaw) +
          (data.trajectories.y(k,last)-data.state.pose.pose.position.y)*std::sin(yaw);
        if (progress > 0 && data.costs(k) < fc) {fc=data.costs(k); forward=k;}
        if (progress < 0 && data.costs(k) < rc) {rc=data.costs(k); reverse=k;}
      }
      RCLCPP_INFO(logger_, "SCORE cycle=%zu critic=%s goal=(%.4f,%.4f) pose=(%.4f,%.4f) forward=[%zu total=%.5f delta=%.5f endx=%.4f] reverse=[%zu total=%.5f delta=%.5f endx=%.4f] fail=%d",
        cycle, critic_names_[index].c_str(), data.goal.position.x, data.goal.position.y,
        data.state.pose.pose.position.x, data.state.pose.pose.position.y,
        forward, fc, data.costs(forward)-before(forward), data.trajectories.x(forward,last),
        reverse, rc, data.costs(reverse)-before(reverse), data.trajectories.x(reverse,last), data.fail_flag);
    }
    ++index;""")
patch("src/optimizer.cpp", '#include "nav2_mppi_controller/optimizer.hpp"',
      '#include "nav2_mppi_controller/optimizer.hpp"\n#include <cstdlib>')
patch("src/optimizer.cpp", "  applyControlSequenceConstraints();\n}", """  applyControlSequenceConstraints();
  static size_t probe_cycle = 0;
  if (std::getenv("EDGEAI_MPPI_SCORE_PROBE") && (++probe_cycle % 10 == 1)) {
    size_t best = 0;
    double forward_weight = 0, reverse_weight = 0;
    for (size_t k = 0; k < costs_.size(); ++k) {
      if (costs_(k) < costs_(best)) {best=k;}
      if (state_.cvx(k,0)>0) {forward_weight+=softmaxes(k);}
      else {reverse_weight+=softmaxes(k);}
    }
    const auto last = generated_trajectories_.x.shape(1)-1;
    RCLCPP_INFO(logger_, "SCORE_UPDATE cycle=%zu best=%zu cost=%.5f weight=%.5f cvx0=%.5f vx1=%.5f predicted_end=(%.4f,%.4f) goal=(%.4f,%.4f) forward_weight=%.5f reverse_weight=%.5f output0=%.5f output1=%.5f",
      probe_cycle,best,costs_(best),softmaxes(best),state_.cvx(best,0),state_.vx(best,1),
      generated_trajectories_.x(best,last),generated_trajectories_.y(best,last),
      goal_.position.x,goal_.position.y,forward_weight,reverse_weight,
      control_sequence_.vx(0),control_sequence_.vx(1));
  }
}""")
