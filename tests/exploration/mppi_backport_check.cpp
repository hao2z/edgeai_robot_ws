// Behavioral regressions for the Jazzy translation of Nav2 PR #6072.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "nav2_mppi_controller/optimizer.hpp"

void near(float actual, float expected, const char * what)
{
  if (std::abs(actual - expected) > 1e-6f) {
    throw std::runtime_error(std::string(what) + ": " + std::to_string(actual) +
      " expected " + std::to_string(expected));
  }
}

class Harness : public mppi::Optimizer
{
public:
  using Optimizer::applyControlSequenceConstraints;
  using Optimizer::applyControlSequenceInterIterationConstraints;
  using Optimizer::control_sequence_;
  using Optimizer::settings_;
  using Optimizer::state_;
  using Optimizer::costs_;
  using Optimizer::generated_trajectories_;
  using Optimizer::integrateStateVelocities;
  using Optimizer::updateControlSequence;
  Harness()
  {
    settings_.constraints = {0.2f, -0.08f, 0.2f, 0.65f,
      0.4f, -0.8f, 0.4f, -0.8f, 0.8f};
    settings_.model_dt = 0.05f;
    settings_.controller_period = 0.05f;
    settings_.shift_control_sequence = true;
    control_sequence_.reset(8);
    motion_model_ = std::make_shared<mppi::DiffDriveMotionModel>();
    motion_model_->initialize(settings_.constraints, settings_.model_dt);
  }
};

int main()
{
  Harness h;
  auto & s = h.settings_;
  auto & q = h.control_sequence_;
  // Raw samples must remain raw, while physical rollouts respect acceleration.
  mppi::models::State rollout;
  rollout.reset(2, 8);
  rollout.cvx.fill(0.8f);
  rollout.cwz.fill(1.0f);
  mppi::DiffDriveMotionModel model;
  model.initialize(s.constraints, s.model_dt);
  model.predict(rollout);
  near(rollout.cvx(0, 0), 0.8f, "raw vx preserved");
  near(rollout.cwz(0, 0), 1.0f, "raw wz preserved");
  near(rollout.vx(0, 1), 0.02f, "physical vx acceleration");
  near(rollout.wz(0, 1), 0.04f, "physical wz acceleration");
  rollout.reset(2, 8);
  rollout.cvx.fill(-0.8f);
  model.predict(rollout);
  near(rollout.vx(0, 1), -0.04f, "zero-speed asymmetric lower bound");
  near(rollout.cvx(0, 0), -0.8f, "negative raw vx preserved");

  // Shifted output must be one, not two, acceleration steps from current speed.
  q.vx.fill(0.2f);
  q.wz.fill(0.65f);
  h.applyControlSequenceInterIterationConstraints();
  near(q.vx(0), 0, "shifted center pinned");
  h.applyControlSequenceConstraints();
  near(q.vx(0), 0, "shifted zero represents now");
  near(q.vx(1), 0.02f, "shifted command one linear step");
  near(q.wz(1), 0.04f, "shifted command one angular step");
  for (size_t i = 1; i < q.vx.size(); ++i) {
    if (q.vx(i)-q.vx(i-1) > 0.020001f) {
      throw std::runtime_error("intra-sequence acceleration violated");
    }
  }
  // A faster controller uses its own period for the first command.
  s.shift_control_sequence = false;
  s.controller_period = 0.025f;
  q.vx.fill(0.2f);
  q.wz.fill(0.65f);
  h.applyControlSequenceInterIterationConstraints();
  near(q.vx(0), 0.01f, "nonshift center uses controller period");
  h.applyControlSequenceConstraints();
  near(q.vx(0), 0.01f, "nonshift sent command uses controller period");
  near(q.vx(1), 0.03f, "remaining sequence uses model period");
  near(q.wz(0), 0.02f, "nonshift angular period");
  // Forward braking and reverse acceleration with asymmetric limits.
  h.state_.speed.linear.x = 0.1;
  q.vx.fill(-0.08f);
  h.applyControlSequenceConstraints();
  near(q.vx(0), 0.08f, "forward braking remains feasible");
  h.state_.speed.linear.x = -0.04;
  q.vx.fill(-0.08f);
  h.applyControlSequenceConstraints();
  near(q.vx(0), -0.05f, "reverse acceleration respects magnitude");

  // Independent deterministic scoring/softmax check, with no simulator or TF.
  // Actual MPPI rollout and update functions must agree with scalar arithmetic.
  h.state_.reset(2, 8);
  h.state_.speed = geometry_msgs::msg::Twist();
  h.state_.pose.pose.orientation.w = 1;
  q.reset(8);
  s.temperature = 0.3f;
  s.gamma = 0.015f;
  s.sampling_std.vx = 0.05f;
  s.sampling_std.wz = 0.1f;
  s.sampling_std.vy = 0.0f;
  h.generated_trajectories_.reset(2, 8);
  for (size_t j = 0; j < 8; ++j) {
    h.state_.cvx(0,j) = h.state_.vx(0,j) = 0.1f;
    h.state_.cvx(1,j) = h.state_.vx(1,j) = -0.1f;
  }
  h.integrateStateVelocities(h.generated_trajectories_, h.state_);
  std::cout << "Numeric probe x0=" << h.state_.pose.pose.position.x << " dt=" << s.model_dt;
  for (size_t row = 0; row < 2; ++row) {
    std::cout << "\nrow=" << row << " vx/yaw/x:";
    for (size_t j = 0; j < 8; ++j) {
      std::cout << " (" << h.state_.vx(row,j) << "," << h.generated_trajectories_.yaws(row,j)
        << "," << h.generated_trajectories_.x(row,j) << ")";
    }
  }
  std::cout << std::endl;
  near(h.generated_trajectories_.x(0,7), 0.04f, "positive rollout x");
  near(h.generated_trajectories_.x(1,7), -0.04f, "negative rollout x");
  h.costs_ = xt::eval(5.0f * xt::mean(xt::hypot(
    h.generated_trajectories_.x - 0.6f, h.generated_trajectories_.y), {1},
    xt::evaluation_strategy::immediate));
  near(h.costs_(0), 2.8875f, "forward mean goal cost");
  near(h.costs_(1), 3.1125f, "reverse mean goal cost");
  h.updateControlSequence();
  if (!(q.vx(0) > 0)) {
    throw std::runtime_error("lower forward cost did not produce positive control");
  }
  std::cout << "PASS: raw samples, physical rollouts, shifted/nonshifted boundaries, "
    "controller/model timing, asymmetric acceleration, deterministic rollout/goal/softmax\n";
}
