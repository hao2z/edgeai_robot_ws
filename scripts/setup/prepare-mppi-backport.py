"""Backport Nav2 main PR #6072 to the pinned Jazzy xtensor implementation.

Upstream: ec1c42d36e85f70574aaa21a9f587585d1de3258 (Apache-2.0).
Only generated cache sources are written. Exact matches fail closed on drift.
The Eigen expressions are translated to equivalent scalar/xtensor operations.
"""
import pathlib
import shutil
import sys

UPSTREAM = "ec1c42d36e85f70574aaa21a9f587585d1de3258"


def prepare(source, target):
    shutil.copytree(source, target, dirs_exist_ok=True)

    def patch(relative, old, new):
        path = target / relative
        text = path.read_text()
        if text.count(old) != 1:
            raise RuntimeError(f"Unexpected pinned source: {relative}: {old[:65]!r}")
        path.write_text(text.replace(old, new))

    base = "include/nav2_mppi_controller/"
    patch(base + "models/optimizer_settings.hpp", "  float model_dt{0.0f};",
          "  float model_dt{0.0f};\n  float controller_period{0.0f};\n  unsigned int sgf_order{2u};")
    patch(base + "optimizer.hpp", "  void generateNoisedTrajectories();",
          "  void generateNoisedTrajectories();\n  void applyControlSequenceInterIterationConstraints();")

    # Keep raw controls untouched; only the predicted physical state is bounded.
    for axis, max_delta, min_delta in (
        ("x", "max_delta_vx", "min_delta_vx"),
        ("y", "max_delta_vy", "min_delta_vy"),
    ):
        cv = f"cv{axis}_curr"
        last = f"v{axis}_last"
        old = f"""float & {cv} = state.cv{axis}(i, j - 1);
        if ({last} > 0) {{
          {cv} = std::clamp({cv}, {last} + {min_delta}, {last} + {max_delta});
        }} else {{
          {cv} = std::clamp({cv}, {last} - {max_delta}, {last} - {min_delta});
        }}
        state.v{axis}(i, j) = {cv};
        {last} = {cv};"""
        # The holonomic section has two extra indentation spaces.
        indent = "  " if axis == "y" else ""
        old = old.replace("\n", "\n" + indent)
        new = f"""const float {cv} = state.cv{axis}(i, j - 1);
        state.v{axis}(i, j) = ({last} >= 0) ?
          std::clamp({cv}, {last} + {min_delta}, {last} + {max_delta}) :
          std::clamp({cv}, {last} - {max_delta}, {last} - {min_delta});
        {last} = state.v{axis}(i, j);""".replace("\n", "\n" + indent)
        patch(base + "motion_models.hpp", old, new)
    patch(base + "motion_models.hpp", """float & cwz_curr = state.cwz(i, j - 1);
        cwz_curr = std::clamp(cwz_curr, wz_last - max_delta_wz, wz_last + max_delta_wz);
        state.wz(i, j) = cwz_curr;
        wz_last = cwz_curr;""", """const float cwz_curr = state.cwz(i, j - 1);
        state.wz(i, j) = std::clamp(cwz_curr, wz_last - max_delta_wz, wz_last + max_delta_wz);
        wz_last = state.wz(i, j);""")

    cpp = "src/optimizer.cpp"
    patch(cpp, '  getParam(s.open_loop, "open_loop", false);', '''  getParam(s.open_loop, "open_loop", false);
  getParam(s.sgf_order, "sgf_order", 2);
  if (s.sgf_order < 1 || s.sgf_order > 2) {
    RCLCPP_WARN(logger_, "sgf_order must be 1 or 2, defaulting to 2");
    s.sgf_order = 2;
  }''')
    patch(cpp, "  const double controller_period = 1.0 / controller_frequency;",
          "  const double controller_period = 1.0 / controller_frequency;\n"
          "  settings_.controller_period = static_cast<float>(controller_period);")
    patch(cpp, """  if (settings_.open_loop) {
    last_command_vel_ = geometry_msgs::msg::Twist();
  }""", "  last_command_vel_ = geometry_msgs::msg::Twist();")
    patch(cpp, "  state_.speed = settings_.open_loop ? last_command_vel_ : robot_speed;", """  if (settings_.open_loop) {
    state_.speed = last_command_vel_;
  } else {
    // PR #6072: compensate one control period of measurement/actuation latency.
    const auto & c = settings_.constraints;
    const double dt = settings_.controller_period;
    state_.speed = robot_speed;
    state_.speed.linear.x = std::clamp(last_command_vel_.linear.x,
      robot_speed.linear.x + dt * c.ax_min, robot_speed.linear.x + dt * c.ax_max);
    state_.speed.angular.z = std::clamp(last_command_vel_.angular.z,
      robot_speed.angular.z - dt * c.az_max, robot_speed.angular.z + dt * c.az_max);
    if (isHolonomic()) {
      state_.speed.linear.y = std::clamp(last_command_vel_.linear.y,
        robot_speed.linear.y + dt * c.ay_min, robot_speed.linear.y + dt * c.ay_max);
    }
  }""")
    patch(cpp, """void Optimizer::generateNoisedTrajectories()
{
  noise_generator_""", """void Optimizer::generateNoisedTrajectories()
{
  applyControlSequenceInterIterationConstraints();
  noise_generator_""")
    path = target / cpp
    text = path.read_text()
    start = text.index("void Optimizer::applyControlSequenceConstraints()")
    end = text.index("void Optimizer::updateStateVelocities(", start)
    text = text[:start] + '''// Backported from Nav2 PR #6072; xtensor scalar indexing replaces Eigen.
void Optimizer::applyControlSequenceInterIterationConstraints()
{
  auto & s = settings_;
  const float dt = s.controller_period;
  const float vx = static_cast<float>(state_.speed.linear.x);
  const float wz = static_cast<float>(state_.speed.angular.z);
  if (s.shift_control_sequence) {
    // Index zero represents now; index one is the command actually sent.
    control_sequence_.vx(0) = vx;
    control_sequence_.wz(0) = wz;
    if (isHolonomic()) {
      control_sequence_.vy(0) = static_cast<float>(state_.speed.linear.y);
    }
  } else {
    control_sequence_.vx(0) = utils::clampVelocityByAccel(
      vx, control_sequence_.vx(0), dt * s.constraints.ax_min, dt * s.constraints.ax_max);
    control_sequence_.wz(0) = utils::clampVelocityByAccel(
      wz, control_sequence_.wz(0), -dt * s.constraints.az_max, dt * s.constraints.az_max);
    if (isHolonomic()) {
      control_sequence_.vy(0) = utils::clampVelocityByAccel(
        static_cast<float>(state_.speed.linear.y), control_sequence_.vy(0),
        dt * s.constraints.ay_min, dt * s.constraints.ay_max);
    }
  }
}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;
  motion_model_->applyConstraints(control_sequence_);
  float vx_last = static_cast<float>(state_.speed.linear.x);
  float wz_last = static_cast<float>(state_.speed.angular.z);
  float vy_last = isHolonomic() ? static_cast<float>(state_.speed.linear.y) : 0.0f;
  if (s.shift_control_sequence) {
    control_sequence_.vx(0) = vx_last;
    control_sequence_.wz(0) = wz_last;
    if (isHolonomic()) {control_sequence_.vy(0) = vy_last;}
  }
  for (size_t i = 0; i < control_sequence_.vx.size(); ++i) {
    const float dt = i == 0 ? s.controller_period : s.model_dt;
    float & vx = control_sequence_.vx(i);
    vx = std::clamp(vx, s.constraints.vx_min, s.constraints.vx_max);
    vx = utils::clampVelocityByAccel(
      vx_last, vx, dt * s.constraints.ax_min, dt * s.constraints.ax_max);
    vx_last = vx;
    float & wz = control_sequence_.wz(i);
    wz = std::clamp(wz, -s.constraints.wz, s.constraints.wz);
    wz = utils::clampVelocityByAccel(
      wz_last, wz, -dt * s.constraints.az_max, dt * s.constraints.az_max);
    wz_last = wz;
    if (isHolonomic()) {
      float & vy = control_sequence_.vy(i);
      vy = std::clamp(vy, -s.constraints.vy, s.constraints.vy);
      vy = utils::clampVelocityByAccel(
        vy_last, vy, dt * s.constraints.ay_min, dt * s.constraints.ay_max);
      vy_last = vy;
    }
  }
  motion_model_->applyConstraints(control_sequence_);
}

''' + text[end:]
    path.write_text(text)
    patch(cpp, '  RCLCPP_INFO(logger_, "Optimizer reset");',
          '  RCLCPP_INFO(logger_, "Optimizer reset (Jazzy backport: Nav2 PR #6072)");')
    utils = base + "tools/utils.hpp"
    patch(utils, """  // Savitzky-Golay Quadratic, 9-point Coefficients
  xt::xarray<float> filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
  filter /= 231.0;""", """  xt::xarray<float> filter;
  if (settings.sgf_order == 1) {
    filter = xt::ones<float>({9});
    filter /= 9.0f;
  } else {
    filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
    filter /= 231.0f;
  }""")
    patch(utils, "}  // namespace mppi::utils", """// PR #6072: max accelerates speed, min decelerates, including reverse travel.
inline float clampVelocityByAccel(
  float last_vel, float curr_vel, float min_delta, float max_delta)
{
  if (last_vel >= 0) {
    return std::clamp(curr_vel, last_vel + min_delta, last_vel + max_delta);
  }
  return std::clamp(curr_vel, last_vel - max_delta, last_vel - min_delta);
}

}  // namespace mppi::utils""")
    # Independent environment defect: xsimd cos(0) evaluates to -1 when
    # compiled with -ffast-math in this toolchain, reversing rollout movement.
    # Keep SIMD and -O3; prohibit unsafe floating-point algebra in MPPI.
    patch("CMakeLists.txt", " -ffast-math", " -fno-fast-math")
    (target / "edgeai-backport-revision").write_text(UPSTREAM + "\nstrict-math-v1\n")
    patch("CMakeLists.txt", "ament_package()", """if(EDGEAI_BACKPORT_TEST_SOURCE)
  add_executable(edgeai_mppi_backport_check ${EDGEAI_BACKPORT_TEST_SOURCE})
  target_link_libraries(edgeai_mppi_backport_check mppi_controller)
  ament_target_dependencies(edgeai_mppi_backport_check ${dependencies_pkgs})
endif()

ament_package()""")


if __name__ == "__main__":
    prepare(*map(pathlib.Path, sys.argv[1:]))
