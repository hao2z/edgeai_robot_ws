// Project adapter: history survives suppression TTLs, but never survives a new mission.
namespace frontier_exploration_ros2
{
double FrontierExplorerCore::repeat_known_area(const std::pair<double, double> & p) const
{
  if (!map.has_value()) {return 0.0;}
  // World-fixed 10 cm samples in a 3 m square avoid counting map resize as gain.
  int known = 0;
  for (int y = -15; y <= 15; ++y) {
    for (int x = -15; x <= 15; ++x) {
      int mx = 0, my = 0;
      if (map->worldToMapNoThrow(p.first + x * 0.1, p.second + y * 0.1, mx, my) &&
        map->getCost(mx, my) != static_cast<int>(OccupancyGrid2d::CostValues::NoInformation))
      {++known;}
    }
  }
  return known * 0.01;
}

void FrontierExplorerCore::finish_repeat_attempt(const FrontierLike & frontier, double before)
{
  if (!params.repeat_guard_enabled) {return;}
  const auto p = frontier.center_point;
  const auto now = callbacks.now_ns();
  auto it = std::find_if(repeat_memory_.begin(), repeat_memory_.end(), [&](const auto & r) {
    return std::hypot(r.point.first-p.first, r.point.second-p.second) <=
           std::max(0.05, params.repeat_guard_match_distance_m);
  });
  if (it == repeat_memory_.end()) {
    const auto limit = static_cast<std::size_t>(std::max(1, params.frontier_suppression_max_attempt_records));
    if (repeat_memory_.size() >= limit) {
      auto oldest = std::min_element(repeat_memory_.begin(), repeat_memory_.end(),
        [](const auto & a, const auto & b) {return a.touched < b.touched;});
      repeat_memory_.erase(oldest);
    }
    repeat_memory_.push_back(RepeatMemory{});
    it = repeat_memory_.end()-1;
    it->point = p;
  }
  it->point = p;  // Compare before/after samples at the same world location.
  it->before = before;
  it->pending = true;
  it->ready = now + static_cast<int64_t>(std::max(0.0, params.post_goal_min_settle)*1e9);
  it->touched = now;
}

FrontierSequence FrontierExplorerCore::filter_repeat_frontiers(const FrontierSequence & frontiers)
{
  if (!params.repeat_guard_enabled) {return frontiers;}
  const auto now = callbacks.now_ns();
  for (auto & r : repeat_memory_) {
    if (!r.pending || now < r.ready) {continue;}
    r.pending = false;
    const double gain = repeat_known_area(r.point)-r.before;
    if (gain >= std::max(0.01, params.repeat_guard_min_gain_m2)) {
      r.strikes = 0;
      r.until = 0;
      callbacks.log_info("REPEAT_GUARD_GAIN: new local map information; cooldown reset");
      continue;
    }
    r.strikes = std::min(r.strikes+1, 20);
    const double base = std::max(1.0, params.repeat_guard_base_cooldown_s);
    const double cap = std::max(base, params.repeat_guard_max_cooldown_s);
    const double seconds = std::min(cap, base * std::pow(2.0, r.strikes-1));
    r.until = now + static_cast<int64_t>(seconds*1e9);
    std::ostringstream message;
    message << "REPEAT_GUARD_COOLDOWN: frontier=(" << r.point.first << ", " << r.point.second
            << "), gain_m2=" << gain << ", no_gain_attempts=" << r.strikes
            << ", cooldown_s=" << seconds;
    callbacks.log_info(message.str());
  }
  FrontierSequence result;
  for (const auto & f : frontiers) {
    const bool blocked = std::any_of(repeat_memory_.begin(), repeat_memory_.end(), [&](const auto & r) {
      return (r.pending || now < r.until) &&
             std::hypot(f.center_point.first-r.point.first, f.center_point.second-r.point.second) <=
             std::max(0.05, params.repeat_guard_match_distance_m);
    });
    if (!blocked) {result.push_back(f);}
  }
  return result;
}
}  // namespace frontier_exploration_ros2
