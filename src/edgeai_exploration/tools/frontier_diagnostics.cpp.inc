namespace frontier_exploration_ros2
{
void FrontierExplorerCore::diagnose_frontier_exhaustion(const geometry_msgs::msg::Pose & pose)
{
  if (!map || !decision_map || !costmap) {return;}
  auto empty_cost = map->map();
  std::fill(empty_cost.data.begin(), empty_cost.data.end(), int8_t{0});
  FrontierSearchOptions loose = frontier_search_options(); loose.min_frontier_size_cells = 1;
  const auto raw = get_frontier(pose, *map, OccupancyGrid2d(empty_cost), std::nullopt, 0., false, loose);
  const auto decision = get_frontier(pose, *decision_map, OccupancyGrid2d(empty_cost), std::nullopt, 0., false, loose);
  const auto filtered = get_frontier(pose, *decision_map, *costmap, local_costmap, 0., false, loose);
  const auto describe = [](const auto & result) {
      std::ostringstream s; s << result.frontiers.size() << " clusters [";
      for (const auto & f : result.frontiers) {s << f.size << ',';} s << ']'; return s.str();
    };
  callbacks.log_info("FRONTIER_EMPTY_DIAGNOSTICS: raw="+describe(raw)+
    "; decision="+describe(decision)+"; cost_filtered="+describe(filtered)+
    "; min_cluster="+std::to_string(params.min_frontier_size_cells));
  const char * base = std::getenv("EDGEAI_FRONTIER_DIAGNOSTICS_DIR");
  if (!base || !*base) {return;}
  try {
    const auto directory = std::filesystem::path(base)/std::to_string(callbacks.now_ns());
    std::filesystem::create_directories(directory);
    const auto dump = [&](const std::string & name, const OccupancyGrid2d & grid) {
        std::ofstream f(directory/(name+".txt")); const auto & g = grid.map();
        f << std::setprecision(17) << g.info.width << ' ' << g.info.height << ' ' << g.info.resolution << ' '
          << g.info.origin.position.x << ' ' << g.info.origin.position.y << '\n';
        for (auto value : g.data) {f << int(value) << ' ';}
      };
    dump("map", *map); dump("decision", *decision_map); dump("global_costmap", *costmap);
    if (local_costmap) {dump("local_costmap", *local_costmap);}
    std::ofstream(directory/"pose.txt") << std::setprecision(17) << pose.position.x << ' ' << pose.position.y;
  } catch (const std::exception & error) {
    callbacks.log_warn(std::string("Frontier diagnostic snapshot failed: ")+error.what());
  }
}
}  // namespace frontier_exploration_ros2
