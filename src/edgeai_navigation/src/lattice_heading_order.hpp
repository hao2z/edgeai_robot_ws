#pragma once
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace edgeai_navigation
{
// A priority hint, never an exclusion: a blocked exact pose may still have a
// valid nearby endpoint under the native planner's goal tolerance.
inline bool exactGoalFootprintClear(nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint, double x, double y, double yaw,
  bool allow_unknown)
{
  unsigned int mx, my;
  if (!map.worldToMap(x, y, mx, my)) {return false;}
  auto clear = [allow_unknown](double cost) {
      return (cost >= 0.0 && cost < nav2_costmap_2d::LETHAL_OBSTACLE) ||
             (allow_unknown && cost == nav2_costmap_2d::NO_INFORMATION);
    };
  if (!clear(map.getCost(mx, my))) {return false;}
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&map);
  return clear(checker.footprintCostAtPose(x, y, yaw, footprint));
}
}  // namespace edgeai_navigation
