#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <set>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace edgeai_navigation
{
struct RecoveryPose {double x, y, yaw;};
using Polygon = std::vector<geometry_msgs::msg::Point>;
struct RecoveryMotion {std::string kind; double amount;};

inline Polygon transformed(const Polygon & body, RecoveryPose pose)
{
  Polygon result;
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  for (auto point : body) {
    const double x = point.x, y = point.y;
    point.x = pose.x + c * x - s * y; point.y = pose.y + s * x + c * y;
    result.push_back(point);
  }
  return result;
}

// Use the same Nav2 footprint rasterizer and lethal threshold as native behaviors.
// The caller owns a costmap snapshot; no subscriptions or extra map copies here.
inline double footprintCost(nav2_costmap_2d::Costmap2D & map,
  const Polygon & body, RecoveryPose pose)
{
  unsigned int mx, my;
  if (body.size() < 3 || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    !std::isfinite(pose.yaw) || !map.worldToMap(pose.x, pose.y, mx, my))
  {return nav2_costmap_2d::LETHAL_OBSTACLE;}
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&map);
  return checker.footprintCostAtPose(pose.x, pose.y, pose.yaw, body);
}

inline bool poseFree(nav2_costmap_2d::Costmap2D & map,
  const Polygon & body, RecoveryPose pose)
{
  const double cost = footprintCost(map, body, pose);
  return cost >= 0.0 && cost < nav2_costmap_2d::LETHAL_OBSTACLE;
}

inline RecoveryPose advanced(RecoveryPose pose, const RecoveryMotion & motion, double fraction)
{
  if (motion.kind == "spin") {pose.yaw += motion.amount * fraction;}
  else {
    const double distance = motion.amount * fraction * (motion.kind == "backup" ? -1 : 1);
    pose.x += distance * std::cos(pose.yaw); pose.y += distance * std::sin(pose.yaw);
  }
  return pose;
}

inline bool motionFree(nav2_costmap_2d::Costmap2D & map, const Polygon & body,
  RecoveryPose pose, const RecoveryMotion & motion)
{
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(motion.amount) /
      (motion.kind == "spin" ? 0.025 : std::min(0.01, map.getResolution() / 2.0)))));
  for (int i = 0; i <= steps; ++i) {
    if (!poseFree(map, body, advanced(pose, motion, static_cast<double>(i) / steps))) {return false;}
  }
  return true;
}

// Rank already-feasible endpoints by native costmap cost (inflation is soft).
// This replaces a full-map obstacle-distance scan for every recovery candidate.
inline double footprintPreference(nav2_costmap_2d::Costmap2D & map,
  const Polygon & body, RecoveryPose pose)
{
  return 1.0 - footprintCost(map, body, pose) / nav2_costmap_2d::LETHAL_OBSTACLE;
}

struct OccupancyOverlap
{
  bool valid = true;
  double area = 0.0;
  std::set<std::pair<int, int>> cells;
};

inline double clippedArea(Polygon polygon, double xmin, double ymin, double xmax, double ymax)
{
  for (int side = 0; side < 4 && !polygon.empty(); ++side) {
    const bool use_x = side < 2; const double bound = side == 0 ? xmin : side == 1 ? xmax : side == 2 ? ymin : ymax;
    auto coordinate = [use_x](const auto & p) {return use_x ? p.x : p.y;};
    auto inside = [&](const auto & p) {return side % 2 == 0 ? coordinate(p) >= bound : coordinate(p) <= bound;};
    Polygon output;
    auto previous = polygon.back();
    for (const auto & point : polygon) {
      if (inside(point) != inside(previous)) {
        const double fraction = (bound - coordinate(previous)) / (coordinate(point) - coordinate(previous));
        geometry_msgs::msg::Point hit;
        hit.x = previous.x + fraction * (point.x - previous.x);
        hit.y = previous.y + fraction * (point.y - previous.y); output.push_back(hit);
      }
      if (inside(point)) {output.push_back(point);}
      previous = point;
    }
    polygon = std::move(output);
  }
  double area = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i]; const auto & b = polygon[(i + 1) % polygon.size()];
    area += a.x * b.y - b.x * a.y;
  }
  return std::abs(area) / 2.0;
}

inline OccupancyOverlap occupiedOverlap(const nav2_costmap_2d::Costmap2D & map,
  const Polygon & body, RecoveryPose pose)
{
  OccupancyOverlap result;
  if (body.size() < 3) {result.valid = false; return result;}
  const auto polygon = transformed(body, pose);
  unsigned int xmin = map.getSizeInCellsX(), xmax = 0, ymin = map.getSizeInCellsY(), ymax = 0;
  for (const auto & p : polygon) {
    unsigned int x, y;
    if (!map.worldToMap(p.x, p.y, x, y)) {result.valid = false; return result;}
    xmin = std::min(xmin, x); xmax = std::max(xmax, x); ymin = std::min(ymin, y); ymax = std::max(ymax, y);
  }
  const double half = map.getResolution() / 2.0;
  for (unsigned y = ymin; y <= ymax; ++y) {
    for (unsigned x = xmin; x <= xmax; ++x) {
      const auto cost = map.getCost(x, y);
      if (cost < 254) {continue;}
      double wx, wy; map.mapToWorld(x, y, wx, wy);
      const double area = clippedArea(polygon, wx - half, wy - half, wx + half, wy + half);
      if (area <= 1e-10) {continue;}
      if (cost == 255) {result.valid = false; return result;}
      result.area += area;
      result.cells.insert({static_cast<int>(std::llround(wx / map.getResolution())),
        static_cast<int>(std::llround(wy / map.getResolution()))});
    }
  }
  return result;
}

// Exit existing known overlap, including actual body overlap. Both footprints
// must lose overlap monotonically without touching any new occupied cells.
inline bool occupiedEscapeFree(nav2_costmap_2d::Costmap2D & map,
  const Polygon & actual, const Polygon & padded, RecoveryPose pose,
  const RecoveryMotion & motion, const OccupancyOverlap & allowed,
  const OccupancyOverlap * original_actual = nullptr)
{
  if (motion.kind == "spin" || actual.size() < 3 || !allowed.valid) {return false;}
  const auto actual_allowed = original_actual ? *original_actual : occupiedOverlap(map, actual, pose);
  if (!actual_allowed.valid) {return false;}
  double previous = occupiedOverlap(map, padded, pose).area;
  double previous_actual = occupiedOverlap(map, actual, pose).area;
  const int steps = std::max(1, static_cast<int>(std::ceil(motion.amount / .005)));
  for (int i = 0; i <= steps; ++i) {
    const auto sample = advanced(pose, motion, static_cast<double>(i) / steps);
    const auto physical = occupiedOverlap(map, actual, sample);
    if (!physical.valid || physical.area > previous_actual + 1e-8 ||
      !std::includes(actual_allowed.cells.begin(), actual_allowed.cells.end(), physical.cells.begin(), physical.cells.end())) {return false;}
    previous_actual = physical.area;
    const auto overlap = occupiedOverlap(map, padded, sample);
    if (!overlap.valid || overlap.area > previous + 1e-8 ||
      !std::includes(allowed.cells.begin(), allowed.cells.end(), overlap.cells.begin(), overlap.cells.end())) {return false;}
    previous = overlap.area;
  }
  return previous < allowed.area - 1e-8 ||
    (previous < 1e-8 && poseFree(map, padded, advanced(pose, motion, 1.0)));
}

inline bool rppConnectionFree(nav2_costmap_2d::Costmap2D & map,
  const Polygon & body, RecoveryPose pose, double target_x, double target_y, bool reversing)
{
  const double dx = target_x - pose.x, dy = target_y - pose.y;
  const double x = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
  const double y = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  const double distance = std::hypot(x, y);
  if (distance < .04) {return poseFree(map, body, pose);}
  if (x < 0.0 && !reversing) {return false;}
  const double curvature = 2.0 * y / (distance * distance);
  const double length = std::min(.12, distance * .75) * (x < 0.0 ? -1.0 : 1.0);
  for (int i = 0; i <= 24; ++i) {
    const double step = length * i / 24.0;
    auto sample = pose;
    if (std::abs(curvature) < 1e-6) {sample.x += step * std::cos(pose.yaw); sample.y += step * std::sin(pose.yaw);}
    else {
      sample.yaw += curvature * step;
      sample.x += (std::sin(sample.yaw) - std::sin(pose.yaw)) / curvature;
      sample.y -= (std::cos(sample.yaw) - std::cos(pose.yaw)) / curvature;
    }
    if (!poseFree(map, body, sample)) {return false;}
  }
  return true;
}
}  // namespace edgeai_navigation
