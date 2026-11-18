#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace frontier_exploration_ros2
{

// Cheap endpoint check on the raw SLAM map. The global planner still owns
// route checks and live obstacles. Unknown cells may border a valid frontier.
class FrontierEndpointClearance
{
public:
  void configure(const std::vector<double> & footprint, const std::vector<double> & headings)
  {
    if (footprint.empty()) {return;}
    if (footprint.size() < 6 || footprint.size() % 2 != 0 || headings.empty() ||
      !std::all_of(footprint.begin(), footprint.end(), [](double v) {return std::isfinite(v);}) ||
      !std::all_of(headings.begin(), headings.end(), [](double v) {return std::isfinite(v);}))
    {
      throw std::invalid_argument("Invalid frontier footprint or heading list");
    }
    for (std::size_t i = 0; i < footprint.size(); i += 2) {
      radius_ = std::max(radius_, std::hypot(footprint[i], footprint[i + 1]));
    }
    for (const double yaw : headings) {
      const double c = std::cos(yaw), s = std::sin(yaw);
      std::vector<std::pair<double, double>> points;
      for (std::size_t i = 0; i < footprint.size(); i += 2) {
        points.emplace_back(c * footprint[i] - s * footprint[i + 1],
          s * footprint[i] + c * footprint[i + 1]);
      }
      Shape shape{project(points, 1., 0.), project(points, 0., 1.), {}};
      for (std::size_t i = 0; i < points.size(); ++i) {
        const auto & a = points[i];
        const auto & b = points[(i + 1) % points.size()];
        const double dx = b.first - a.first, dy = b.second - a.second;
        if (dx != 0. || dy != 0.) {shape.axes.push_back(project(points, -dy, dx));}
      }
      shapes_.push_back(std::move(shape));
    }
  }

  template<typename Grid>
  bool allows(const Grid & grid, double x, double y, int occupied_threshold) const
  {
    if (shapes_.empty()) {return true;}
    const auto & info = grid.map().info;
    const double resolution = info.resolution, half_cell = resolution / 2.;
    if (!(resolution > 0.)) {return false;}
    const double left = info.origin.position.x - x, bottom = info.origin.position.y - y;
    const double right = left + grid.getSizeX() * resolution;
    const double top = bottom + grid.getSizeY() * resolution;
    const auto lower = [&](double origin) {
        return std::max(0, static_cast<int>(std::floor((-radius_ - origin) / resolution)));
      };
    const auto upper = [&](double origin, int size) {
        return std::min(size - 1, static_cast<int>(std::floor((radius_ - origin) / resolution)));
      };
    const int min_x = lower(left), max_x = upper(left, grid.getSizeX());
    const int min_y = lower(bottom), max_y = upper(bottom, grid.getSizeY());
    std::vector<std::pair<double, double>> obstacles;
    for (int iy = min_y; iy <= max_y; ++iy) {
      for (int ix = min_x; ix <= max_x; ++ix) {
        if (grid.getCost(ix, iy) >= occupied_threshold) {
          obstacles.emplace_back(left + (ix + .5) * resolution, bottom + (iy + .5) * resolution);
        }
      }
    }
    for (const auto & shape : shapes_) {
      if (shape.x.low < left || shape.x.high > right || shape.y.low < bottom || shape.y.high > top) {
        continue;
      }
      const bool blocked = std::any_of(obstacles.begin(), obstacles.end(), [&](const auto & cell) {
          if (disjoint(shape.x, cell, half_cell) || disjoint(shape.y, cell, half_cell)) {return false;}
          return std::none_of(shape.axes.begin(), shape.axes.end(), [&](const Axis & axis) {
              return disjoint(axis, cell, half_cell);
            });
        });
      if (!blocked) {return true;}
    }
    return false;
  }

private:
  struct Axis {double x, y, low, high, cell_projection;};
  struct Shape {Axis x, y; std::vector<Axis> axes;};

  static Axis project(const std::vector<std::pair<double, double>> & points, double x, double y)
  {
    Axis axis{x, y, std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(), std::abs(x) + std::abs(y)};
    for (const auto & point : points) {
      const double value = point.first * x + point.second * y;
      axis.low = std::min(axis.low, value);
      axis.high = std::max(axis.high, value);
    }
    return axis;
  }

  static bool disjoint(const Axis & axis, const std::pair<double, double> & cell, double half_cell)
  {
    const double center = axis.x * cell.first + axis.y * cell.second;
    const double extent = half_cell * axis.cell_projection;
    return center + extent < axis.low || center - extent > axis.high;
  }

  double radius_{0.};
  std::vector<Shape> shapes_;
};
}  // namespace frontier_exploration_ros2
