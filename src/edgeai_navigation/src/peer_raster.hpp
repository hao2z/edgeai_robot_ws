#pragma once
#include <cmath>

namespace edgeai_navigation
{
// Exact overlap of an axis-aligned costmap cell and an oriented padded body.
// Check all four separating axes; a diagonal-radius box marks extra corners.
inline bool peerCellIntersects(
  double dx, double dy, double c, double s, double xmin, double xmax,
  double ymin, double ymax, double padding, double resolution)
{
  const double mx = (xmin + xmax) / 2., my = (ymin + ymax) / 2.;
  const double hx = (xmax - xmin) / 2. + padding;
  const double hy = (ymax - ymin) / 2. + padding;
  const double half = resolution / 2.;
  const double ac = std::abs(c), as = std::abs(s);
  const double x = dx - (c * mx - s * my), y = dy - (s * mx + c * my);
  return std::abs(x) <= half + hx * ac + hy * as &&
         std::abs(y) <= half + hx * as + hy * ac &&
         std::abs(c * x + s * y) <= hx + half * (ac + as) &&
         std::abs(-s * x + c * y) <= hy + half * (ac + as);
}
}  // namespace edgeai_navigation
