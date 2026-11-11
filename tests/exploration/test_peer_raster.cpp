#include <gtest/gtest.h>
#include <cmath>
#include "../../src/edgeai_navigation/src/peer_raster.hpp"

TEST(PeerRaster, RotatedCellsDoNotAcquireExtraCornerPadding)
{
  using edgeai_navigation::peerCellIntersects;
  const double c = std::sqrt(.5);
  const auto rotated = [c](double u, double v) {
      return peerCellIntersects(c * (u - v), c * (u + v), c, c,
        -.3125, .1125, -.165, .165, .01, .05);
    };
  EXPECT_TRUE(rotated(-.1, 0.));  // Actual body remains lethal.
  EXPECT_TRUE(rotated(.14, 0.));  // Cell crosses the padded front edge.
  EXPECT_FALSE(rotated(.1525, .205));  // Old half-diagonal box overmarked this corner.
  EXPECT_TRUE(peerCellIntersects(.1465, 0., 1., 0., -.3125, .1125, -.165, .165, .01, .05));
  EXPECT_FALSE(peerCellIntersects(.1485, 0., 1., 0., -.3125, .1125, -.165, .165, .01, .05));
}

TEST(PeerRaster, EveryCellContainingBodySamplesRemainsLethal)
{
  for (int angle = 0; angle < 32; ++angle) {
    const double yaw = angle * 2. * std::acos(-1.) / 32.;
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (int i = 0; i <= 8; ++i) {
      for (int j = 0; j <= 8; ++j) {
        const double u = -.3225 + .445 * i / 8., v = -.175 + .35 * j / 8.;
        const double x = (std::floor((c * u - s * v) / .05) + .5) * .05;
        const double y = (std::floor((s * u + c * v) / .05) + .5) * .05;
        EXPECT_TRUE(edgeai_navigation::peerCellIntersects(x, y, c, s,
          -.3125, .1125, -.165, .165, .01, .05));
      }
    }
  }
}
