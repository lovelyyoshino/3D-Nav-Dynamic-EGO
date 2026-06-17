#include <gtest/gtest.h>

#include "nav3d/planner/bspline/uniform_bspline.h"

TEST(UniformBspline, CubicCurveInterpolatesEndpointClamps)
{
  const std::vector<nav3d::common::Point3D> waypoints{
    {0, 0, 0},
    {1, 0, 0},
    {2, 1, 0},
    {3, 1, 0},
  };

  const auto spline = nav3d::planner::UniformBspline::fitThroughWaypoints(waypoints, 0.25);

  EXPECT_NEAR(spline.evaluate(0.0).x, 0.0, 1e-9);
  EXPECT_NEAR(spline.evaluate(0.0).y, 0.0, 1e-9);
  EXPECT_NEAR(spline.evaluate(spline.duration()).x, 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluate(spline.duration()).y, 1.0, 1e-9);
  EXPECT_GT(spline.duration(), 0.0);
}

TEST(UniformBspline, ProducesFiniteSamplesAlongPath)
{
  const auto spline = nav3d::planner::UniformBspline::fitThroughWaypoints(
    {{0, 0, 0}, {1, 1, 0}, {2, 0, 0}, {3, 0, 0}}, 0.5);

  for (double t = 0.0; t <= spline.duration(); t += 0.1) {
    const auto p = spline.evaluate(t);
    EXPECT_TRUE(std::isfinite(p.x));
    EXPECT_TRUE(std::isfinite(p.y));
    EXPECT_TRUE(std::isfinite(p.z));
  }
}

TEST(UniformBspline, RescaleTimePreservesGeometryAtNormalizedTimes)
{
  const auto spline = nav3d::planner::UniformBspline::fitThroughWaypoints(
    {{0, 0, 0}, {1, 1, 0}, {2, 0, 0}, {3, 0, 0}}, 0.5);

  const auto slower = spline.rescaleTime(2.0);

  EXPECT_DOUBLE_EQ(slower.interval(), 1.0);
  EXPECT_DOUBLE_EQ(slower.duration(), spline.duration() * 2.0);
  for (const double ratio : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    const auto original = spline.evaluate(spline.duration() * ratio);
    const auto scaled = slower.evaluate(slower.duration() * ratio);
    EXPECT_NEAR(scaled.x, original.x, 1e-9);
    EXPECT_NEAR(scaled.y, original.y, 1e-9);
    EXPECT_NEAR(scaled.z, original.z, 1e-9);
  }
}
