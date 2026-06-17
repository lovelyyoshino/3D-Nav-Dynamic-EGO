#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "nav3d/collision/trajectory_feasibility_checker.h"
#include "nav3d/planner/bspline/uniform_bspline.h"

TEST(TrajectoryFeasibilityChecker, ReportsVelocityViolationAndRequiredTimeScale)
{
  const auto spline = nav3d::planner::UniformBspline::fitThroughWaypoints(
    {{0.0, 0.0, 0.0}, {4.0, 0.0, 0.0}}, 0.2);
  nav3d::collision::TrajectoryFeasibilityChecker checker(0.02);
  nav3d::collision::DynamicLimits limits;
  limits.max_velocity = 2.0;
  limits.max_acceleration = std::numeric_limits<double>::infinity();

  const auto result = checker.check(spline, limits);

  EXPECT_FALSE(result.feasible);
  EXPECT_TRUE(result.velocity_violation);
  EXPECT_FALSE(result.acceleration_violation);
  EXPECT_TRUE(result.first_violation_time.has_value());
  EXPECT_GT(result.max_velocity_observed, limits.max_velocity);
  EXPECT_GT(result.required_time_scale, 1.0);
}

TEST(TrajectoryFeasibilityChecker, TimeScaledSplineCanSatisfyVelocityLimit)
{
  const auto spline = nav3d::planner::UniformBspline::fitThroughWaypoints(
    {{0.0, 0.0, 0.0}, {4.0, 0.0, 0.0}}, 0.2);
  nav3d::collision::TrajectoryFeasibilityChecker checker(0.02);
  nav3d::collision::DynamicLimits limits;
  limits.max_velocity = 2.0;
  limits.max_acceleration = std::numeric_limits<double>::infinity();

  const auto initial = checker.check(spline, limits);
  ASSERT_FALSE(initial.feasible);

  const auto slower = spline.rescaleTime(initial.required_time_scale * 1.05);
  const auto after_scaling = checker.check(slower, limits);

  EXPECT_TRUE(after_scaling.feasible);
  EXPECT_LE(after_scaling.max_velocity_observed, limits.max_velocity + 1e-6);
}

TEST(TrajectoryFeasibilityChecker, AnalyticBoundsUseDerivativeControlPolygon)
{
  const nav3d::planner::UniformBspline spline(
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}},
    3,
    0.5);
  const nav3d::collision::TrajectoryFeasibilityChecker checker(0.02);

  const auto bounds = checker.computeAnalyticBounds(spline);

  EXPECT_NEAR(bounds.max_velocity_bound, 6.0, 1e-9);
  EXPECT_NEAR(bounds.max_acceleration_bound, 0.0, 1e-9);
}

TEST(TrajectoryFeasibilityChecker, AnalyticBoundsScaleWithSplineTime)
{
  const nav3d::planner::UniformBspline spline(
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 2.0, 0.0}, {2.0, 2.0, 0.0}},
    3,
    0.5);
  const nav3d::collision::TrajectoryFeasibilityChecker checker(0.02);

  const auto initial = checker.computeAnalyticBounds(spline);
  const auto slower = checker.computeAnalyticBounds(spline.rescaleTime(2.0));

  EXPECT_NEAR(initial.max_velocity_bound, 12.0, 1e-9);
  EXPECT_NEAR(initial.max_acceleration_bound, 24.0 * std::sqrt(5.0), 1e-9);
  EXPECT_NEAR(slower.max_velocity_bound, initial.max_velocity_bound / 2.0, 1e-9);
  EXPECT_NEAR(slower.max_acceleration_bound, initial.max_acceleration_bound / 4.0, 1e-9);
}

TEST(TrajectoryFeasibilityChecker, AnalyticCheckCatchesUnsampledControlPolygonViolation)
{
  const nav3d::planner::UniformBspline spline(
    {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {-10.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
    3,
    1.0);
  nav3d::collision::DynamicLimits limits;
  limits.max_velocity = 10.0;
  limits.max_acceleration = std::numeric_limits<double>::infinity();

  const nav3d::collision::TrajectoryFeasibilityChecker coarse_checker(5.0);
  const auto sampled = coarse_checker.check(spline, limits);
  ASSERT_TRUE(sampled.feasible);

  const auto analytic = coarse_checker.checkAnalyticBounds(spline, limits);

  EXPECT_FALSE(analytic.feasible);
  EXPECT_TRUE(analytic.velocity_violation);
  EXPECT_FALSE(analytic.acceleration_violation);
  EXPECT_FALSE(analytic.first_violation_time.has_value());
  EXPECT_NEAR(analytic.max_velocity_observed, 60.0, 1e-9);
  EXPECT_NEAR(analytic.required_time_scale, 6.0, 1e-9);
}

TEST(TrajectoryFeasibilityChecker, ExactAnalyticExtremaCanAcceptTrajectoryRejectedByBounds)
{
  const nav3d::planner::UniformBspline spline(
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 2.0, 0.0}, {2.0, 2.0, 0.0}},
    3,
    0.5);
  const nav3d::collision::TrajectoryFeasibilityChecker checker(0.05);

  const auto bounds = checker.computeAnalyticBounds(spline);
  const auto extrema = checker.computeAnalyticExtrema(spline);

  EXPECT_LT(extrema.max_velocity, bounds.max_velocity_bound);
  EXPECT_NEAR(extrema.max_acceleration, bounds.max_acceleration_bound, 1e-6);
  EXPECT_NEAR(extrema.max_velocity, 3.0 * std::sqrt(5.0), 1e-6);
  EXPECT_NEAR(extrema.max_acceleration, 24.0 * std::sqrt(5.0), 1e-6);

  nav3d::collision::DynamicLimits limits;
  limits.max_velocity = 7.0;
  limits.max_acceleration = 60.0;

  const auto conservative = checker.checkAnalyticBounds(spline, limits);
  const auto exact = checker.checkAnalyticExact(spline, limits);

  EXPECT_FALSE(conservative.feasible);
  EXPECT_TRUE(exact.feasible);
  EXPECT_FALSE(exact.velocity_violation);
  EXPECT_FALSE(exact.acceleration_violation);
  EXPECT_NEAR(exact.max_velocity_observed, extrema.max_velocity, 1e-6);
  EXPECT_NEAR(exact.max_acceleration_observed, extrema.max_acceleration, 1e-6);
}
