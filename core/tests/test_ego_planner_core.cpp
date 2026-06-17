#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "nav3d/map/i_map.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/ego_planner_core.h"
#include "nav3d/planner/i_path_searcher.h"

namespace {

class FixedPathSearcher final : public nav3d::planner::IPathSearcher {
public:
  explicit FixedPathSearcher(nav3d::common::Path3D path) : path_(std::move(path)) {}

  nav3d::planner::SearchResult search(
    const nav3d::map::IMap&,
    const nav3d::common::Point3D&,
    const nav3d::common::Point3D&,
    const nav3d::planner::SearchOptions&) const override
  {
    return {nav3d::planner::SearchStatus::Success, path_, 1};
  }

private:
  nav3d::common::Path3D path_;
};

class GoalLineSearcher final : public nav3d::planner::IPathSearcher {
public:
  nav3d::planner::SearchResult search(
    const nav3d::map::IMap&,
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal,
    const nav3d::planner::SearchOptions&) const override
  {
    return {
      nav3d::planner::SearchStatus::Success,
      {start, start + (goal - start) * 0.5, goal},
      1,
    };
  }
};

class RejectGoalXSearcher final : public nav3d::planner::IPathSearcher {
public:
  explicit RejectGoalXSearcher(double rejected_x) : rejected_x_(rejected_x) {}

  nav3d::planner::SearchResult search(
    const nav3d::map::IMap&,
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal,
    const nav3d::planner::SearchOptions&) const override
  {
    if (std::abs(goal.x - rejected_x_) < 1e-9) {
      return {nav3d::planner::SearchStatus::NoPath, {}, 1};
    }
    return {
      nav3d::planner::SearchStatus::Success,
      {start, start + (goal - start) * 0.5, goal},
      1,
    };
  }

private:
  double rejected_x_ = 0.0;
};

class RejectShortGoalSearcher final : public nav3d::planner::IPathSearcher {
public:
  nav3d::planner::SearchResult search(
    const nav3d::map::IMap&,
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal,
    const nav3d::planner::SearchOptions&) const override
  {
    if (goal.x < 3.0) {
      return {nav3d::planner::SearchStatus::NoPath, {}, 1};
    }
    return {
      nav3d::planner::SearchStatus::Success,
      {start, start + (goal - start) * 0.5, goal},
      1,
    };
  }
};

class ThinWallMap final : public nav3d::map::IMap {
public:
  bool isOccupied(const nav3d::common::Point3D& p) const override
  {
    return p.x > 1.25 && p.x < 1.75 && std::abs(p.y) < 0.05 && std::abs(p.z) < 0.05;
  }

  bool isFree(const nav3d::common::Point3D& p) const override
  {
    return isInBounds(p) && !isOccupied(p);
  }

  double getDistance(const nav3d::common::Point3D&) const override { return 0.0; }

  bool isInBounds(const nav3d::common::Point3D& p) const override
  {
    return p.x >= 0.0 && p.x <= 3.0 && p.y >= 0.0 && p.y <= 0.0 && p.z >= 0.0 && p.z <= 0.0;
  }

  double getResolution() const override { return 1.0; }

  nav3d::common::BoundingBox getBounds() const override
  {
    return {{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, true};
  }
};

class NearStartWallMap final : public nav3d::map::IMap {
public:
  bool isOccupied(const nav3d::common::Point3D& p) const override
  {
    return p.x > 0.05 && p.x < 0.15 && std::abs(p.y) < 0.05 && std::abs(p.z) < 0.05;
  }

  bool isFree(const nav3d::common::Point3D& p) const override
  {
    return isInBounds(p) && !isOccupied(p);
  }

  double getDistance(const nav3d::common::Point3D&) const override { return 0.0; }

  bool isInBounds(const nav3d::common::Point3D& p) const override
  {
    return p.x >= 0.0 && p.x <= 3.0 && p.y >= 0.0 && p.y <= 0.0 && p.z >= 0.0 && p.z <= 0.0;
  }

  double getResolution() const override { return 1.0; }

  nav3d::common::BoundingBox getBounds() const override
  {
    return {{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, true};
  }
};

class RoundedCornerShortcutMap final : public nav3d::map::IMap {
public:
  bool isOccupied(const nav3d::common::Point3D& p) const override
  {
    const double dx = p.x - 0.8;
    const double dy = p.y - 0.2;
    return dx * dx + dy * dy < 0.06 * 0.06 && std::abs(p.z) < 0.2;
  }

  bool isFree(const nav3d::common::Point3D& p) const override
  {
    return isInBounds(p) && !isOccupied(p);
  }

  double getDistance(const nav3d::common::Point3D&) const override { return 0.0; }

  bool isInBounds(const nav3d::common::Point3D& p) const override
  {
    return p.x >= -0.1 && p.x <= 2.1 &&
           p.y >= -0.1 && p.y <= 1.1 &&
           p.z >= -0.1 && p.z <= 0.1;
  }

  double getResolution() const override { return 0.1; }

  nav3d::common::BoundingBox getBounds() const override
  {
    return {{-0.1, -0.1, -0.1}, {2.1, 1.1, 0.1}, true};
  }
};

nav3d::planner::EgoPlannerCoreConfig makeFastTestConfig()
{
  nav3d::planner::EgoPlannerCoreConfig config;
  config.search.mode = nav3d::planner::PlanningMode::Mode2D;
  config.search.allow_diagonal = false;
  config.optimizer.max_iterations = 20;
  config.optimizer.rebound_search = config.search;
  config.trajectory_sample_step_seconds = 0.01;
  config.emergency_stop_horizon_seconds = 0.0;
  return config;
}

}  // namespace

TEST(EgoPlannerCore, PlansAFreeTrajectoryFromSearchOptimizationAndPostCheck)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {4.5, 0.5, 0.5});

  nav3d::planner::EgoPlannerCore planner(makeFastTestConfig());

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {4.0, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.status, nav3d::planner::EgoPlanStatus::Success);
  ASSERT_EQ(result.search.status, nav3d::planner::SearchStatus::Success);
  EXPECT_FALSE(result.collision.in_collision);
  EXPECT_EQ(result.fallback.action, nav3d::planner::FallbackAction::WaitForMapOrNewGoal);
  EXPECT_NEAR(result.trajectory.evaluate(0.0).x, 0.5, 1e-6);
  EXPECT_NEAR(result.trajectory.evaluate(result.trajectory.duration()).x, 4.5, 1e-6);
}

TEST(EgoPlannerCore, ReportsEffectivePlannedGoalFromSearchPathIn2DMode)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 1.0}, {3.5, 3.5, 1.5});

  nav3d::planner::EgoPlannerCore planner(makeFastTestConfig());

  const auto result = planner.plan(map, {0.0, 0.0, 1.0}, {3.0, 3.0, 3.0});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.planned_goal, nav3d::common::Point3D(3.5, 3.5, 1.5));
}

TEST(EgoPlannerCore, PlansToNearestFreePoseWhenRequestedGoalIsOccupied)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, -2.0, 0.0}, {4.5, 2.5, 0.5});
  map.insertOccupied({2.0, 0.0, 0.0});

  nav3d::planner::EgoPlannerCore planner(makeFastTestConfig());

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.requested_goal, nav3d::common::Point3D(2.0, 0.0, 0.0));
  EXPECT_FALSE(map.isOccupied(result.planned_goal));
  EXPECT_LE(nav3d::common::distance(result.planned_goal, result.requested_goal), 1.0);
  EXPECT_NEAR(result.trajectory.evaluate(result.trajectory.duration()).x, result.planned_goal.x, 1e-6);
}

TEST(EgoPlannerCore, PlansFromNearestFreePoseWhenRequestedStartIsOccupied)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, -2.0, 0.0}, {4.5, 2.5, 0.5});
  map.insertOccupied({0.0, 0.0, 0.0});

  nav3d::planner::EgoPlannerCore planner(makeFastTestConfig());

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.search.path.empty());
  EXPECT_FALSE(map.isOccupied(result.search.path.front()));
  EXPECT_LE(nav3d::common::distance(result.search.path.front(), {0.0, 0.0, 0.0}), 1.0);
  EXPECT_EQ(result.planned_goal, nav3d::common::Point3D(3.5, 0.5, 0.5));
}

TEST(EgoPlannerCore, UsesConfiguredJpsSearcher)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {8.5, 0.5, 0.5});

  auto config = makeFastTestConfig();
  config.search.algorithm = nav3d::planner::SearchAlgorithm::Jps;
  config.search.mode = nav3d::planner::PlanningMode::Mode3D;
  config.search.allow_diagonal = false;
  nav3d::planner::EgoPlannerCore planner(config);

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {8.0, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.search.status, nav3d::planner::SearchStatus::Success);
  EXPECT_LT(result.search.iterations, 8);
}

TEST(EgoPlannerCore, TurnsOptimizerFailureIntoRetryThenShorterGoalDecisions)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
  map.insertOccupied({2.0, 0.0, 0.0});

  auto config = makeFastTestConfig();
  config.failure_cascade.max_optimization_retries = 1;
  config.failure_cascade.shorter_goal_ratios = {0.5};
  auto fixed_searcher = std::make_shared<FixedPathSearcher>(nav3d::common::Path3D{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {2.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {4.0, 0.0, 0.0},
  });
  nav3d::planner::EgoPlannerCore planner(config, fixed_searcher);

  const auto retry = planner.plan(map, {0.0, 0.0, 0.0}, {4.0, 0.0, 0.0});

  ASSERT_FALSE(retry.success);
  EXPECT_EQ(retry.status, nav3d::planner::EgoPlanStatus::OptimizationFailed);
  EXPECT_FALSE(retry.optimization.success);
  EXPECT_EQ(retry.fallback.action, nav3d::planner::FallbackAction::RetryOptimization);
  EXPECT_DOUBLE_EQ(retry.fallback.collision_weight_scale, 2.0);

  const auto shortened = planner.plan(map, {0.0, 0.0, 0.0}, {4.0, 0.0, 0.0});

  ASSERT_FALSE(shortened.success);
  EXPECT_EQ(shortened.status, nav3d::planner::EgoPlanStatus::OptimizationFailed);
  EXPECT_EQ(shortened.fallback.action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  ASSERT_TRUE(shortened.fallback.next_goal.has_value());
  EXPECT_DOUBLE_EQ(shortened.fallback.next_goal->x, 2.0);
}

TEST(EgoPlannerCore, TurnsSecondaryTrajectoryCollisionIntoFallbackDecision)
{
  ThinWallMap map;
  auto config = makeFastTestConfig();
  config.failure_cascade.max_optimization_retries = 0;
  config.failure_cascade.shorter_goal_ratios = {0.5};
  auto fixed_searcher = std::make_shared<FixedPathSearcher>(nav3d::common::Path3D{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {2.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
  });
  nav3d::planner::EgoPlannerCore planner(config, fixed_searcher);

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.status, nav3d::planner::EgoPlanStatus::TrajectoryCollision);
  ASSERT_TRUE(result.collision.in_collision);
  EXPECT_EQ(result.fallback.action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  ASSERT_TRUE(result.fallback.next_goal.has_value());
  EXPECT_DOUBLE_EQ(result.fallback.next_goal->x, 1.5);
}

TEST(EgoPlannerCore, FallsBackToPathFollowingTrajectoryWhenSmoothedCurveCollides)
{
  RoundedCornerShortcutMap map;
  auto config = makeFastTestConfig();
  config.optimizer.max_iterations = 1;
  config.optimizer.initial_step = 0.0;
  auto fixed_searcher = std::make_shared<FixedPathSearcher>(nav3d::common::Path3D{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {1.0, 1.0, 0.0},
    {2.0, 1.0, 0.0},
  });
  nav3d::planner::EgoPlannerCore planner(config, fixed_searcher);

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {2.0, 1.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.status, nav3d::planner::EgoPlanStatus::Success);
  EXPECT_FALSE(result.collision.in_collision);
  EXPECT_EQ(result.trajectory.degree(), 1);
}

TEST(EgoPlannerCore, PlanWithFallbacksRetriesThenPlansToShorterGoal)
{
  ThinWallMap map;
  auto config = makeFastTestConfig();
  config.failure_cascade.max_optimization_retries = 1;
  config.failure_cascade.shorter_goal_ratios = {0.25};
  auto line_searcher = std::make_shared<GoalLineSearcher>();
  nav3d::planner::EgoPlannerCore planner(config, line_searcher);

  const auto result = planner.planWithFallbacks(map, {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.status, nav3d::planner::EgoPlanStatus::Success);
  EXPECT_EQ(result.attempts, 3);
  ASSERT_EQ(result.fallback_history.size(), 2u);
  EXPECT_EQ(result.fallback_history.at(0).action, nav3d::planner::FallbackAction::RetryOptimization);
  EXPECT_EQ(result.fallback_history.at(1).action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  EXPECT_DOUBLE_EQ(result.planned_goal.x, 0.75);
}

TEST(EgoPlannerCore, RescalesOptimizedTrajectoryToSatisfyDynamicLimits)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {8.0, 0.0, 0.0});
  auto config = makeFastTestConfig();
  config.enable_dynamic_feasibility_check = true;
  config.dynamic_limits.max_velocity = 2.0;
  config.dynamic_limits.max_acceleration = std::numeric_limits<double>::infinity();
  config.feasibility_sample_step_seconds = 0.02;
  config.max_dynamic_time_scale = 20.0;
  config.optimizer.interval = 0.2;
  auto line_searcher = std::make_shared<GoalLineSearcher>();
  nav3d::planner::EgoPlannerCore planner(config, line_searcher);

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {6.0, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.time_scaled);
  EXPECT_GT(result.time_scale, 1.0);
  EXPECT_TRUE(result.feasibility.feasible);
  EXPECT_GT(result.trajectory.duration(), result.optimization.spline.duration());
}

TEST(EgoPlannerCore, AllocatesInitialSplineTimeFromPathLengthAndVelocity)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0});
  auto config = makeFastTestConfig();
  config.enable_initial_time_allocation = true;
  config.dynamic_limits.max_velocity = 1.0;
  config.optimizer.interval = 0.2;
  auto fixed_searcher = std::make_shared<FixedPathSearcher>(nav3d::common::Path3D{
    {0.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {6.0, 0.0, 0.0},
  });
  nav3d::planner::EgoPlannerCore planner(config, fixed_searcher);

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {6.0, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.time_scaled);
  EXPECT_NEAR(result.optimization.warm_start.interval(), 1.5, 1e-9);
  EXPECT_NEAR(result.optimization.warm_start.duration(), 6.0, 1e-9);
  EXPECT_NEAR(result.trajectory.interval(), 1.5, 1e-9);
  EXPECT_NEAR(result.trajectory.duration(), 6.0, 1e-9);
}

TEST(EgoPlannerCore, ReportsDynamicFeasibilityViolationWhenRequiredTimeScaleExceedsCap)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {8.0, 0.0, 0.0});
  auto config = makeFastTestConfig();
  config.enable_dynamic_feasibility_check = true;
  config.dynamic_limits.max_velocity = 0.1;
  config.dynamic_limits.max_acceleration = std::numeric_limits<double>::infinity();
  config.feasibility_sample_step_seconds = 0.02;
  config.max_dynamic_time_scale = 2.0;
  config.optimizer.interval = 0.2;
  auto line_searcher = std::make_shared<GoalLineSearcher>();
  nav3d::planner::EgoPlannerCore planner(config, line_searcher);

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {6.0, 0.0, 0.0});

  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.status, nav3d::planner::EgoPlanStatus::DynamicFeasibilityViolation);
  EXPECT_FALSE(result.time_scaled);
  EXPECT_FALSE(result.feasibility.feasible);
  EXPECT_GT(result.feasibility.required_time_scale, config.max_dynamic_time_scale);
}

TEST(EgoPlannerCore, AnalyticDynamicFeasibilityModeUsesDerivativeBounds)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0});
  auto config = makeFastTestConfig();
  config.enable_dynamic_feasibility_check = true;
  config.dynamic_feasibility_mode = nav3d::planner::DynamicFeasibilityMode::AnalyticBounds;
  config.dynamic_limits.max_velocity = 10.0;
  config.dynamic_limits.max_acceleration = std::numeric_limits<double>::infinity();
  config.feasibility_sample_step_seconds = 5.0;
  config.max_dynamic_time_scale = 2.0;
  config.optimizer.interval = 1.0;
  config.optimizer.max_iterations = 1;
  config.optimizer.initial_step = 0.0;
  config.optimizer.smoothness_weight = 0.0;
  config.optimizer.terminal_weight = 0.0;
  config.optimizer.fitness_weight = 0.0;
  auto fixed_searcher = std::make_shared<FixedPathSearcher>(nav3d::common::Path3D{
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {-10.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
  });
  nav3d::planner::EgoPlannerCore planner(config, fixed_searcher);

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.status, nav3d::planner::EgoPlanStatus::DynamicFeasibilityViolation);
  EXPECT_FALSE(result.time_scaled);
  EXPECT_FALSE(result.feasibility.feasible);
  EXPECT_TRUE(result.feasibility.velocity_violation);
  EXPECT_NEAR(result.feasibility.max_velocity_observed, 60.0, 1e-9);
  EXPECT_NEAR(result.feasibility.required_time_scale, 6.0, 1e-9);
}

TEST(EgoPlannerCore, ExactAnalyticDynamicFeasibilityModeUsesContinuousExtrema)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {2.0, 2.0, 0.0});
  auto config = makeFastTestConfig();
  config.enable_dynamic_feasibility_check = true;
  config.dynamic_feasibility_mode = nav3d::planner::DynamicFeasibilityMode::AnalyticExact;
  config.dynamic_limits.max_velocity = 7.0;
  config.dynamic_limits.max_acceleration = 60.0;
  config.feasibility_sample_step_seconds = 5.0;
  config.max_dynamic_time_scale = 1.0;
  config.optimizer.interval = 0.5;
  config.optimizer.max_iterations = 1;
  config.optimizer.initial_step = 0.0;
  config.optimizer.smoothness_weight = 0.0;
  config.optimizer.terminal_weight = 0.0;
  config.optimizer.fitness_weight = 0.0;
  auto fixed_searcher = std::make_shared<FixedPathSearcher>(nav3d::common::Path3D{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {1.0, 2.0, 0.0},
    {2.0, 2.0, 0.0},
  });
  nav3d::planner::EgoPlannerCore planner(config, fixed_searcher);

  const auto result = planner.plan(map, {0.0, 0.0, 0.0}, {2.0, 2.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.time_scaled);
  EXPECT_TRUE(result.feasibility.feasible);
  const nav3d::collision::TrajectoryFeasibilityChecker checker(
    config.feasibility_sample_step_seconds);
  const auto exact = checker.computeAnalyticExtrema(result.trajectory);
  const auto bounds = checker.computeAnalyticBounds(result.trajectory);
  EXPECT_LT(exact.max_velocity, bounds.max_velocity_bound);
  EXPECT_NEAR(result.feasibility.max_velocity_observed, exact.max_velocity, 1e-6);
  EXPECT_NEAR(result.feasibility.max_acceleration_observed, exact.max_acceleration, 1e-6);
}

TEST(EgoPlannerCore, PlanWithFallbacksRecordsWaitWhenShorterGoalsAreExhausted)
{
  ThinWallMap map;
  auto config = makeFastTestConfig();
  config.failure_cascade.max_optimization_retries = 0;
  config.failure_cascade.shorter_goal_ratios = {0.5};
  auto searcher = std::make_shared<RejectShortGoalSearcher>();
  nav3d::planner::EgoPlannerCore planner(config, searcher);

  const auto result = planner.planWithFallbacks(map, {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.fallback.action, nav3d::planner::FallbackAction::WaitForMapOrNewGoal);
  ASSERT_EQ(result.fallback_history.size(), 2u);
  EXPECT_EQ(result.fallback_history.at(0).action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  EXPECT_EQ(result.fallback_history.at(1).action, nav3d::planner::FallbackAction::WaitForMapOrNewGoal);
}

TEST(EgoPlannerCore, PlanWithFallbacksStopsOnImminentPostCheckCollision)
{
  NearStartWallMap map;
  auto config = makeFastTestConfig();
  config.failure_cascade.max_optimization_retries = 3;
  config.emergency_stop_horizon_seconds = 0.2;
  auto line_searcher = std::make_shared<GoalLineSearcher>();
  nav3d::planner::EgoPlannerCore planner(config, line_searcher);

  const auto result = planner.planWithFallbacks(map, {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.status, nav3d::planner::EgoPlanStatus::EmergencyStop);
  EXPECT_EQ(result.attempts, 1);
  ASSERT_EQ(result.fallback_history.size(), 1u);
  EXPECT_EQ(result.fallback_history.front().action, nav3d::planner::FallbackAction::EmergencyStop);
}

TEST(EgoPlannerCore, PlanWithFallbacksTriesNextShorterGoalWhenFirstShorterGoalSearchFails)
{
  ThinWallMap map;
  auto config = makeFastTestConfig();
  config.failure_cascade.max_optimization_retries = 0;
  config.failure_cascade.shorter_goal_ratios = {0.5, 0.25};
  auto searcher = std::make_shared<RejectGoalXSearcher>(1.5);
  nav3d::planner::EgoPlannerCore planner(config, searcher);

  const auto result = planner.planWithFallbacks(map, {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.attempts, 3);
  ASSERT_EQ(result.fallback_history.size(), 2u);
  EXPECT_EQ(result.fallback_history.at(0).action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  EXPECT_DOUBLE_EQ(result.fallback_history.at(0).next_goal->x, 1.5);
  EXPECT_EQ(result.fallback_history.at(1).action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  EXPECT_DOUBLE_EQ(result.fallback_history.at(1).next_goal->x, 0.75);
  EXPECT_DOUBLE_EQ(result.planned_goal.x, 0.75);
}
