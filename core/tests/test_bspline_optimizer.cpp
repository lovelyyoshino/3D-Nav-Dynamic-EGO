#include <gtest/gtest.h>

#include <limits>

#include "nav3d/collision/trajectory_checker.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/bspline/bspline_optimizer.h"

namespace {

double smoothnessCost(const std::vector<nav3d::common::Point3D>& points)
{
  return nav3d::planner::BsplineOptimizer::evaluateCost(points, nav3d::planner::BsplineOptimizerConfig{}).smoothness;
}

}  // namespace

TEST(BsplineOptimizer, FitsAStarPathAsWarmStartSpline)
{
  const std::vector<nav3d::common::Point3D> path{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {2.0, 1.0, 0.0},
    {3.0, 1.0, 0.0},
  };

  const auto warm_start = nav3d::planner::BsplineOptimizer::makeWarmStart(path, 0.2);

  EXPECT_NEAR(warm_start.evaluate(0.0).x, 0.0, 1e-9);
  EXPECT_NEAR(warm_start.evaluate(warm_start.duration()).x, 3.0, 1e-9);
  EXPECT_GE(warm_start.controlPoints().size(), path.size());
}

TEST(BsplineOptimizer, ReducesSmoothnessCostWhileKeepingEndpointsFixed)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({-2.0, -2.0, 0.0}, {6.0, 4.0, 0.0});
  const std::vector<nav3d::common::Point3D> path{
    {0.0, 0.0, 0.0},
    {1.0, 2.0, 0.0},
    {2.0, -1.0, 0.0},
    {3.0, 2.0, 0.0},
    {4.0, 0.0, 0.0},
  };
  nav3d::planner::BsplineOptimizerConfig config;
  config.max_iterations = 30;
  config.collision_weight = 0.0;
  config.terminal_weight = 20.0;

  const auto result = nav3d::planner::BsplineOptimizer::optimize(path, map, config);

  ASSERT_TRUE(result.success);
  EXPECT_LT(result.final_cost, result.initial_cost);
  EXPECT_LT(smoothnessCost(result.spline.controlPoints()), smoothnessCost(result.warm_start.controlPoints()));
  EXPECT_NEAR(result.spline.evaluate(0.0).x, path.front().x, 1e-6);
  EXPECT_NEAR(result.spline.evaluate(result.spline.duration()).x, path.back().x, 1e-6);
}

TEST(BsplineOptimizer, UsesLocalAStarReboundToMoveWarmStartAwayFromObstacle)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, -2.0, 0.0}, {4.5, 2.0, 1.0});
  map.insertOccupied({2.0, 0.0, 0.0});
  const std::vector<nav3d::common::Point3D> path{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {2.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {4.0, 0.0, 0.0},
  };
  nav3d::planner::BsplineOptimizerConfig config;
  config.max_iterations = 40;
  config.collision_weight = 12.0;
  config.smoothness_weight = 0.2;
  config.rebound_search.mode = nav3d::planner::PlanningMode::Mode2D;
  config.rebound_search.allow_diagonal = false;

  const auto result = nav3d::planner::BsplineOptimizer::optimize(path, map, config);

  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.used_rebound);
  EXPECT_GT(result.rebound_segments, 0);
  EXPECT_LT(result.final_cost, result.initial_cost);
  EXPECT_FALSE(map.isOccupied(result.spline.controlPoints().at(4)));

  nav3d::collision::TrajectoryChecker checker(0.05);
  EXPECT_FALSE(checker.check(map, result.spline).in_collision);
}

TEST(BsplineOptimizer, ReportsFailureWhenCollidingWarmStartHasNoLocalReboundPath)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
  map.insertOccupied({2.0, 0.0, 0.0});
  const std::vector<nav3d::common::Point3D> path{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {2.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {4.0, 0.0, 0.0},
  };
  nav3d::planner::BsplineOptimizerConfig config;
  config.max_iterations = 20;
  config.rebound_search.mode = nav3d::planner::PlanningMode::Mode2D;
  config.rebound_search.allow_diagonal = false;

  const auto result = nav3d::planner::BsplineOptimizer::optimize(path, map, config);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.used_rebound);
  EXPECT_NEAR(result.spline.evaluate(0.0).x, 0.0, 1e-9);
  EXPECT_NEAR(result.spline.evaluate(result.spline.duration()).x, 4.0, 1e-9);
}

TEST(BsplineOptimizer, ReportsVelocityFeasibilityCostFromInterval)
{
  const std::vector<nav3d::common::Point3D> points{
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {2.0, 0.0, 0.0},
    {4.0, 0.0, 0.0},
    {4.0, 0.0, 0.0},
    {4.0, 0.0, 0.0},
  };
  nav3d::planner::BsplineOptimizerConfig config;
  config.interval = 0.5;
  config.feasibility_weight = 1.0;
  config.max_velocity = 1.0;
  config.max_acceleration = std::numeric_limits<double>::infinity();

  const auto cost = nav3d::planner::BsplineOptimizer::evaluateCost(points, config);

  EXPECT_GT(cost.feasibility, 0.0);
  EXPECT_NEAR(
    cost.total,
    cost.smoothness + cost.collision + cost.terminal + cost.fitness + cost.feasibility,
    1e-9);
}

TEST(BsplineOptimizer, ReportsAccelerationFeasibilityCostFromInterval)
{
  const std::vector<nav3d::common::Point3D> points{
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
  };
  nav3d::planner::BsplineOptimizerConfig config;
  config.interval = 1.0;
  config.feasibility_weight = 2.0;
  config.max_velocity = std::numeric_limits<double>::infinity();
  config.max_acceleration = 0.5;

  const auto cost = nav3d::planner::BsplineOptimizer::evaluateCost(points, config);

  EXPECT_GT(cost.feasibility, 0.0);
  EXPECT_NEAR(
    cost.total,
    cost.smoothness + cost.collision + cost.terminal + cost.fitness + cost.feasibility,
    1e-9);
}

TEST(BsplineOptimizer, ReducesFeasibilityCostWhenVelocityLimitIsEnabled)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({-1.0, -1.0, 0.0}, {8.0, 1.0, 0.0});
  const std::vector<nav3d::common::Point3D> path{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {6.0, 0.0, 0.0},
    {7.0, 0.0, 0.0},
  };
  nav3d::planner::BsplineOptimizerConfig config;
  config.interval = 0.2;
  config.max_iterations = 60;
  config.smoothness_weight = 0.1;
  config.collision_weight = 0.0;
  config.terminal_weight = 20.0;
  config.fitness_weight = 0.01;
  config.feasibility_weight = 5.0;
  config.max_velocity = 12.0;
  config.max_acceleration = std::numeric_limits<double>::infinity();

  const auto result = nav3d::planner::BsplineOptimizer::optimize(path, map, config);
  const auto initial = nav3d::planner::BsplineOptimizer::evaluateCost(result.warm_start.controlPoints(), config);
  const auto final = nav3d::planner::BsplineOptimizer::evaluateCost(result.spline.controlPoints(), config);

  ASSERT_TRUE(result.success);
  EXPECT_LT(final.feasibility, initial.feasibility);
  EXPECT_NEAR(result.spline.evaluate(0.0).x, path.front().x, 1e-6);
  EXPECT_NEAR(result.spline.evaluate(result.spline.duration()).x, path.back().x, 1e-6);
}

TEST(BsplineOptimizer, ReducesFeasibilityCostWhenAccelerationLimitIsEnabled)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({-1.0, -1.0, 0.0}, {4.0, 1.0, 0.0});
  const std::vector<nav3d::common::Point3D> path{
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
  };
  nav3d::planner::BsplineOptimizerConfig config;
  config.interval = 1.0;
  config.max_iterations = 60;
  config.smoothness_weight = 0.1;
  config.collision_weight = 0.0;
  config.terminal_weight = 20.0;
  config.fitness_weight = 0.01;
  config.feasibility_weight = 5.0;
  config.max_velocity = std::numeric_limits<double>::infinity();
  config.max_acceleration = 1.0;

  const auto result = nav3d::planner::BsplineOptimizer::optimize(path, map, config);
  const auto initial = nav3d::planner::BsplineOptimizer::evaluateCost(result.warm_start.controlPoints(), config);
  const auto final = nav3d::planner::BsplineOptimizer::evaluateCost(result.spline.controlPoints(), config);

  ASSERT_TRUE(result.success);
  EXPECT_LT(final.feasibility, initial.feasibility);
  EXPECT_NEAR(result.spline.evaluate(0.0).x, path.front().x, 1e-6);
  EXPECT_NEAR(result.spline.evaluate(result.spline.duration()).x, path.back().x, 1e-6);
}
