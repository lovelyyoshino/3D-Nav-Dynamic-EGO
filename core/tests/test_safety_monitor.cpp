#include <gtest/gtest.h>

#include "nav3d/controller/safety_monitor.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/bspline/uniform_bspline.h"

namespace {

nav3d::planner::UniformBspline makeLineTrajectory()
{
  return nav3d::planner::UniformBspline(
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}},
    1,
    1.0);
}

nav3d::controller::SafetyMonitor makeMonitor()
{
  nav3d::controller::SafetyMonitorConfig config;
  config.trajectory_sample_step_seconds = 0.05;
  config.emergency_stop_time_horizon = 0.5;
  config.lookahead_time_horizon = 2.5;
  return nav3d::controller::SafetyMonitor(config);
}

nav3d::planner::ReplanFSM makeShorteningFsm()
{
  nav3d::planner::FailureCascadeConfig config;
  config.max_optimization_retries = 0;
  config.shorter_goal_ratios = {0.5};
  return nav3d::planner::ReplanFSM(config);
}

}  // namespace

TEST(SafetyMonitor, RequestsEmergencyStopWhenCurrentPoseIsOccupied)
{
  nav3d::map::VoxelGridMap map(0.1);
  map.insertOccupied({0.0, 0.0, 0.0});
  auto fsm = makeShorteningFsm();

  const auto decision = makeMonitor().evaluate(
    map,
    {0.0, 0.0, 0.0},
    makeLineTrajectory(),
    {3.0, 0.0, 0.0},
    &fsm);

  EXPECT_EQ(decision.action, nav3d::controller::SafetyAction::EmergencyStop);
  EXPECT_TRUE(decision.current_pose_in_collision);
  EXPECT_FALSE(decision.first_collision_time.has_value());
  ASSERT_TRUE(decision.fallback.has_value());
  EXPECT_EQ(decision.fallback->action, nav3d::planner::FallbackAction::EmergencyStop);
}

TEST(SafetyMonitor, RequestsEmergencyStopWhenLookaheadCollisionIsNear)
{
  nav3d::map::VoxelGridMap map(0.1);
  map.insertOccupied({0.25, 0.0, 0.0});
  auto fsm = makeShorteningFsm();

  const auto decision = makeMonitor().evaluate(
    map,
    {0.0, 0.0, 0.0},
    makeLineTrajectory(),
    {3.0, 0.0, 0.0},
    &fsm);

  EXPECT_EQ(decision.action, nav3d::controller::SafetyAction::EmergencyStop);
  EXPECT_FALSE(decision.current_pose_in_collision);
  ASSERT_TRUE(decision.first_collision_time.has_value());
  EXPECT_LE(*decision.first_collision_time, 0.5);
  ASSERT_TRUE(decision.fallback.has_value());
  EXPECT_EQ(decision.fallback->action, nav3d::planner::FallbackAction::EmergencyStop);
}

TEST(SafetyMonitor, RequestsReplanWhenLookaheadCollisionIsLater)
{
  nav3d::map::VoxelGridMap map(0.1);
  map.insertOccupied({1.5, 0.0, 0.0});
  auto fsm = makeShorteningFsm();

  const auto decision = makeMonitor().evaluate(
    map,
    {0.0, 0.0, 0.0},
    makeLineTrajectory(),
    {3.0, 0.0, 0.0},
    &fsm);

  EXPECT_EQ(decision.action, nav3d::controller::SafetyAction::ReplanNeeded);
  EXPECT_FALSE(decision.current_pose_in_collision);
  ASSERT_TRUE(decision.first_collision_time.has_value());
  EXPECT_GT(*decision.first_collision_time, 0.5);
  ASSERT_TRUE(decision.fallback.has_value());
  EXPECT_EQ(decision.fallback->action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  ASSERT_TRUE(decision.fallback->next_goal.has_value());
  EXPECT_DOUBLE_EQ(decision.fallback->next_goal->x, 1.5);
}

TEST(SafetyMonitor, TakesNoActionWhenPoseAndLookaheadAreClear)
{
  nav3d::map::VoxelGridMap map(0.1);

  const auto decision = makeMonitor().evaluate(
    map,
    {0.0, 0.0, 0.0},
    makeLineTrajectory(),
    {3.0, 0.0, 0.0});

  EXPECT_EQ(decision.action, nav3d::controller::SafetyAction::NoAction);
  EXPECT_FALSE(decision.current_pose_in_collision);
  EXPECT_FALSE(decision.first_collision_time.has_value());
  EXPECT_FALSE(decision.first_collision_point.has_value());
  EXPECT_FALSE(decision.fallback.has_value());
}

TEST(SafetyMonitor, IgnoresCollisionBehindCurrentTrajectoryProgress)
{
  nav3d::map::VoxelGridMap map(0.1);
  map.insertOccupied({0.25, 0.0, 0.0});

  const auto decision = makeMonitor().evaluate(
    map,
    {2.0, 0.0, 0.0},
    makeLineTrajectory(),
    {3.0, 0.0, 0.0});

  EXPECT_EQ(decision.action, nav3d::controller::SafetyAction::NoAction);
  EXPECT_FALSE(decision.first_collision_time.has_value());
}

TEST(SafetyMonitor, TreatsNearCollisionAheadOfCurrentProgressAsEmergency)
{
  nav3d::map::VoxelGridMap map(0.1);
  map.insertOccupied({2.25, 0.0, 0.0});

  const auto decision = makeMonitor().evaluate(
    map,
    {2.0, 0.0, 0.0},
    makeLineTrajectory(),
    {3.0, 0.0, 0.0});

  EXPECT_EQ(decision.action, nav3d::controller::SafetyAction::EmergencyStop);
  ASSERT_TRUE(decision.first_collision_time.has_value());
  EXPECT_GT(*decision.first_collision_time, 2.0);
}
