#include <gtest/gtest.h>

#include "nav3d/planner/replan_fsm.h"

TEST(ReplanFSM, RetriesOptimizationBeforeShorteningGoal)
{
  nav3d::planner::FailureCascadeConfig config;
  config.max_optimization_retries = 2;
  config.retry_collision_weight_multiplier = 2.0;
  nav3d::planner::ReplanFSM fsm(config);

  const auto first = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);
  EXPECT_EQ(first.action, nav3d::planner::FallbackAction::RetryOptimization);
  EXPECT_DOUBLE_EQ(first.collision_weight_scale, 2.0);

  const auto second = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);
  EXPECT_EQ(second.action, nav3d::planner::FallbackAction::RetryOptimization);
  EXPECT_DOUBLE_EQ(second.collision_weight_scale, 4.0);

  const auto third = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);
  EXPECT_EQ(third.action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  ASSERT_TRUE(third.next_goal.has_value());
  EXPECT_DOUBLE_EQ(third.next_goal->x, 5.0);
  EXPECT_DOUBLE_EQ(third.next_goal->y, 0.0);
  EXPECT_DOUBLE_EQ(third.next_goal->z, 0.0);
}

TEST(ReplanFSM, UsesNextCloserGoalRatioAfterPreviousShorteningAttempt)
{
  nav3d::planner::FailureCascadeConfig config;
  config.max_optimization_retries = 0;
  config.shorter_goal_ratios = {0.5, 0.3};
  nav3d::planner::ReplanFSM fsm(config);

  const auto first = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);
  ASSERT_TRUE(first.next_goal.has_value());
  EXPECT_DOUBLE_EQ(first.next_goal->x, 5.0);

  fsm.onShorterGoalRejected();
  const auto second = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);

  EXPECT_EQ(second.action, nav3d::planner::FallbackAction::ReplanToShorterGoal);
  ASSERT_TRUE(second.next_goal.has_value());
  EXPECT_DOUBLE_EQ(second.next_goal->x, 3.0);
}

TEST(ReplanFSM, RequestsEmergencyStopForImminentCollision)
{
  nav3d::planner::ReplanFSM fsm;

  const auto decision = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, true);

  EXPECT_EQ(decision.action, nav3d::planner::FallbackAction::EmergencyStop);
  EXPECT_EQ(fsm.state(), nav3d::planner::ReplanState::EmergencyStop);
}

TEST(ReplanFSM, ReportsWaitForMapWhenShorterGoalsAreExhausted)
{
  nav3d::planner::FailureCascadeConfig config;
  config.max_optimization_retries = 0;
  config.shorter_goal_ratios = {0.5};
  nav3d::planner::ReplanFSM fsm(config);

  const auto first = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);
  ASSERT_EQ(first.action, nav3d::planner::FallbackAction::ReplanToShorterGoal);

  fsm.onShorterGoalRejected();
  const auto second = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);

  EXPECT_EQ(second.action, nav3d::planner::FallbackAction::WaitForMapOrNewGoal);
  EXPECT_EQ(fsm.state(), nav3d::planner::ReplanState::WaitingForMapOrGoal);
}

TEST(ReplanFSM, ResetRestoresNormalPlanning)
{
  nav3d::planner::FailureCascadeConfig config;
  config.max_optimization_retries = 0;
  nav3d::planner::ReplanFSM fsm(config);

  const auto shortened = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);
  ASSERT_EQ(shortened.action, nav3d::planner::FallbackAction::ReplanToShorterGoal);

  fsm.reset();
  const auto after_reset = fsm.onTrajectoryCollision({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, false);

  EXPECT_EQ(fsm.state(), nav3d::planner::ReplanState::ShorteningGoal);
  ASSERT_TRUE(after_reset.next_goal.has_value());
  EXPECT_DOUBLE_EQ(after_reset.next_goal->x, 5.0);
}
