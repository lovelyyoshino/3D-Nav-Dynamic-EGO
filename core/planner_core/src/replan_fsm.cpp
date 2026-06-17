#include "nav3d/planner/replan_fsm.h"

#include <cmath>
#include <stdexcept>

namespace nav3d::planner {
namespace {

void validateConfig(const FailureCascadeConfig& config)
{
  if (config.max_optimization_retries < 0) {
    throw std::invalid_argument("max_optimization_retries must be non-negative");
  }
  if (config.retry_collision_weight_multiplier <= 1.0) {
    throw std::invalid_argument("retry_collision_weight_multiplier must be greater than 1");
  }
  for (const double ratio : config.shorter_goal_ratios) {
    if (ratio <= 0.0 || ratio >= 1.0) {
      throw std::invalid_argument("shorter_goal_ratios must be in (0, 1)");
    }
  }
}

common::Point3D interpolateGoal(
  const common::Point3D& current_position,
  const common::Point3D& original_goal,
  double ratio)
{
  return current_position + (original_goal - current_position) * ratio;
}

}  // namespace

ReplanFSM::ReplanFSM() : ReplanFSM(FailureCascadeConfig{})
{
}

ReplanFSM::ReplanFSM(FailureCascadeConfig config) : config_(std::move(config))
{
  validateConfig(config_);
}

FallbackDecision ReplanFSM::onTrajectoryCollision(
  const common::Point3D& current_position,
  const common::Point3D& original_goal,
  bool imminent_collision)
{
  if (imminent_collision) {
    state_ = ReplanState::EmergencyStop;
    return {
      FallbackAction::EmergencyStop,
      state_,
      1.0,
      std::nullopt,
    };
  }

  if (optimization_retries_used_ < config_.max_optimization_retries) {
    ++optimization_retries_used_;
    state_ = ReplanState::RetryingOptimization;
    return {
      FallbackAction::RetryOptimization,
      state_,
      std::pow(config_.retry_collision_weight_multiplier, optimization_retries_used_),
      std::nullopt,
    };
  }

  return makeShorterGoalDecision(current_position, original_goal);
}

void ReplanFSM::onShorterGoalRejected()
{
  if (shorter_goal_index_ < config_.shorter_goal_ratios.size()) {
    ++shorter_goal_index_;
  }
}

void ReplanFSM::reset()
{
  state_ = ReplanState::Normal;
  optimization_retries_used_ = 0;
  shorter_goal_index_ = 0;
}

ReplanState ReplanFSM::state() const
{
  return state_;
}

FallbackDecision ReplanFSM::makeShorterGoalDecision(
  const common::Point3D& current_position,
  const common::Point3D& original_goal)
{
  if (shorter_goal_index_ < config_.shorter_goal_ratios.size()) {
    state_ = ReplanState::ShorteningGoal;
    return {
      FallbackAction::ReplanToShorterGoal,
      state_,
      1.0,
      interpolateGoal(current_position, original_goal, config_.shorter_goal_ratios[shorter_goal_index_]),
    };
  }

  state_ = ReplanState::WaitingForMapOrGoal;
  return {
    FallbackAction::WaitForMapOrNewGoal,
    state_,
    1.0,
    std::nullopt,
  };
}

}  // namespace nav3d::planner
