#pragma once

#include <optional>
#include <vector>

#include "nav3d/common/types.h"

namespace nav3d::planner {

enum class ReplanState {
  Normal,
  RetryingOptimization,
  ShorteningGoal,
  EmergencyStop,
  WaitingForMapOrGoal,
};

enum class FallbackAction {
  RetryOptimization,
  ReplanToShorterGoal,
  EmergencyStop,
  WaitForMapOrNewGoal,
};

struct FailureCascadeConfig {
  int max_optimization_retries = 3;
  double retry_collision_weight_multiplier = 2.0;
  std::vector<double> shorter_goal_ratios = {0.5, 0.3};
};

struct FallbackDecision {
  FallbackAction action = FallbackAction::WaitForMapOrNewGoal;
  ReplanState next_state = ReplanState::WaitingForMapOrGoal;
  double collision_weight_scale = 1.0;
  std::optional<common::Point3D> next_goal;
};

class ReplanFSM {
public:
  ReplanFSM();
  explicit ReplanFSM(FailureCascadeConfig config);

  FallbackDecision onTrajectoryCollision(
    const common::Point3D& current_position,
    const common::Point3D& original_goal,
    bool imminent_collision);

  void onShorterGoalRejected();
  void reset();
  ReplanState state() const;

private:
  FallbackDecision makeShorterGoalDecision(
    const common::Point3D& current_position,
    const common::Point3D& original_goal);

  FailureCascadeConfig config_;
  ReplanState state_ = ReplanState::Normal;
  int optimization_retries_used_ = 0;
  std::size_t shorter_goal_index_ = 0;
};

}  // namespace nav3d::planner
