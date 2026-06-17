#pragma once

#include <optional>

#include "nav3d/collision/trajectory_checker.h"
#include "nav3d/common/types.h"
#include "nav3d/map/i_map.h"
#include "nav3d/planner/bspline/uniform_bspline.h"
#include "nav3d/planner/replan_fsm.h"

namespace nav3d::controller {

enum class SafetyAction {
  NoAction,
  ReplanNeeded,
  EmergencyStop,
};

struct SafetyMonitorConfig {
  double trajectory_sample_step_seconds = 0.05;
  double emergency_stop_time_horizon = 0.5;
  double lookahead_time_horizon = 2.0;
};

struct SafetyDecision {
  SafetyAction action = SafetyAction::NoAction;
  bool current_pose_in_collision = false;
  std::optional<double> first_collision_time;
  std::optional<common::Point3D> first_collision_point;
  std::optional<planner::FallbackDecision> fallback;
};

class SafetyMonitor {
public:
  SafetyMonitor();
  explicit SafetyMonitor(SafetyMonitorConfig config);

  SafetyDecision evaluate(
    const map::IMap& map,
    const common::Point3D& current_position,
    const planner::UniformBspline& trajectory,
    const common::Point3D& original_goal,
    planner::ReplanFSM* replan_fsm = nullptr) const;

  const SafetyMonitorConfig& config() const;

private:
  SafetyMonitorConfig config_;
  collision::TrajectoryChecker trajectory_checker_;
};

}  // namespace nav3d::controller
