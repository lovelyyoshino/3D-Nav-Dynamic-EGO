#pragma once

#include <memory>
#include <vector>

#include "nav3d/collision/trajectory_feasibility_checker.h"
#include "nav3d/collision/trajectory_checker.h"
#include "nav3d/map/i_map.h"
#include "nav3d/planner/bspline/bspline_optimizer.h"
#include "nav3d/planner/i_path_searcher.h"
#include "nav3d/planner/replan_fsm.h"

namespace nav3d::planner {

enum class EgoPlanStatus {
  Success,
  SearchFailed,
  OptimizationFailed,
  DynamicFeasibilityViolation,
  TrajectoryCollision,
  EmergencyStop,
};

enum class DynamicFeasibilityMode {
  Sampled,
  AnalyticBounds,
  AnalyticExact,
};

struct EgoPlannerCoreConfig {
  SearchOptions search;
  BsplineOptimizerConfig optimizer;
  FailureCascadeConfig failure_cascade;
  double trajectory_sample_step_seconds = 0.05;
  double emergency_stop_horizon_seconds = 0.2;
  int max_fallback_attempts = 8;
  bool enable_dynamic_feasibility_check = false;
  DynamicFeasibilityMode dynamic_feasibility_mode = DynamicFeasibilityMode::Sampled;
  collision::DynamicLimits dynamic_limits;
  double feasibility_sample_step_seconds = 0.05;
  double max_dynamic_time_scale = 10.0;
  bool enable_initial_time_allocation = false;
  double min_time_allocation_interval = 0.05;
};

struct EgoPlanResult {
  bool success = false;
  EgoPlanStatus status = EgoPlanStatus::SearchFailed;
  int attempts = 0;
  common::Point3D requested_goal;
  common::Point3D planned_goal;
  SearchResult search;
  BsplineOptimizeResult optimization;
  collision::TrajectoryFeasibilityResult feasibility;
  collision::TrajectoryCollisionResult collision;
  UniformBspline trajectory;
  bool time_scaled = false;
  double time_scale = 1.0;
  FallbackDecision fallback;
  std::vector<FallbackDecision> fallback_history;
};

class EgoPlannerCore {
public:
  explicit EgoPlannerCore(EgoPlannerCoreConfig config = {});
  EgoPlannerCore(
    EgoPlannerCoreConfig config,
    std::shared_ptr<const IPathSearcher> path_searcher);

  EgoPlanResult plan(
    const map::IMap& map,
    const common::Point3D& start,
    const common::Point3D& goal);

  EgoPlanResult planWithFallbacks(
    const map::IMap& map,
    const common::Point3D& start,
    const common::Point3D& goal);

  void resetFallbackState();

private:
  EgoPlanResult runSingleAttempt(
    const map::IMap& map,
    const common::Point3D& start,
    const common::Point3D& requested_goal,
    const common::Point3D& attempt_goal,
    const BsplineOptimizerConfig& optimizer_config,
    ReplanFSM& fallback_fsm) const;

  bool isImminentCollision(const collision::TrajectoryCollisionResult& collision) const;

  EgoPlannerCoreConfig config_;
  std::shared_ptr<const IPathSearcher> path_searcher_;
  ReplanFSM fallback_fsm_;
};

}  // namespace nav3d::planner
