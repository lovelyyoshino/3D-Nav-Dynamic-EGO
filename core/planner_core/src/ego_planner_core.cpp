#include "nav3d/planner/ego_planner_core.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include "nav3d/planner/path_searching/astar_3d.h"
#include "nav3d/planner/path_searching/jps_3d.h"

namespace nav3d::planner {
namespace {

std::shared_ptr<const IPathSearcher> makePathSearcher(SearchAlgorithm algorithm)
{
  switch (algorithm) {
    case SearchAlgorithm::AStar:
      return std::make_shared<AStar3D>();
    case SearchAlgorithm::Jps:
      return std::make_shared<Jps3D>();
  }
  return std::make_shared<AStar3D>();
}

collision::TrajectoryFeasibilityResult checkDynamicFeasibility(
  const collision::TrajectoryFeasibilityChecker& checker,
  const UniformBspline& trajectory,
  const collision::DynamicLimits& limits,
  DynamicFeasibilityMode mode)
{
  switch (mode) {
    case DynamicFeasibilityMode::Sampled:
      return checker.check(trajectory, limits);
    case DynamicFeasibilityMode::AnalyticBounds:
      return checker.checkAnalyticBounds(trajectory, limits);
    case DynamicFeasibilityMode::AnalyticExact:
      return checker.checkAnalyticExact(trajectory, limits);
  }
  return checker.check(trajectory, limits);
}

double pathLength(const common::Path3D& path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += common::distance(path[i - 1], path[i]);
  }
  return length;
}

BsplineOptimizerConfig allocateInitialTime(
  const common::Path3D& path,
  const EgoPlannerCoreConfig& planner_config,
  BsplineOptimizerConfig optimizer_config)
{
  if (!planner_config.enable_initial_time_allocation ||
      !std::isfinite(planner_config.dynamic_limits.max_velocity) ||
      planner_config.dynamic_limits.max_velocity <= 0.0) {
    return optimizer_config;
  }

  const double desired_duration = pathLength(path) / planner_config.dynamic_limits.max_velocity;
  if (!std::isfinite(desired_duration) || desired_duration <= 0.0) {
    return optimizer_config;
  }

  const std::size_t control_point_count = path.size() + 4u;
  const std::size_t segment_count =
    control_point_count > 3u ? control_point_count - 3u : 1u;
  optimizer_config.interval = std::max(
    planner_config.min_time_allocation_interval,
    desired_duration / static_cast<double>(segment_count));
  return optimizer_config;
}

double spatialSampleStep(const map::IMap& map)
{
  const double resolution = map.getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return 0.05;
  }
  return std::max(1e-3, resolution * 0.5);
}

common::Path3D densifyPathForContinuousTrajectory(
  const common::Path3D& path,
  const map::IMap& map)
{
  if (path.size() < 2) {
    return path;
  }

  const double step = spatialSampleStep(map);
  common::Path3D dense_path;
  dense_path.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto& from = path[i - 1];
    const auto& to = path[i];
    const double distance = common::distance(from, to);
    if (!std::isfinite(distance) || distance <= 1e-12) {
      continue;
    }
    const int subdivisions = std::max(1, static_cast<int>(std::ceil(distance / step)));
    for (int j = 1; j <= subdivisions; ++j) {
      const double ratio = static_cast<double>(j) / static_cast<double>(subdivisions);
      dense_path.push_back(from + (to - from) * ratio);
    }
  }
  return dense_path;
}

std::optional<UniformBspline> makePathFollowingTrajectory(
  const common::Path3D& path,
  const map::IMap& map,
  double interval)
{
  auto dense_path = densifyPathForContinuousTrajectory(path, map);
  if (dense_path.size() < 2) {
    return std::nullopt;
  }
  return UniformBspline(std::move(dense_path), 1, interval);
}

bool enforceDynamicFeasibility(
  const EgoPlannerCoreConfig& config,
  EgoPlanResult& result)
{
  if (!config.enable_dynamic_feasibility_check) {
    return true;
  }

  const collision::TrajectoryFeasibilityChecker feasibility_checker(
    config.feasibility_sample_step_seconds);
  result.feasibility = checkDynamicFeasibility(
    feasibility_checker,
    result.trajectory,
    config.dynamic_limits,
    config.dynamic_feasibility_mode);
  if (result.feasibility.feasible) {
    return true;
  }

  const double required_scale = result.feasibility.required_time_scale;
  if (std::isfinite(required_scale) && required_scale <= config.max_dynamic_time_scale) {
    const double scale = std::min(required_scale * 1.01, config.max_dynamic_time_scale);
    result.trajectory = result.trajectory.rescaleTime(scale);
    result.time_scaled = true;
    result.time_scale = scale;
    result.feasibility = checkDynamicFeasibility(
      feasibility_checker,
      result.trajectory,
      config.dynamic_limits,
      config.dynamic_feasibility_mode);
  }

  if (!result.feasibility.feasible) {
    result.status = EgoPlanStatus::DynamicFeasibilityViolation;
    return false;
  }
  return true;
}

}  // namespace

EgoPlannerCore::EgoPlannerCore(EgoPlannerCoreConfig config)
  : EgoPlannerCore(config, makePathSearcher(config.search.algorithm))
{
}

EgoPlannerCore::EgoPlannerCore(
  EgoPlannerCoreConfig config,
  std::shared_ptr<const IPathSearcher> path_searcher)
  : config_(std::move(config)),
    path_searcher_(std::move(path_searcher)),
    fallback_fsm_(config_.failure_cascade)
{
  if (!path_searcher_) {
    throw std::invalid_argument("EgoPlannerCore requires a path searcher");
  }
  if (config_.max_fallback_attempts <= 0) {
    throw std::invalid_argument("max_fallback_attempts must be positive");
  }
  if (config_.feasibility_sample_step_seconds <= 0.0) {
    throw std::invalid_argument("feasibility_sample_step_seconds must be positive");
  }
  if (config_.max_dynamic_time_scale <= 0.0) {
    throw std::invalid_argument("max_dynamic_time_scale must be positive");
  }
  if (config_.min_time_allocation_interval <= 0.0) {
    throw std::invalid_argument("min_time_allocation_interval must be positive");
  }
}

EgoPlanResult EgoPlannerCore::plan(
  const map::IMap& map,
  const common::Point3D& start,
  const common::Point3D& goal)
{
  return runSingleAttempt(map, start, goal, goal, config_.optimizer, fallback_fsm_);
}

EgoPlanResult EgoPlannerCore::planWithFallbacks(
  const map::IMap& map,
  const common::Point3D& start,
  const common::Point3D& goal)
{
  ReplanFSM fallback_fsm(config_.failure_cascade);
  BsplineOptimizerConfig optimizer_config = config_.optimizer;
  common::Point3D active_goal = goal;
  std::vector<FallbackDecision> history;
  EgoPlanResult last_result;

  for (int attempt = 1; attempt <= config_.max_fallback_attempts; ++attempt) {
    const bool planning_shorter_goal = active_goal != goal;
    last_result = runSingleAttempt(map, start, goal, active_goal, optimizer_config, fallback_fsm);
    last_result.attempts = attempt;
    last_result.fallback_history = history;

    if (last_result.success) {
      return last_result;
    }

    if (planning_shorter_goal &&
        (last_result.status == EgoPlanStatus::SearchFailed ||
         last_result.fallback.action == FallbackAction::ReplanToShorterGoal)) {
      fallback_fsm.onShorterGoalRejected();
      last_result.fallback = fallback_fsm.onTrajectoryCollision(start, goal, false);
    }

    const bool should_record_fallback =
      last_result.fallback.action != FallbackAction::WaitForMapOrNewGoal ||
      planning_shorter_goal || !history.empty();
    if (should_record_fallback) {
      history.push_back(last_result.fallback);
      last_result.fallback_history = history;
    }

    if (last_result.fallback.action == FallbackAction::RetryOptimization) {
      optimizer_config = config_.optimizer;
      optimizer_config.collision_weight *= last_result.fallback.collision_weight_scale;
      continue;
    }

    if (last_result.fallback.action == FallbackAction::ReplanToShorterGoal &&
        last_result.fallback.next_goal.has_value()) {
      active_goal = *last_result.fallback.next_goal;
      optimizer_config = config_.optimizer;
      continue;
    }

    return last_result;
  }

  last_result.fallback_history = std::move(history);
  return last_result;
}

void EgoPlannerCore::resetFallbackState()
{
  fallback_fsm_.reset();
}

EgoPlanResult EgoPlannerCore::runSingleAttempt(
  const map::IMap& map,
  const common::Point3D& start,
  const common::Point3D& requested_goal,
  const common::Point3D& attempt_goal,
  const BsplineOptimizerConfig& optimizer_config,
  ReplanFSM& fallback_fsm) const
{
  EgoPlanResult result;
  result.attempts = 1;
  result.requested_goal = requested_goal;
  result.planned_goal = attempt_goal;

  result.search = path_searcher_->search(map, start, attempt_goal, config_.search);
  if (result.search.status != SearchStatus::Success || result.search.path.empty()) {
    result.status = EgoPlanStatus::SearchFailed;
    return result;
  }
  result.planned_goal = result.search.path.back();

  const auto allocated_optimizer_config =
    allocateInitialTime(result.search.path, config_, optimizer_config);
  result.optimization =
    BsplineOptimizer::optimize(result.search.path, map, allocated_optimizer_config);
  result.trajectory = result.optimization.spline;
  if (!result.optimization.success) {
    result.status = EgoPlanStatus::OptimizationFailed;
    result.fallback = fallback_fsm.onTrajectoryCollision(start, requested_goal, false);
    result.fallback_history.push_back(result.fallback);
    return result;
  }

  if (!enforceDynamicFeasibility(config_, result)) {
    return result;
  }

  const collision::TrajectoryChecker checker(config_.trajectory_sample_step_seconds);
  result.collision = checker.check(map, result.trajectory);
  if (result.collision.in_collision) {
    const auto path_following_trajectory =
      makePathFollowingTrajectory(result.search.path, map, allocated_optimizer_config.interval);
    if (path_following_trajectory.has_value()) {
      result.trajectory = *path_following_trajectory;
      result.time_scaled = false;
      result.time_scale = 1.0;
      result.collision = checker.check(map, result.trajectory);
      if (!result.collision.in_collision) {
        if (!enforceDynamicFeasibility(config_, result)) {
          return result;
        }
        result.collision = checker.check(map, result.trajectory);
        if (!result.collision.in_collision) {
          result.success = true;
          result.status = EgoPlanStatus::Success;
          fallback_fsm.reset();
          return result;
        }
      }
    }

    const bool imminent = isImminentCollision(result.collision);
    result.status = imminent ? EgoPlanStatus::EmergencyStop : EgoPlanStatus::TrajectoryCollision;
    result.fallback = fallback_fsm.onTrajectoryCollision(start, requested_goal, imminent);
    result.fallback_history.push_back(result.fallback);
    return result;
  }

  result.success = true;
  result.status = EgoPlanStatus::Success;
  fallback_fsm.reset();
  return result;
}

bool EgoPlannerCore::isImminentCollision(
  const collision::TrajectoryCollisionResult& collision) const
{
  return collision.first_collision_time.has_value() &&
         *collision.first_collision_time <= config_.emergency_stop_horizon_seconds;
}

}  // namespace nav3d::planner
