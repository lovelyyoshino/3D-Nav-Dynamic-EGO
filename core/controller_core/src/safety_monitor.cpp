#include "nav3d/controller/safety_monitor.h"

#include <limits>
#include <stdexcept>
#include <utility>

namespace nav3d::controller {
namespace {

void validateConfig(const SafetyMonitorConfig& config)
{
  if (config.trajectory_sample_step_seconds <= 0.0) {
    throw std::invalid_argument("trajectory_sample_step_seconds must be positive");
  }
  if (config.emergency_stop_time_horizon < 0.0) {
    throw std::invalid_argument("emergency_stop_time_horizon must be non-negative");
  }
  if (config.lookahead_time_horizon < 0.0) {
    throw std::invalid_argument("lookahead_time_horizon must be non-negative");
  }
  if (config.emergency_stop_time_horizon > config.lookahead_time_horizon) {
    throw std::invalid_argument("emergency_stop_time_horizon must not exceed lookahead_time_horizon");
  }
}

std::optional<planner::FallbackDecision> makeFallback(
  planner::ReplanFSM* replan_fsm,
  const common::Point3D& current_position,
  const common::Point3D& original_goal,
  bool imminent_collision)
{
  if (replan_fsm == nullptr) {
    return std::nullopt;
  }
  return replan_fsm->onTrajectoryCollision(current_position, original_goal, imminent_collision);
}

double estimateCurrentTrajectoryTime(
  const planner::UniformBspline& trajectory,
  const common::Point3D& current_position,
  double sample_step_seconds)
{
  const double duration = trajectory.duration();
  double best_time = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();

  for (double t = 0.0; t <= duration; t += sample_step_seconds) {
    const double candidate_distance = common::distance(current_position, trajectory.evaluate(t));
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      best_time = t;
    }
  }

  const double end_distance = common::distance(current_position, trajectory.evaluate(duration));
  if (end_distance < best_distance) {
    best_time = duration;
  }
  return best_time;
}

}  // namespace

SafetyMonitor::SafetyMonitor() : SafetyMonitor(SafetyMonitorConfig{})
{
}

SafetyMonitor::SafetyMonitor(SafetyMonitorConfig config)
  : config_(std::move(config)),
    trajectory_checker_(config_.trajectory_sample_step_seconds)
{
  validateConfig(config_);
}

SafetyDecision SafetyMonitor::evaluate(
  const map::IMap& map,
  const common::Point3D& current_position,
  const planner::UniformBspline& trajectory,
  const common::Point3D& original_goal,
  planner::ReplanFSM* replan_fsm) const
{
  if (map.isOccupied(current_position)) {
    return {
      SafetyAction::EmergencyStop,
      true,
      std::nullopt,
      std::nullopt,
      makeFallback(replan_fsm, current_position, original_goal, true),
    };
  }

  const double current_time = estimateCurrentTrajectoryTime(
    trajectory,
    current_position,
    config_.trajectory_sample_step_seconds);
  const double lookahead_end = current_time + config_.lookahead_time_horizon;

  const auto collision = trajectory_checker_.checkRange(map, trajectory, current_time, lookahead_end);
  if (!collision.in_collision || !collision.first_collision_time.has_value()) {
    return {};
  }

  const double relative_collision_time = *collision.first_collision_time - current_time;
  const bool imminent_collision = relative_collision_time <= config_.emergency_stop_time_horizon;

  return {
    imminent_collision ? SafetyAction::EmergencyStop : SafetyAction::ReplanNeeded,
    false,
    collision.first_collision_time,
    collision.first_collision_point,
    makeFallback(replan_fsm, current_position, original_goal, imminent_collision),
  };
}

const SafetyMonitorConfig& SafetyMonitor::config() const
{
  return config_;
}

}  // namespace nav3d::controller
