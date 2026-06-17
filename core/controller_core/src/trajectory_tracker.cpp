#include "nav3d/controller/trajectory_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav3d/controller/motion_model.h"

namespace nav3d::controller {
namespace {

constexpr double kPi = 3.14159265358979323846;

void validateConfig(const TrajectoryTrackerConfig& config)
{
  if (config.sample_step_seconds <= 0.0) {
    throw std::invalid_argument("sample_step_seconds must be positive");
  }
  if (config.lookahead_time < 0.0) {
    throw std::invalid_argument("lookahead_time must be non-negative");
  }
  if (config.control_dt <= 0.0) {
    throw std::invalid_argument("control_dt must be positive");
  }
  if (config.max_linear_speed <= 0.0) {
    throw std::invalid_argument("max_linear_speed must be positive");
  }
  if (config.motion_model.max_linear_speed <= 0.0) {
    throw std::invalid_argument("motion_model.max_linear_speed must be positive");
  }
  if (config.goal_tolerance <= 0.0) {
    throw std::invalid_argument("goal_tolerance must be positive");
  }
  if (config.yaw_gain < 0.0) {
    throw std::invalid_argument("yaw_gain must be non-negative");
  }
  if (config.max_yaw_rate < 0.0) {
    throw std::invalid_argument("max_yaw_rate must be non-negative");
  }
  if (config.command_path_sample_step <= 0.0) {
    throw std::invalid_argument("command_path_sample_step must be positive");
  }
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double estimateTrajectoryTime(
  const planner::UniformBspline& trajectory,
  const common::Point3D& current_position,
  double sample_step_seconds)
{
  const double duration = trajectory.duration();
  if (duration <= 0.0 || !std::isfinite(duration)) {
    return 0.0;
  }

  double best_time = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (double t = 0.0; t <= duration; t += sample_step_seconds) {
    const double candidate_distance =
      common::distance(current_position, trajectory.evaluate(t));
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      best_time = t;
    }
  }

  const double end_distance =
    common::distance(current_position, trajectory.evaluate(duration));
  if (end_distance < best_distance) {
    best_time = duration;
  }
  return best_time;
}

common::Point3D toBodyFrame(const common::Point3D& map_velocity, double yaw)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return {
    cos_yaw * map_velocity.x + sin_yaw * map_velocity.y,
    -sin_yaw * map_velocity.x + cos_yaw * map_velocity.y,
    map_velocity.z,
  };
}

common::Point3D toMapFrame(const common::Point3D& body_velocity, double yaw)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return {
    cos_yaw * body_velocity.x - sin_yaw * body_velocity.y,
    sin_yaw * body_velocity.x + cos_yaw * body_velocity.y,
    body_velocity.z,
  };
}

bool isFreeForCommand(const map::IMap& map, const common::Point3D& point)
{
  return map.isFree(point);
}

bool isSegmentFreeForCommand(
  const map::IMap& map,
  const common::Point3D& from,
  const common::Point3D& to,
  double sample_step)
{
  if (!isFreeForCommand(map, from) || !isFreeForCommand(map, to)) {
    return false;
  }

  const double length = common::distance(from, to);
  if (length <= 1e-9) {
    return true;
  }

  const int samples = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
  for (int i = 1; i < samples; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples);
    if (!isFreeForCommand(map, from + (to - from) * ratio)) {
      return false;
    }
  }
  return true;
}

std::optional<common::Point3D> farthestReachableTrajectoryPoint(
  const map::IMap& map,
  const planner::UniformBspline& trajectory,
  const common::Point3D& current_position,
  double current_time,
  double target_time,
  double trajectory_sample_step,
  double segment_sample_step)
{
  std::optional<common::Point3D> best;
  if (target_time <= current_time) {
    return best;
  }

  const double duration = trajectory.duration();
  const double end_time = std::min(target_time, duration);
  for (double t = current_time; t <= end_time; t += trajectory_sample_step) {
    const auto candidate = trajectory.evaluate(t);
    if (common::distance(current_position, candidate) <= 1e-9) {
      continue;
    }
    if (isSegmentFreeForCommand(map, current_position, candidate, segment_sample_step)) {
      best = candidate;
      continue;
    }
    if (best.has_value()) {
      break;
    }
  }

  const auto target = trajectory.evaluate(end_time);
  if (common::distance(current_position, target) > 1e-9 &&
      isSegmentFreeForCommand(map, current_position, target, segment_sample_step)) {
    best = target;
  }

  return best;
}

TrajectoryCommand makeCommandToTarget(
  const TrajectoryTrackerConfig& config,
  const common::Point3D& current_position,
  double current_yaw,
  const common::Point3D& target,
  std::optional<TrajectoryCommand> previous_command)
{
  const auto error = target - current_position;
  const double distance_to_target = common::norm(error);
  if (distance_to_target <= 1e-9) {
    return {};
  }

  const double speed = std::min(
    config.max_linear_speed,
    distance_to_target / config.control_dt);
  const auto map_velocity = error / distance_to_target * speed;

  TrajectoryCommand command;
  command.linear = config.command_frame == CommandFrame::Body
    ? toBodyFrame(map_velocity, current_yaw)
    : map_velocity;

  const double xy_speed = std::hypot(map_velocity.x, map_velocity.y);
  if (xy_speed > 1e-9 && config.max_yaw_rate > 0.0 && config.yaw_gain > 0.0) {
    const double desired_yaw = std::atan2(map_velocity.y, map_velocity.x);
    const double yaw_error = normalizeAngle(desired_yaw - current_yaw);
    command.angular_z = clamp(
      yaw_error * config.yaw_gain,
      -config.max_yaw_rate,
      config.max_yaw_rate);
  }
  MotionModelConfig motion_config = config.motion_model;
  motion_config.max_linear_speed = config.max_linear_speed;
  motion_config.max_yaw_rate = config.max_yaw_rate;
  const MotionModel motion_model(motion_config);
  return motion_model.constrain(command, previous_command, config.control_dt);
}

TrajectoryCommand clampCommandToFreeSegment(
  const TrajectoryTrackerConfig& config,
  const map::IMap& map,
  const common::Point3D& current_position,
  double current_yaw,
  TrajectoryCommand command)
{
  if (command.goal_reached) {
    return command;
  }

  const double segment_sample_step =
    std::min(config.command_path_sample_step, map.getResolution());
  common::Point3D map_velocity = config.command_frame == CommandFrame::Body
    ? toMapFrame(command.linear, current_yaw)
    : command.linear;
  const auto projected_position = current_position + map_velocity * config.control_dt;
  if (isSegmentFreeForCommand(map, current_position, projected_position, segment_sample_step)) {
    return command;
  }

  double low = 0.0;
  double high = 1.0;
  for (int iteration = 0; iteration < 24; ++iteration) {
    const double mid = (low + high) * 0.5;
    const auto candidate = current_position + map_velocity * config.control_dt * mid;
    if (isSegmentFreeForCommand(map, current_position, candidate, segment_sample_step)) {
      low = mid;
    } else {
      high = mid;
    }
  }

  if (low <= 1e-6) {
    command.linear = {};
    return command;
  }

  map_velocity = map_velocity * low;
  command.linear = config.command_frame == CommandFrame::Body
    ? toBodyFrame(map_velocity, current_yaw)
    : map_velocity;
  return command;
}

}  // namespace

TrajectoryTracker::TrajectoryTracker() : TrajectoryTracker(TrajectoryTrackerConfig{})
{
}

TrajectoryTracker::TrajectoryTracker(TrajectoryTrackerConfig config)
  : config_(std::move(config))
{
  validateConfig(config_);
}

TrajectoryCommand TrajectoryTracker::computeCommand(
  const planner::UniformBspline& trajectory,
  const common::Point3D& current_position,
  double current_yaw,
  const common::Point3D& goal,
  std::optional<TrajectoryCommand> previous_command) const
{
  if (common::distance(current_position, goal) <= config_.goal_tolerance) {
    return {{}, 0.0, true};
  }

  const double duration = trajectory.duration();
  if (duration <= 0.0 || !std::isfinite(duration)) {
    return {};
  }

  if (!std::isfinite(current_yaw)) {
    current_yaw = 0.0;
  }

  const double current_time = estimateTrajectoryTime(
    trajectory,
    current_position,
    config_.sample_step_seconds);
  const double target_time = std::min(duration, current_time + config_.lookahead_time);
  const auto target = trajectory.evaluate(target_time);
  return makeCommandToTarget(config_, current_position, current_yaw, target, previous_command);
}

TrajectoryCommand TrajectoryTracker::computeCommand(
  const planner::UniformBspline& trajectory,
  const common::Point3D& current_position,
  double current_yaw,
  const common::Point3D& goal,
  const map::IMap& map,
  std::optional<TrajectoryCommand> previous_command) const
{
  if (common::distance(current_position, goal) <= config_.goal_tolerance) {
    return {{}, 0.0, true};
  }

  const double duration = trajectory.duration();
  if (duration <= 0.0 || !std::isfinite(duration)) {
    return {};
  }

  if (!std::isfinite(current_yaw)) {
    current_yaw = 0.0;
  }

  const double current_time = estimateTrajectoryTime(
    trajectory,
    current_position,
    config_.sample_step_seconds);
  const double target_time = std::min(duration, current_time + config_.lookahead_time);
  const auto target = trajectory.evaluate(target_time);
  const double segment_sample_step =
    std::min(config_.command_path_sample_step, map.getResolution());

  if (isSegmentFreeForCommand(map, current_position, target, segment_sample_step)) {
    return clampCommandToFreeSegment(
      config_,
      map,
      current_position,
      current_yaw,
      makeCommandToTarget(config_, current_position, current_yaw, target, previous_command));
  }

  const auto reachable = farthestReachableTrajectoryPoint(
    map,
    trajectory,
    current_position,
    current_time,
    target_time,
    config_.sample_step_seconds,
    segment_sample_step);
  if (!reachable.has_value()) {
    return {};
  }

  return clampCommandToFreeSegment(
    config_,
    map,
    current_position,
    current_yaw,
    makeCommandToTarget(
      config_,
      current_position,
      current_yaw,
      *reachable,
      previous_command));
}

const TrajectoryTrackerConfig& TrajectoryTracker::config() const
{
  return config_;
}

}  // namespace nav3d::controller
