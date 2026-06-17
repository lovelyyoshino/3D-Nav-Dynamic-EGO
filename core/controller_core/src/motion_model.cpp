#include "nav3d/controller/motion_model.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace nav3d::controller {
namespace {

void validateConfig(const MotionModelConfig& config)
{
  if (!std::isfinite(config.max_linear_speed) || config.max_linear_speed <= 0.0) {
    throw std::invalid_argument("max_linear_speed must be positive and finite");
  }
  if (!std::isfinite(config.max_linear_acceleration) ||
      config.max_linear_acceleration <= 0.0) {
    throw std::invalid_argument("max_linear_acceleration must be positive and finite");
  }
  if (!std::isfinite(config.max_yaw_rate) || config.max_yaw_rate < 0.0) {
    throw std::invalid_argument("max_yaw_rate must be non-negative and finite");
  }
  if (!std::isfinite(config.max_yaw_acceleration) || config.max_yaw_acceleration < 0.0) {
    throw std::invalid_argument("max_yaw_acceleration must be non-negative and finite");
  }
  if (config.type == MotionModelType::Ackermann &&
      (!std::isfinite(config.min_turning_radius) || config.min_turning_radius <= 0.0)) {
    throw std::invalid_argument("min_turning_radius must be positive and finite");
  }
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

common::Point3D limitVectorMagnitude(const common::Point3D& value, double max_norm)
{
  const double magnitude = common::norm(value);
  if (magnitude <= max_norm || magnitude <= 1e-12) {
    return value;
  }
  return value / magnitude * max_norm;
}

common::Point3D applyAccelerationLimit(
  const common::Point3D& desired,
  const common::Point3D& previous,
  double max_delta)
{
  const auto delta = desired - previous;
  return previous + limitVectorMagnitude(delta, max_delta);
}

double applyScalarAccelerationLimit(double desired, double previous, double max_delta)
{
  return previous + clamp(desired - previous, -max_delta, max_delta);
}

}  // namespace

MotionModel::MotionModel() : MotionModel(MotionModelConfig{})
{
}

MotionModel::MotionModel(MotionModelConfig config)
  : config_(std::move(config))
{
  validateConfig(config_);
}

TrajectoryCommand MotionModel::constrain(
  const TrajectoryCommand& desired,
  std::optional<TrajectoryCommand> previous,
  double dt) const
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("motion model dt must be positive and finite");
  }

  TrajectoryCommand constrained = desired;
  constrained.goal_reached = desired.goal_reached;

  switch (config_.type) {
    case MotionModelType::DifferentialDrive:
      constrained.linear.y = 0.0;
      constrained.linear.z = 0.0;
      break;
    case MotionModelType::Omni:
      constrained.linear.z = 0.0;
      break;
    case MotionModelType::Uav:
      break;
    case MotionModelType::Ackermann:
      constrained.linear.y = 0.0;
      constrained.linear.z = 0.0;
      break;
  }

  constrained.linear = limitVectorMagnitude(constrained.linear, config_.max_linear_speed);
  constrained.angular_z = clamp(
    constrained.angular_z,
    -config_.max_yaw_rate,
    config_.max_yaw_rate);
  if (config_.type == MotionModelType::Ackermann) {
    const double yaw_rate_from_turning_radius =
      std::abs(constrained.linear.x) / config_.min_turning_radius;
    constrained.angular_z = clamp(
      constrained.angular_z,
      -yaw_rate_from_turning_radius,
      yaw_rate_from_turning_radius);
  }

  if (previous.has_value()) {
    constrained.linear = applyAccelerationLimit(
      constrained.linear,
      previous->linear,
      config_.max_linear_acceleration * dt);
    constrained.angular_z = applyScalarAccelerationLimit(
      constrained.angular_z,
      previous->angular_z,
      config_.max_yaw_acceleration * dt);
  }

  return constrained;
}

const MotionModelConfig& MotionModel::config() const
{
  return config_;
}

}  // namespace nav3d::controller
