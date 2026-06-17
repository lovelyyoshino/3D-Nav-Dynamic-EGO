#pragma once

#include <optional>

#include "nav3d/common/types.h"

namespace nav3d::controller {

enum class MotionModelType {
  DifferentialDrive,
  Omni,
  Uav,
  Ackermann,
};

struct TrajectoryCommand {
  common::Point3D linear;
  double angular_z = 0.0;
  bool goal_reached = false;
};

struct MotionModelConfig {
  MotionModelType type = MotionModelType::DifferentialDrive;
  double max_linear_speed = 0.6;
  double max_linear_acceleration = 1.0;
  double max_yaw_rate = 0.75;
  double max_yaw_acceleration = 1.5;
  double min_turning_radius = 1.0;
};

class MotionModel {
public:
  MotionModel();
  explicit MotionModel(MotionModelConfig config);

  TrajectoryCommand constrain(
    const TrajectoryCommand& desired,
    std::optional<TrajectoryCommand> previous,
    double dt) const;

  const MotionModelConfig& config() const;

private:
  MotionModelConfig config_;
};

}  // namespace nav3d::controller
