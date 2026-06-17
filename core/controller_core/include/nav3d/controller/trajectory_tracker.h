#pragma once

#include <optional>

#include "nav3d/map/i_map.h"
#include "nav3d/controller/motion_model.h"
#include "nav3d/planner/bspline/uniform_bspline.h"

namespace nav3d::controller {

enum class CommandFrame {
  Map,
  Body,
};

struct TrajectoryTrackerConfig {
  double sample_step_seconds = 0.05;
  double lookahead_time = 0.4;
  double control_dt = 0.1;
  double max_linear_speed = 0.6;
  double goal_tolerance = 0.15;
  double yaw_gain = 2.0;
  double max_yaw_rate = 0.75;
  double command_path_sample_step = 0.05;
  CommandFrame command_frame = CommandFrame::Body;
  MotionModelConfig motion_model{MotionModelType::Omni};
};

class TrajectoryTracker {
public:
  TrajectoryTracker();
  explicit TrajectoryTracker(TrajectoryTrackerConfig config);

  TrajectoryCommand computeCommand(
    const planner::UniformBspline& trajectory,
    const common::Point3D& current_position,
    double current_yaw,
    const common::Point3D& goal,
    std::optional<TrajectoryCommand> previous_command = std::nullopt) const;

  TrajectoryCommand computeCommand(
    const planner::UniformBspline& trajectory,
    const common::Point3D& current_position,
    double current_yaw,
    const common::Point3D& goal,
    const map::IMap& map,
    std::optional<TrajectoryCommand> previous_command = std::nullopt) const;

  const TrajectoryTrackerConfig& config() const;

private:
  TrajectoryTrackerConfig config_;
};

}  // namespace nav3d::controller
