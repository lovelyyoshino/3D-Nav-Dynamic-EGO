#pragma once

#include <limits>
#include <optional>

#include "nav3d/planner/bspline/uniform_bspline.h"

namespace nav3d::collision {

struct DynamicLimits {
  double max_velocity = std::numeric_limits<double>::infinity();
  double max_acceleration = std::numeric_limits<double>::infinity();
};

struct TrajectoryFeasibilityResult {
  bool feasible = true;
  bool velocity_violation = false;
  bool acceleration_violation = false;
  std::optional<double> first_violation_time;
  double max_velocity_observed = 0.0;
  double max_acceleration_observed = 0.0;
  double required_time_scale = 1.0;
};

struct TrajectoryAnalyticBounds {
  double max_velocity_bound = 0.0;
  double max_acceleration_bound = 0.0;
};

struct TrajectoryAnalyticExtrema {
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
};

class TrajectoryFeasibilityChecker {
public:
  explicit TrajectoryFeasibilityChecker(double sample_step_seconds);

  TrajectoryFeasibilityResult check(
    const planner::UniformBspline& trajectory,
    const DynamicLimits& limits) const;

  TrajectoryAnalyticBounds computeAnalyticBounds(
    const planner::UniformBspline& trajectory) const;

  TrajectoryAnalyticExtrema computeAnalyticExtrema(
    const planner::UniformBspline& trajectory) const;

  TrajectoryFeasibilityResult checkAnalyticBounds(
    const planner::UniformBspline& trajectory,
    const DynamicLimits& limits) const;

  TrajectoryFeasibilityResult checkAnalyticExact(
    const planner::UniformBspline& trajectory,
    const DynamicLimits& limits) const;

private:
  double sample_step_seconds_ = 0.05;
};

}  // namespace nav3d::collision
