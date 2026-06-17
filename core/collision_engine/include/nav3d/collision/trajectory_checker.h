#pragma once

#include <optional>

#include "nav3d/map/i_map.h"
#include "nav3d/planner/bspline/uniform_bspline.h"

namespace nav3d::collision {

struct TrajectoryCollisionResult {
  bool in_collision = false;
  std::optional<double> first_collision_time;
  std::optional<common::Point3D> first_collision_point;
};

class TrajectoryChecker {
public:
  explicit TrajectoryChecker(double sample_step_seconds);

  TrajectoryCollisionResult check(
    const map::IMap& map,
    const planner::UniformBspline& trajectory) const;

  TrajectoryCollisionResult checkRange(
    const map::IMap& map,
    const planner::UniformBspline& trajectory,
    double start_time,
    double end_time) const;

private:
  double sample_step_seconds_ = 0.05;
};

}  // namespace nav3d::collision
