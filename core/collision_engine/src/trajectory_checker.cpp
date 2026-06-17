#include "nav3d/collision/trajectory_checker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace nav3d::collision {
namespace {

TrajectoryCollisionResult collisionAt(
  const map::IMap& map,
  const planner::UniformBspline& trajectory,
  double time)
{
  const auto point = trajectory.evaluate(time);
  if (!map.isFree(point)) {
    return {true, time, point};
  }
  return {};
}

double spatialSampleStep(const map::IMap& map)
{
  const double resolution = map.getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(1e-3, resolution * 0.5);
}

TrajectoryCollisionResult checkInterpolatedRange(
  const map::IMap& map,
  const planner::UniformBspline& trajectory,
  double start_time,
  double end_time,
  const common::Point3D& start_point,
  const common::Point3D& end_point)
{
  const double step = spatialSampleStep(map);
  const double segment_distance = common::distance(start_point, end_point);
  if (!std::isfinite(step) || segment_distance <= step) {
    return {};
  }

  const int subdivisions = std::max(1, static_cast<int>(std::ceil(segment_distance / step)));
  for (int i = 1; i < subdivisions; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(subdivisions);
    const double time = start_time + (end_time - start_time) * ratio;
    const auto collision = collisionAt(map, trajectory, time);
    if (collision.in_collision) {
      return collision;
    }
  }
  return {};
}

}  // namespace

TrajectoryChecker::TrajectoryChecker(double sample_step_seconds)
  : sample_step_seconds_(sample_step_seconds)
{
  if (sample_step_seconds_ <= 0.0) {
    throw std::invalid_argument("trajectory sample step must be positive");
  }
}

TrajectoryCollisionResult TrajectoryChecker::check(
  const map::IMap& map,
  const planner::UniformBspline& trajectory) const
{
  return checkRange(map, trajectory, 0.0, trajectory.duration());
}

TrajectoryCollisionResult TrajectoryChecker::checkRange(
  const map::IMap& map,
  const planner::UniformBspline& trajectory,
  double start_time,
  double end_time) const
{
  const double duration = trajectory.duration();
  const double start = std::clamp(start_time, 0.0, duration);
  const double end = std::clamp(end_time, 0.0, duration);
  if (start > end) {
    return {};
  }

  auto previous_time = start;
  auto previous_point = trajectory.evaluate(previous_time);
  if (!map.isFree(previous_point)) {
    return {true, previous_time, previous_point};
  }

  for (double t = start + sample_step_seconds_; t <= end; t += sample_step_seconds_) {
    const auto point = trajectory.evaluate(t);
    const auto interpolated_collision = checkInterpolatedRange(
      map,
      trajectory,
      previous_time,
      t,
      previous_point,
      point);
    if (interpolated_collision.in_collision) {
      return interpolated_collision;
    }
    if (!map.isFree(point)) {
      return {true, t, point};
    }
    previous_time = t;
    previous_point = point;
  }

  const auto end_point = trajectory.evaluate(end);
  const auto interpolated_collision = checkInterpolatedRange(
    map,
    trajectory,
    previous_time,
    end,
    previous_point,
    end_point);
  if (interpolated_collision.in_collision) {
    return interpolated_collision;
  }
  if (!map.isFree(end_point)) {
    return {true, end, end_point};
  }

  return {};
}

}  // namespace nav3d::collision
