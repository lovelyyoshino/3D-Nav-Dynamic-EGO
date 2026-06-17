#include "nav3d/collision/trajectory_feasibility_checker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nav3d::collision {
namespace {

constexpr double kRootEpsilon = 1e-9;

bool violates(double observed, double limit)
{
  return std::isfinite(limit) && observed > limit;
}

double scaleForLimit(double observed, double limit, bool squared_time_scale)
{
  if (!std::isfinite(limit) || observed <= limit) {
    return 1.0;
  }
  if (limit <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double ratio = observed / limit;
  return squared_time_scale ? std::sqrt(ratio) : ratio;
}

void validateLimits(const DynamicLimits& limits)
{
  if (limits.max_velocity < 0.0) {
    throw std::invalid_argument("max_velocity must be non-negative");
  }
  if (limits.max_acceleration < 0.0) {
    throw std::invalid_argument("max_acceleration must be non-negative");
  }
}

double basis(int degree, int i, double t, const std::vector<double>& knots)
{
  if (degree == 0) {
    const bool in_span = knots[static_cast<std::size_t>(i)] <= t &&
      t < knots[static_cast<std::size_t>(i + 1)];
    const bool at_end =
      t == knots.back() && i + 1 == static_cast<int>(knots.size()) - 1;
    return (in_span || at_end) ? 1.0 : 0.0;
  }

  double value = 0.0;
  const double left_den = knots[static_cast<std::size_t>(i + degree)] -
    knots[static_cast<std::size_t>(i)];
  if (left_den > 0.0) {
    value += (t - knots[static_cast<std::size_t>(i)]) / left_den *
      basis(degree - 1, i, t, knots);
  }

  const double right_den = knots[static_cast<std::size_t>(i + degree + 1)] -
    knots[static_cast<std::size_t>(i + 1)];
  if (right_den > 0.0) {
    value += (knots[static_cast<std::size_t>(i + degree + 1)] - t) / right_den *
      basis(degree - 1, i + 1, t, knots);
  }
  return value;
}

std::vector<common::Point3D> derivativeControlPoints(
  const std::vector<common::Point3D>& control_points,
  const std::vector<double>& knots,
  int degree)
{
  if (degree <= 0 || control_points.size() < 2) {
    return {};
  }

  std::vector<common::Point3D> derivatives;
  derivatives.reserve(control_points.size() - 1);
  for (std::size_t i = 0; i + 1 < control_points.size(); ++i) {
    const double denominator = knots[i + static_cast<std::size_t>(degree) + 1] - knots[i + 1];
    if (denominator <= 0.0) {
      derivatives.push_back({});
      continue;
    }
    derivatives.push_back((control_points[i + 1] - control_points[i]) *
      (static_cast<double>(degree) / denominator));
  }
  return derivatives;
}

common::Point3D evaluateSplineVector(
  const std::vector<common::Point3D>& control_points,
  const std::vector<double>& knots,
  int degree,
  double t)
{
  if (control_points.empty()) {
    return {};
  }
  if (t <= knots.front() + kRootEpsilon) {
    return control_points.front();
  }
  if (t >= knots.back() - kRootEpsilon) {
    return control_points.back();
  }

  common::Point3D value;
  for (int i = 0; i < static_cast<int>(control_points.size()); ++i) {
    value = value + control_points[static_cast<std::size_t>(i)] * basis(degree, i, t, knots);
  }
  return value;
}

double squaredNormAt(
  const std::vector<common::Point3D>& control_points,
  const std::vector<double>& knots,
  int degree,
  double t)
{
  const auto value = evaluateSplineVector(control_points, knots, degree, t);
  return value.x * value.x + value.y * value.y + value.z * value.z;
}

std::vector<double> solveQuadratic(double a, double b, double c)
{
  if (std::abs(a) < kRootEpsilon) {
    if (std::abs(b) < kRootEpsilon) {
      return {};
    }
    return {-c / b};
  }

  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < -kRootEpsilon) {
    return {};
  }
  if (std::abs(discriminant) <= kRootEpsilon) {
    return {-b / (2.0 * a)};
  }

  const double root = std::sqrt(discriminant);
  return {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)};
}

std::vector<double> solveCubicByBisection(double a, double b, double c, double d, double min_t, double max_t)
{
  if (std::abs(a) < kRootEpsilon) {
    return solveQuadratic(b, c, d);
  }

  std::vector<double> split_points{min_t, max_t};
  for (const double root : solveQuadratic(3.0 * a, 2.0 * b, c)) {
    if (root > min_t + kRootEpsilon && root < max_t - kRootEpsilon) {
      split_points.push_back(root);
    }
  }
  std::sort(split_points.begin(), split_points.end());

  auto evaluate = [&](double t) {
    return ((a * t + b) * t + c) * t + d;
  };

  std::vector<double> roots;
  for (std::size_t i = 1; i < split_points.size(); ++i) {
    double left = split_points[i - 1];
    double right = split_points[i];
    double f_left = evaluate(left);
    double f_right = evaluate(right);

    if (std::abs(f_left) <= kRootEpsilon) {
      roots.push_back(left);
      continue;
    }
    if (std::abs(f_right) <= kRootEpsilon) {
      roots.push_back(right);
      continue;
    }
    if ((f_left < 0.0) == (f_right < 0.0)) {
      continue;
    }

    for (int iteration = 0; iteration < 80; ++iteration) {
      const double mid = 0.5 * (left + right);
      const double f_mid = evaluate(mid);
      if (std::abs(f_mid) <= kRootEpsilon) {
        left = right = mid;
        break;
      }
      if ((f_left < 0.0) == (f_mid < 0.0)) {
        left = mid;
        f_left = f_mid;
      } else {
        right = mid;
        f_right = f_mid;
      }
    }
    roots.push_back(0.5 * (left + right));
  }
  return roots;
}

std::vector<double> fitDerivativeRootsOnSpan(
  const std::vector<common::Point3D>& control_points,
  const std::vector<double>& knots,
  int degree,
  double start,
  double end)
{
  if (degree == 1) {
    const double f0 = squaredNormAt(control_points, knots, degree, start);
    const double f1 = squaredNormAt(control_points, knots, degree, end);
    const double mid = 0.5 * (start + end);
    const double fm = squaredNormAt(control_points, knots, degree, mid);
    const double derivative = f1 - f0;
    if (std::abs(derivative) <= kRootEpsilon && fm > f0 + kRootEpsilon) {
      return {mid};
    }
    return {};
  }

  if (degree != 2) {
    return {};
  }

  const double span = end - start;
  if (span <= 0.0) {
    return {};
  }

  const auto c = evaluateSplineVector(control_points, knots, degree, start);
  const auto mid = evaluateSplineVector(control_points, knots, degree, start + span * 0.5);
  const auto end_value = evaluateSplineVector(control_points, knots, degree, end);
  const auto a = (end_value + c - (mid * 2.0)) * (2.0 / (span * span));
  const auto b = (end_value - c) / span - (a * span);

  const double cubic = 4.0 * (a.x * a.x + a.y * a.y + a.z * a.z);
  const double quadratic = 6.0 * (a.x * b.x + a.y * b.y + a.z * b.z);
  const double linear =
    2.0 * (b.x * b.x + b.y * b.y + b.z * b.z +
           2.0 * (a.x * c.x + a.y * c.y + a.z * c.z));
  const double constant = 2.0 * (b.x * c.x + b.y * c.y + b.z * c.z);

  std::vector<double> roots;
  for (const double local_root : solveCubicByBisection(cubic, quadratic, linear, constant, 0.0, span)) {
    if (local_root > kRootEpsilon && local_root < span - kRootEpsilon) {
      roots.push_back(start + local_root);
    }
  }
  return roots;
}

double maxSplineVectorNorm(
  const std::vector<common::Point3D>& control_points,
  const std::vector<double>& knots,
  int degree)
{
  if (control_points.empty() || degree < 0) {
    return 0.0;
  }

  double max_squared = 0.0;
  const int last_span = static_cast<int>(knots.size()) - 1;
  for (int span = degree; span < last_span - degree; ++span) {
    const double start = knots[static_cast<std::size_t>(span)];
    const double end = knots[static_cast<std::size_t>(span + 1)];
    if (end <= start) {
      continue;
    }

    std::vector<double> candidates{start, end};
    const auto roots = fitDerivativeRootsOnSpan(control_points, knots, degree, start, end);
    candidates.insert(candidates.end(), roots.begin(), roots.end());

    for (const double t : candidates) {
      max_squared = std::max(max_squared, squaredNormAt(control_points, knots, degree, t));
    }
  }
  return std::sqrt(max_squared);
}

}  // namespace

TrajectoryFeasibilityChecker::TrajectoryFeasibilityChecker(double sample_step_seconds)
  : sample_step_seconds_(sample_step_seconds)
{
  if (sample_step_seconds_ <= 0.0) {
    throw std::invalid_argument("feasibility sample step must be positive");
  }
}

TrajectoryFeasibilityResult TrajectoryFeasibilityChecker::check(
  const planner::UniformBspline& trajectory,
  const DynamicLimits& limits) const
{
  validateLimits(limits);

  std::vector<std::pair<double, common::Point3D>> samples;
  const double end = trajectory.duration();
  for (double t = 0.0; t < end; t += sample_step_seconds_) {
    samples.push_back({t, trajectory.evaluate(t)});
  }
  samples.push_back({end, trajectory.evaluate(end)});

  TrajectoryFeasibilityResult result;
  std::optional<common::Point3D> previous_velocity;
  double previous_velocity_time = 0.0;

  for (std::size_t i = 1; i < samples.size(); ++i) {
    const double dt = samples[i].first - samples[i - 1].first;
    if (dt <= 0.0) {
      continue;
    }

    const auto velocity = (samples[i].second - samples[i - 1].second) / dt;
    const double speed = common::norm(velocity);
    result.max_velocity_observed = std::max(result.max_velocity_observed, speed);
    if (violates(speed, limits.max_velocity)) {
      result.feasible = false;
      result.velocity_violation = true;
      if (!result.first_violation_time.has_value()) {
        result.first_violation_time = samples[i].first;
      }
    }

    if (previous_velocity.has_value()) {
      const double acceleration_dt = samples[i].first - previous_velocity_time;
      if (acceleration_dt > 0.0) {
        const auto acceleration = (velocity - *previous_velocity) / acceleration_dt;
        const double acceleration_norm = common::norm(acceleration);
        result.max_acceleration_observed =
          std::max(result.max_acceleration_observed, acceleration_norm);
        if (violates(acceleration_norm, limits.max_acceleration)) {
          result.feasible = false;
          result.acceleration_violation = true;
          if (!result.first_violation_time.has_value()) {
            result.first_violation_time = samples[i].first;
          }
        }
      }
    }

    previous_velocity = velocity;
    previous_velocity_time = samples[i].first;
  }

  result.required_time_scale = std::max(
    scaleForLimit(result.max_velocity_observed, limits.max_velocity, false),
    scaleForLimit(result.max_acceleration_observed, limits.max_acceleration, true));
  return result;
}

TrajectoryAnalyticBounds TrajectoryFeasibilityChecker::computeAnalyticBounds(
  const planner::UniformBspline& trajectory) const
{
  const auto& control_points = trajectory.controlPoints();
  const int degree = trajectory.degree();
  const double interval = trajectory.interval();

  TrajectoryAnalyticBounds bounds;
  if (control_points.size() < 2 || degree < 1) {
    return bounds;
  }

  const double velocity_scale = static_cast<double>(degree) / interval;
  for (std::size_t i = 0; i + 1 < control_points.size(); ++i) {
    const auto velocity_control = (control_points[i + 1] - control_points[i]) * velocity_scale;
    bounds.max_velocity_bound =
      std::max(bounds.max_velocity_bound, common::norm(velocity_control));
  }

  if (control_points.size() < 3 || degree < 2) {
    return bounds;
  }

  const double acceleration_scale =
    static_cast<double>(degree * (degree - 1)) / (interval * interval);
  for (std::size_t i = 0; i + 2 < control_points.size(); ++i) {
    const auto second_difference = control_points[i + 2] - (control_points[i + 1] * 2.0) +
      control_points[i];
    const auto acceleration_control = second_difference * acceleration_scale;
    bounds.max_acceleration_bound =
      std::max(bounds.max_acceleration_bound, common::norm(acceleration_control));
  }

  return bounds;
}

TrajectoryAnalyticExtrema TrajectoryFeasibilityChecker::computeAnalyticExtrema(
  const planner::UniformBspline& trajectory) const
{
  const auto& control_points = trajectory.controlPoints();
  const int degree = trajectory.degree();

  TrajectoryAnalyticExtrema extrema;
  if (control_points.size() < 2 || degree < 1) {
    return extrema;
  }

  const auto knots = trajectory.knots();
  const auto velocity_control = derivativeControlPoints(control_points, knots, degree);
  if (!velocity_control.empty()) {
    std::vector<double> velocity_knots(knots.begin() + 1, knots.end() - 1);
    extrema.max_velocity = maxSplineVectorNorm(velocity_control, velocity_knots, degree - 1);

    if (velocity_control.size() >= 2 && degree >= 2) {
      const auto acceleration_control =
        derivativeControlPoints(velocity_control, velocity_knots, degree - 1);
      if (!acceleration_control.empty()) {
        std::vector<double> acceleration_knots(
          velocity_knots.begin() + 1,
          velocity_knots.end() - 1);
        extrema.max_acceleration =
          maxSplineVectorNorm(acceleration_control, acceleration_knots, degree - 2);
      }
    }
  }

  return extrema;
}

TrajectoryFeasibilityResult TrajectoryFeasibilityChecker::checkAnalyticBounds(
  const planner::UniformBspline& trajectory,
  const DynamicLimits& limits) const
{
  validateLimits(limits);

  const auto bounds = computeAnalyticBounds(trajectory);

  TrajectoryFeasibilityResult result;
  result.max_velocity_observed = bounds.max_velocity_bound;
  result.max_acceleration_observed = bounds.max_acceleration_bound;
  result.velocity_violation = violates(bounds.max_velocity_bound, limits.max_velocity);
  result.acceleration_violation = violates(bounds.max_acceleration_bound, limits.max_acceleration);
  result.feasible = !result.velocity_violation && !result.acceleration_violation;
  result.required_time_scale = std::max(
    scaleForLimit(bounds.max_velocity_bound, limits.max_velocity, false),
    scaleForLimit(bounds.max_acceleration_bound, limits.max_acceleration, true));
  return result;
}

TrajectoryFeasibilityResult TrajectoryFeasibilityChecker::checkAnalyticExact(
  const planner::UniformBspline& trajectory,
  const DynamicLimits& limits) const
{
  validateLimits(limits);

  const auto extrema = computeAnalyticExtrema(trajectory);

  TrajectoryFeasibilityResult result;
  result.max_velocity_observed = extrema.max_velocity;
  result.max_acceleration_observed = extrema.max_acceleration;
  result.velocity_violation = violates(extrema.max_velocity, limits.max_velocity);
  result.acceleration_violation = violates(extrema.max_acceleration, limits.max_acceleration);
  result.feasible = !result.velocity_violation && !result.acceleration_violation;
  result.required_time_scale = std::max(
    scaleForLimit(extrema.max_velocity, limits.max_velocity, false),
    scaleForLimit(extrema.max_acceleration, limits.max_acceleration, true));
  return result;
}

}  // namespace nav3d::collision
