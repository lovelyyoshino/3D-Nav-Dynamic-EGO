#include "nav3d/planner/bspline/bspline_optimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace nav3d::planner {
namespace {

struct ReboundCue {
  std::size_t index = 0;
  common::Point3D target;
};

struct CostAndGradient {
  BsplineCostBreakdown cost;
  std::vector<common::Point3D> gradient;
};

double dot(const common::Point3D& a, const common::Point3D& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

double dot(const std::vector<double>& a, const std::vector<double>& b)
{
  double value = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    value += a[i] * b[i];
  }
  return value;
}

double norm(const std::vector<double>& values)
{
  return std::sqrt(dot(values, values));
}

void addScaled(common::Point3D& target, const common::Point3D& value, double scale)
{
  target.x += value.x * scale;
  target.y += value.y * scale;
  target.z += value.z * scale;
}

common::Point3D thirdDifference(
  const std::vector<common::Point3D>& points,
  std::size_t index)
{
  return points[index + 3] - points[index + 2] * 3.0 + points[index + 1] * 3.0 - points[index];
}

std::size_t firstEditableIndex(const std::vector<common::Point3D>& points)
{
  return points.size() > 6 ? 3u : 1u;
}

std::size_t lastEditableExclusive(const std::vector<common::Point3D>& points)
{
  return points.size() > 6 ? points.size() - 3u : points.size() - 1u;
}

bool isEditable(const std::vector<common::Point3D>& points, std::size_t index)
{
  return index >= firstEditableIndex(points) && index < lastEditableExclusive(points);
}

std::vector<double> packEditable(const std::vector<common::Point3D>& points)
{
  std::vector<double> values;
  for (std::size_t i = firstEditableIndex(points); i < lastEditableExclusive(points); ++i) {
    values.push_back(points[i].x);
    values.push_back(points[i].y);
    values.push_back(points[i].z);
  }
  return values;
}

void unpackEditable(const std::vector<double>& values, std::vector<common::Point3D>& points)
{
  std::size_t cursor = 0;
  for (std::size_t i = firstEditableIndex(points); i < lastEditableExclusive(points); ++i) {
    points[i].x = values[cursor++];
    points[i].y = values[cursor++];
    points[i].z = values[cursor++];
  }
}

std::vector<double> packGradient(
  const std::vector<common::Point3D>& points,
  const std::vector<common::Point3D>& gradient)
{
  std::vector<double> values;
  for (std::size_t i = firstEditableIndex(points); i < lastEditableExclusive(points); ++i) {
    values.push_back(gradient[i].x);
    values.push_back(gradient[i].y);
    values.push_back(gradient[i].z);
  }
  return values;
}

std::vector<double> scaled(const std::vector<double>& values, double scale)
{
  std::vector<double> result(values.size(), 0.0);
  for (std::size_t i = 0; i < values.size(); ++i) {
    result[i] = values[i] * scale;
  }
  return result;
}

std::vector<double> add(
  const std::vector<double>& a,
  const std::vector<double>& b,
  double b_scale = 1.0)
{
  std::vector<double> result(a.size(), 0.0);
  for (std::size_t i = 0; i < a.size(); ++i) {
    result[i] = a[i] + b[i] * b_scale;
  }
  return result;
}

CostAndGradient evaluate(
  const std::vector<common::Point3D>& points,
  const std::vector<common::Point3D>& warm_points,
  const std::vector<ReboundCue>& cues,
  const BsplineOptimizerConfig& config)
{
  CostAndGradient result;
  result.gradient.assign(points.size(), {});

  if (points.size() >= 4) {
    for (std::size_t i = 0; i + 3 < points.size(); ++i) {
      const auto d = thirdDifference(points, i);
      const double cost = dot(d, d);
      result.cost.smoothness += cost;
      addScaled(result.gradient[i], d, -2.0 * config.smoothness_weight);
      addScaled(result.gradient[i + 1], d, 6.0 * config.smoothness_weight);
      addScaled(result.gradient[i + 2], d, -6.0 * config.smoothness_weight);
      addScaled(result.gradient[i + 3], d, 2.0 * config.smoothness_weight);
    }
  }
  result.cost.smoothness *= config.smoothness_weight;

  for (const auto& cue : cues) {
    if (cue.index >= points.size()) {
      continue;
    }
    const auto delta = points[cue.index] - cue.target;
    result.cost.collision += dot(delta, delta);
    addScaled(result.gradient[cue.index], delta, 2.0 * config.collision_weight);
  }
  result.cost.collision *= config.collision_weight;

  if (!points.empty()) {
    const auto start_delta = points.front() - warm_points.front();
    const auto end_delta = points.back() - warm_points.back();
    result.cost.terminal = dot(start_delta, start_delta) + dot(end_delta, end_delta);
    addScaled(result.gradient.front(), start_delta, 2.0 * config.terminal_weight);
    addScaled(result.gradient.back(), end_delta, 2.0 * config.terminal_weight);
    result.cost.terminal *= config.terminal_weight;
  }

  for (std::size_t i = 0; i < points.size() && i < warm_points.size(); ++i) {
    if (!isEditable(points, i)) {
      continue;
    }
    const auto delta = points[i] - warm_points[i];
    result.cost.fitness += dot(delta, delta);
    addScaled(result.gradient[i], delta, 2.0 * config.fitness_weight);
  }
  result.cost.fitness *= config.fitness_weight;

  if (config.feasibility_weight > 0.0) {
    const double interval = config.interval;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const auto delta = points[i + 1] - points[i];
      const auto velocity = delta / interval;
      const double speed = common::norm(velocity);
      const double excess = speed - config.max_velocity;
      if (excess <= 0.0 || speed <= 1e-12) {
        continue;
      }

      result.cost.feasibility += excess * excess;
      const auto direction = velocity / speed;
      const double gradient_scale = 2.0 * config.feasibility_weight * excess / interval;
      addScaled(result.gradient[i], direction, -gradient_scale);
      addScaled(result.gradient[i + 1], direction, gradient_scale);
    }

    const double interval_squared = interval * interval;
    for (std::size_t i = 0; i + 2 < points.size(); ++i) {
      const auto second_difference = points[i + 2] - points[i + 1] * 2.0 + points[i];
      const auto acceleration = second_difference / interval_squared;
      const double magnitude = common::norm(acceleration);
      const double excess = magnitude - config.max_acceleration;
      if (excess <= 0.0 || magnitude <= 1e-12) {
        continue;
      }

      result.cost.feasibility += excess * excess;
      const auto direction = acceleration / magnitude;
      const double gradient_scale = 2.0 * config.feasibility_weight * excess / interval_squared;
      addScaled(result.gradient[i], direction, gradient_scale);
      addScaled(result.gradient[i + 1], direction, -2.0 * gradient_scale);
      addScaled(result.gradient[i + 2], direction, gradient_scale);
    }
  }
  result.cost.feasibility *= config.feasibility_weight;

  result.cost.total =
    result.cost.smoothness + result.cost.collision + result.cost.terminal + result.cost.fitness +
    result.cost.feasibility;
  return result;
}

void validateFeasibilityConfig(const BsplineOptimizerConfig& config)
{
  if (config.feasibility_weight < 0.0) {
    throw std::invalid_argument("feasibility_weight must be non-negative");
  }
  if (config.feasibility_weight == 0.0) {
    return;
  }
  if (config.interval <= 0.0) {
    throw std::invalid_argument("B-spline interval must be positive for feasibility cost");
  }
  if (config.max_velocity < 0.0) {
    throw std::invalid_argument("max_velocity must be non-negative");
  }
  if (config.max_acceleration < 0.0) {
    throw std::invalid_argument("max_acceleration must be non-negative");
  }
}

std::size_t nearestPathPointIndex(
  const std::vector<common::Point3D>& path,
  const common::Point3D& point)
{
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = common::distance(path[i], point);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

bool hasCueForIndex(const std::vector<ReboundCue>& cues, std::size_t index)
{
  return std::any_of(cues.begin(), cues.end(), [index](const ReboundCue& cue) {
    return cue.index == index;
  });
}

void appendLocalReboundCues(
  std::vector<ReboundCue>& cues,
  const std::vector<common::Point3D>& control_points,
  const map::IMap& map,
  const SearchOptions& options,
  std::size_t start_index,
  std::size_t goal_index)
{
  if (control_points.empty()) {
    return;
  }
  start_index = std::min(start_index, control_points.size() - 1);
  goal_index = std::min(goal_index, control_points.size() - 1);
  if (start_index >= goal_index) {
    return;
  }
  if (!map.isFree(control_points[start_index]) || !map.isFree(control_points[goal_index])) {
    return;
  }

  auto local_options = options;
  local_options.max_iterations = std::max(local_options.max_iterations, 2000);
  local_options.allow_diagonal = false;
  const auto search =
    AStar3D{}.search(map, control_points[start_index], control_points[goal_index], local_options);
  if (search.status != SearchStatus::Success || search.path.empty()) {
    return;
  }

  const std::size_t cue_begin = std::max(firstEditableIndex(control_points), start_index);
  const std::size_t cue_end = std::min(lastEditableExclusive(control_points), goal_index + 1);
  const std::size_t cue_count = cue_end > cue_begin ? cue_end - cue_begin : 1;
  for (std::size_t j = cue_begin; j < cue_end; ++j) {
    if (hasCueForIndex(cues, j)) {
      continue;
    }
    const std::size_t rank = j - cue_begin + 1;
    const double path_position =
      static_cast<double>(rank) * static_cast<double>(search.path.size() - 1) /
      static_cast<double>(cue_count + 1);
    const std::size_t path_index = std::min(
      search.path.size() - 1,
      static_cast<std::size_t>(std::lround(path_position)));
    cues.push_back({j, search.path[path_index]});
  }
}

std::pair<std::size_t, std::size_t> localWindowAround(
  const std::vector<common::Point3D>& control_points,
  const map::IMap& map,
  std::size_t center_index)
{
  if (control_points.empty()) {
    return {0, 0};
  }

  center_index = std::min(center_index, control_points.size() - 1);
  const std::size_t minimum_radius = std::min<std::size_t>(3, control_points.size() - 1);
  for (std::size_t radius = minimum_radius; radius < control_points.size(); ++radius) {
    const std::size_t start_index = center_index > radius ? center_index - radius : 0;
    const std::size_t goal_index = std::min(control_points.size() - 1, center_index + radius);
    if (start_index >= goal_index) {
      continue;
    }
    if (map.isFree(control_points[start_index]) && map.isFree(control_points[goal_index])) {
      return {start_index, goal_index};
    }
  }

  return {0, 0};
}

std::vector<ReboundCue> buildReboundCues(
  const std::vector<common::Point3D>& control_points,
  const map::IMap& map,
  const SearchOptions& options,
  double interval)
{
  std::vector<ReboundCue> cues;

  for (std::size_t i = firstEditableIndex(control_points); i < lastEditableExclusive(control_points); ++i) {
    if (!map.isOccupied(control_points[i])) {
      continue;
    }

    std::size_t start_index = i;
    while (start_index > 0 && !map.isFree(control_points[start_index])) {
      --start_index;
    }
    std::size_t goal_index = i;
    while (goal_index + 1 < control_points.size() && !map.isFree(control_points[goal_index])) {
      ++goal_index;
    }
    if (!map.isFree(control_points[start_index]) || !map.isFree(control_points[goal_index])) {
      continue;
    }

    appendLocalReboundCues(cues, control_points, map, options, start_index, goal_index);
  }

  if (control_points.size() >= 4 && interval > 0.0) {
    const UniformBspline warm_curve(control_points, 3, interval);
    const double sample_step = std::max(1e-3, interval * 0.25);
    for (double t = 0.0; t <= warm_curve.duration(); t += sample_step) {
      const auto sample = warm_curve.evaluate(t);
      if (!map.isOccupied(sample)) {
        continue;
      }
      const auto nearest_index = nearestPathPointIndex(control_points, sample);
      const auto [start_index, goal_index] = localWindowAround(control_points, map, nearest_index);
      if (start_index == goal_index) {
        continue;
      }
      appendLocalReboundCues(cues, control_points, map, options, start_index, goal_index);
    }

    const auto end_sample = warm_curve.evaluate(warm_curve.duration());
    if (map.isOccupied(end_sample)) {
      const auto nearest_index = nearestPathPointIndex(control_points, end_sample);
      const auto [start_index, goal_index] = localWindowAround(control_points, map, nearest_index);
      if (start_index != goal_index) {
        appendLocalReboundCues(cues, control_points, map, options, start_index, goal_index);
      }
    }
  }

  return cues;
}

bool hasOccupiedEditableControlPoint(
  const std::vector<common::Point3D>& control_points,
  const map::IMap& map)
{
  for (std::size_t i = firstEditableIndex(control_points); i < lastEditableExclusive(control_points); ++i) {
    if (map.isOccupied(control_points[i])) {
      return true;
    }
  }
  return false;
}

std::vector<double> lbfgsDirection(
  const std::vector<double>& gradient,
  const std::vector<std::vector<double>>& s_history,
  const std::vector<std::vector<double>>& y_history)
{
  if (s_history.empty()) {
    return scaled(gradient, -1.0);
  }

  std::vector<double> q = gradient;
  std::vector<double> alpha(s_history.size(), 0.0);
  std::vector<double> rho(s_history.size(), 0.0);
  for (int i = static_cast<int>(s_history.size()) - 1; i >= 0; --i) {
    const double sy = dot(s_history[static_cast<std::size_t>(i)], y_history[static_cast<std::size_t>(i)]);
    rho[static_cast<std::size_t>(i)] = sy > 0.0 ? 1.0 / sy : 0.0;
    alpha[static_cast<std::size_t>(i)] = rho[static_cast<std::size_t>(i)] * dot(s_history[static_cast<std::size_t>(i)], q);
    q = add(q, y_history[static_cast<std::size_t>(i)], -alpha[static_cast<std::size_t>(i)]);
  }

  const auto& last_s = s_history.back();
  const auto& last_y = y_history.back();
  const double yy = dot(last_y, last_y);
  const double gamma = yy > 0.0 ? dot(last_s, last_y) / yy : 1.0;
  std::vector<double> r = scaled(q, gamma);

  for (std::size_t i = 0; i < s_history.size(); ++i) {
    const double beta = rho[i] * dot(y_history[i], r);
    r = add(r, s_history[i], alpha[i] - beta);
  }

  return scaled(r, -1.0);
}

}  // namespace

UniformBspline BsplineOptimizer::makeWarmStart(
  const std::vector<common::Point3D>& path,
  double interval)
{
  return UniformBspline::fitThroughWaypoints(path, interval);
}

BsplineCostBreakdown BsplineOptimizer::evaluateCost(
  const std::vector<common::Point3D>& control_points,
  const BsplineOptimizerConfig& config)
{
  validateFeasibilityConfig(config);
  return evaluate(control_points, control_points, {}, config).cost;
}

BsplineOptimizeResult BsplineOptimizer::optimize(
  const std::vector<common::Point3D>& warm_start_path,
  const map::IMap& map,
  const BsplineOptimizerConfig& config)
{
  if (config.max_iterations <= 0 || config.memory_size <= 0) {
    throw std::invalid_argument("BsplineOptimizer iterations and memory size must be positive");
  }
  validateFeasibilityConfig(config);

  BsplineOptimizeResult result;
  result.warm_start = makeWarmStart(warm_start_path, config.interval);
  std::vector<common::Point3D> points = result.warm_start.controlPoints();
  const std::vector<common::Point3D> warm_points = points;
  const auto cues = buildReboundCues(points, map, config.rebound_search, config.interval);
  result.used_rebound = !cues.empty();
  result.rebound_segments = static_cast<int>(cues.size());

  auto current = evaluate(points, warm_points, cues, config);
  result.initial_cost = current.cost.total;
  if (hasOccupiedEditableControlPoint(points, map) && cues.empty()) {
    result.spline = result.warm_start;
    result.final_cost = result.initial_cost;
    result.success = false;
    return result;
  }

  std::vector<double> x = packEditable(points);
  std::vector<double> gradient = packGradient(points, current.gradient);
  std::vector<std::vector<double>> s_history;
  std::vector<std::vector<double>> y_history;

  for (int iteration = 0; iteration < config.max_iterations && !x.empty(); ++iteration) {
    result.iterations = iteration + 1;
    if (norm(gradient) < config.gradient_tolerance) {
      break;
    }

    std::vector<double> direction = lbfgsDirection(gradient, s_history, y_history);
    if (dot(direction, gradient) >= 0.0) {
      direction = scaled(gradient, -1.0);
    }

    const double directional_derivative = dot(gradient, direction);
    double step = config.initial_step;
    bool accepted = false;
    std::vector<double> next_x;
    std::vector<common::Point3D> next_points;
    CostAndGradient next;

    for (int line_search = 0; line_search < 20; ++line_search) {
      next_x = add(x, direction, step);
      next_points = points;
      unpackEditable(next_x, next_points);
      next = evaluate(next_points, warm_points, cues, config);
      if (next.cost.total <= current.cost.total + 1e-4 * step * directional_derivative) {
        accepted = true;
        break;
      }
      step *= 0.5;
    }

    if (!accepted) {
      break;
    }

    const std::vector<double> next_gradient = packGradient(next_points, next.gradient);
    const std::vector<double> s = add(next_x, x, -1.0);
    const std::vector<double> y = add(next_gradient, gradient, -1.0);
    if (dot(s, y) > 1e-10) {
      s_history.push_back(s);
      y_history.push_back(y);
      if (s_history.size() > static_cast<std::size_t>(config.memory_size)) {
        s_history.erase(s_history.begin());
        y_history.erase(y_history.begin());
      }
    }

    x = std::move(next_x);
    points = std::move(next_points);
    current = std::move(next);
    gradient = next_gradient;
  }

  result.spline = UniformBspline(points, result.warm_start.degree(), config.interval);
  result.final_cost = current.cost.total;
  result.success = std::isfinite(result.final_cost) && result.final_cost <= result.initial_cost;
  return result;
}

}  // namespace nav3d::planner
