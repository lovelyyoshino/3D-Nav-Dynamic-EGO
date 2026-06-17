#include "nav3d/planner/bspline/uniform_bspline.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav3d::planner {
namespace {

double basis(int degree, int i, double u, const std::vector<double>& knots)
{
  if (degree == 0) {
    const bool in_span = knots[i] <= u && u < knots[i + 1];
    const bool at_end = u == knots.back() && i + 1 == static_cast<int>(knots.size()) - 1;
    return (in_span || at_end) ? 1.0 : 0.0;
  }

  double value = 0.0;
  const double left_den = knots[i + degree] - knots[i];
  if (left_den > 0.0) {
    value += (u - knots[i]) / left_den * basis(degree - 1, i, u, knots);
  }
  const double right_den = knots[i + degree + 1] - knots[i + 1];
  if (right_den > 0.0) {
    value += (knots[i + degree + 1] - u) / right_den * basis(degree - 1, i + 1, u, knots);
  }
  return value;
}

std::vector<double> makeOpenUniformKnots(int control_count, int degree)
{
  const int knot_count = control_count + degree + 1;
  std::vector<double> knots(static_cast<std::size_t>(knot_count), 0.0);
  const int interior_count = control_count - degree - 1;
  for (int i = 0; i < knot_count; ++i) {
    if (i <= degree) {
      knots[i] = 0.0;
    } else if (i >= control_count) {
      knots[i] = static_cast<double>(std::max(1, interior_count + 1));
    } else {
      knots[i] = static_cast<double>(i - degree);
    }
  }
  return knots;
}

}  // namespace

UniformBspline::UniformBspline(
  std::vector<common::Point3D> control_points,
  int degree,
  double interval)
  : control_points_(std::move(control_points)), degree_(degree), interval_(interval)
{
  if (interval_ <= 0.0) {
    throw std::invalid_argument("B-spline interval must be positive");
  }
  if (degree_ < 1) {
    throw std::invalid_argument("B-spline degree must be at least 1");
  }
  if (control_points_.size() < static_cast<std::size_t>(degree_ + 1)) {
    throw std::invalid_argument("not enough control points for requested B-spline degree");
  }
}

UniformBspline UniformBspline::fitThroughWaypoints(
  const std::vector<common::Point3D>& waypoints,
  double interval)
{
  if (waypoints.size() < 2) {
    throw std::invalid_argument("at least two waypoints are required");
  }

  std::vector<common::Point3D> control_points;
  control_points.reserve(waypoints.size() + 4);
  control_points.push_back(waypoints.front());
  control_points.push_back(waypoints.front());
  for (const auto& p : waypoints) {
    control_points.push_back(p);
  }
  control_points.push_back(waypoints.back());
  control_points.push_back(waypoints.back());

  return UniformBspline(control_points, 3, interval);
}

common::Point3D UniformBspline::evaluate(double t) const
{
  const double total = duration();
  if (t <= 0.0) {
    return control_points_.front();
  }
  if (t >= total) {
    return control_points_.back();
  }

  const auto knots = makeOpenUniformKnots(static_cast<int>(control_points_.size()), degree_);
  const double u = std::clamp(t / interval_, 0.0, knots.back());
  common::Point3D result;
  for (int i = 0; i < static_cast<int>(control_points_.size()); ++i) {
    const double b = basis(degree_, i, u, knots);
    result = result + control_points_[static_cast<std::size_t>(i)] * b;
  }
  return result;
}

double UniformBspline::duration() const
{
  const int interior_count = static_cast<int>(control_points_.size()) - degree_ - 1;
  return static_cast<double>(std::max(1, interior_count + 1)) * interval_;
}

double UniformBspline::interval() const
{
  return interval_;
}

UniformBspline UniformBspline::rescaleTime(double scale) const
{
  if (scale <= 0.0) {
    throw std::invalid_argument("B-spline time scale must be positive");
  }
  return UniformBspline(control_points_, degree_, interval_ * scale);
}

int UniformBspline::degree() const
{
  return degree_;
}

const std::vector<common::Point3D>& UniformBspline::controlPoints() const
{
  return control_points_;
}

std::vector<double> UniformBspline::knots() const
{
  auto knots = makeOpenUniformKnots(static_cast<int>(control_points_.size()), degree_);
  for (auto& knot : knots) {
    knot *= interval_;
  }
  return knots;
}

}  // namespace nav3d::planner
