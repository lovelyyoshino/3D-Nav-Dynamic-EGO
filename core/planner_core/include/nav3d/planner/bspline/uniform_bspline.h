#pragma once

#include <vector>

#include "nav3d/common/types.h"

namespace nav3d::planner {

class UniformBspline {
public:
  UniformBspline() = default;
  UniformBspline(std::vector<common::Point3D> control_points, int degree, double interval);

  static UniformBspline fitThroughWaypoints(
    const std::vector<common::Point3D>& waypoints,
    double interval);

  common::Point3D evaluate(double t) const;
  double duration() const;
  double interval() const;
  UniformBspline rescaleTime(double scale) const;
  int degree() const;
  const std::vector<common::Point3D>& controlPoints() const;
  std::vector<double> knots() const;

private:
  std::vector<common::Point3D> control_points_;
  int degree_ = 3;
  double interval_ = 1.0;
};

}  // namespace nav3d::planner
