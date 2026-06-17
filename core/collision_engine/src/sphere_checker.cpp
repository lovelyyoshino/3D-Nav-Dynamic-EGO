#include "nav3d/collision/sphere_checker.h"

#include <cmath>
#include <stdexcept>

namespace nav3d::collision {

SphereChecker::SphereChecker(double radius) : radius_(radius)
{
  if (!std::isfinite(radius_) || radius_ < 0.0) {
    throw std::invalid_argument("SphereChecker radius must be non-negative and finite");
  }
}

SphereCollisionResult SphereChecker::check(
  const map::IMap& map,
  const common::Point3D& center) const
{
  const double distance = map.isOccupied(center) ? 0.0 : map.getDistance(center);
  const double clearance = distance - radius_;
  return {
    clearance <= 0.0,
    distance,
    clearance,
  };
}

double SphereChecker::radius() const
{
  return radius_;
}

}  // namespace nav3d::collision
