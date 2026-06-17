#include "nav3d/collision/inflation_layer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav3d::collision {

InflationLayer::InflationLayer(const map::IMap& map, double radius)
  : map_(map), radius_(radius)
{
  if (!std::isfinite(radius_) || radius_ < 0.0) {
    throw std::invalid_argument("InflationLayer radius must be non-negative and finite");
  }
}

bool InflationLayer::isOccupied(const common::Point3D& p) const
{
  return map_.isOccupied(p) || map_.getDistance(p) <= radius_;
}

bool InflationLayer::isFree(const common::Point3D& p) const
{
  return isInBounds(p) && !isOccupied(p);
}

double InflationLayer::getDistance(const common::Point3D& p) const
{
  return std::max(0.0, map_.getDistance(p) - radius_);
}

bool InflationLayer::isInBounds(const common::Point3D& p) const
{
  return map_.isInBounds(p);
}

double InflationLayer::getResolution() const
{
  return map_.getResolution();
}

common::BoundingBox InflationLayer::getBounds() const
{
  return map_.getBounds();
}

double InflationLayer::radius() const
{
  return radius_;
}

}  // namespace nav3d::collision
