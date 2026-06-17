#include "nav3d/map/map_composite.h"

#include <algorithm>
#include <limits>

namespace nav3d::map {

MapComposite::MapComposite(const IMap& global_map, const LocalGrid& local_grid)
  : global_map_(global_map), local_grid_(local_grid)
{
}

bool MapComposite::isOccupied(const common::Point3D& p) const
{
  if (local_grid_.hasObservation(p)) {
    return local_grid_.isOccupied(p);
  }
  return global_map_.isOccupied(p);
}

bool MapComposite::isFree(const common::Point3D& p) const
{
  if (!isInBounds(p)) {
    return false;
  }
  if (local_grid_.hasObservation(p)) {
    return local_grid_.isFree(p);
  }
  return global_map_.isFree(p);
}

double MapComposite::getDistance(const common::Point3D& p) const
{
  if (local_grid_.hasObservation(p)) {
    if (local_grid_.isOccupied(p)) {
      return 0.0;
    }
    return local_grid_.getDistance(p);
  }

  if (local_grid_.hasOccupiedCells() && local_grid_.isInBounds(p)) {
    return std::min(local_grid_.getDistance(p), global_map_.getDistance(p));
  }
  return global_map_.getDistance(p);
}

bool MapComposite::isInBounds(const common::Point3D& p) const
{
  return local_grid_.isInBounds(p) || global_map_.isInBounds(p);
}

double MapComposite::getResolution() const
{
  return std::min(global_map_.getResolution(), local_grid_.getResolution());
}

common::BoundingBox MapComposite::getBounds() const
{
  const auto global_bounds = global_map_.getBounds();
  const auto local_bounds = local_grid_.getBounds();
  if (!global_bounds.valid) {
    return local_bounds;
  }
  if (!local_bounds.valid) {
    return global_bounds;
  }

  common::BoundingBox bounds;
  bounds.valid = true;
  bounds.min = {
    std::min(global_bounds.min.x, local_bounds.min.x),
    std::min(global_bounds.min.y, local_bounds.min.y),
    std::min(global_bounds.min.z, local_bounds.min.z),
  };
  bounds.max = {
    std::max(global_bounds.max.x, local_bounds.max.x),
    std::max(global_bounds.max.y, local_bounds.max.y),
    std::max(global_bounds.max.z, local_bounds.max.z),
  };
  return bounds;
}

}  // namespace nav3d::map
