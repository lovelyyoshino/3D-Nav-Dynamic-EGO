#include "nav3d/map/voxel_grid_map.h"

#include <cmath>
#include <stdexcept>

namespace nav3d::map {

std::size_t GridIndexHash::operator()(const GridIndex& idx) const
{
  const std::size_t h1 = std::hash<int>{}(idx.x);
  const std::size_t h2 = std::hash<int>{}(idx.y);
  const std::size_t h3 = std::hash<int>{}(idx.z);
  return h1 ^ (h2 << 1) ^ (h3 << 2);
}

VoxelGridMap::VoxelGridMap(double resolution) : resolution_(resolution)
{
  if (resolution_ <= 0.0) {
    throw std::invalid_argument("VoxelGridMap resolution must be positive");
  }
}

VoxelGridMap VoxelGridMap::fromPointCloud(const PointCloud& cloud, double resolution)
{
  VoxelGridMap map(resolution);
  for (const auto& point : cloud.points) {
    map.insertOccupied(point);
  }
  return map;
}

void VoxelGridMap::insertOccupied(const common::Point3D& p)
{
  occupied_.insert(worldToGrid(p));
  bounds_.expandToInclude(gridToWorld(worldToGrid(p)));
}

void VoxelGridMap::setExplicitBounds(const common::Point3D& min, const common::Point3D& max)
{
  bounds_.min = min;
  bounds_.max = max;
  bounds_.valid = true;
  has_explicit_bounds_ = true;
}

GridIndex VoxelGridMap::worldToGrid(const common::Point3D& p) const
{
  return {
    static_cast<int>(std::floor(p.x / resolution_ + 1e-9)),
    static_cast<int>(std::floor(p.y / resolution_ + 1e-9)),
    static_cast<int>(std::floor(p.z / resolution_ + 1e-9)),
  };
}

common::Point3D VoxelGridMap::gridToWorld(const GridIndex& idx) const
{
  return {
    static_cast<double>(idx.x) * resolution_,
    static_cast<double>(idx.y) * resolution_,
    static_cast<double>(idx.z) * resolution_,
  };
}

bool VoxelGridMap::isOccupied(const common::Point3D& p) const
{
  return occupied_.find(worldToGrid(p)) != occupied_.end();
}

bool VoxelGridMap::isFree(const common::Point3D& p) const
{
  return isInBounds(p) && !isOccupied(p);
}

double VoxelGridMap::getDistance(const common::Point3D& p) const
{
  if (occupied_.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double best = std::numeric_limits<double>::infinity();
  for (const auto& cell : occupied_) {
    best = std::min(best, common::distance(p, gridToWorld(cell)));
  }
  return best;
}

bool VoxelGridMap::isInBounds(const common::Point3D& p) const
{
  if (!has_explicit_bounds_) {
    return true;
  }
  const double eps = 1e-9;
  return p.x >= bounds_.min.x - eps && p.x <= bounds_.max.x + eps &&
         p.y >= bounds_.min.y - eps && p.y <= bounds_.max.y + eps &&
         p.z >= bounds_.min.z - eps && p.z <= bounds_.max.z + eps;
}

double VoxelGridMap::getResolution() const
{
  return resolution_;
}

common::BoundingBox VoxelGridMap::getBounds() const
{
  return bounds_;
}

const std::unordered_set<GridIndex, GridIndexHash>& VoxelGridMap::occupiedCells() const
{
  return occupied_;
}

}  // namespace nav3d::map
