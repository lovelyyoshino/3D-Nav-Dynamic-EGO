#pragma once

#include <cstddef>
#include <limits>
#include <unordered_set>
#include <vector>

#include "nav3d/map/i_map.h"
#include "nav3d/map/pcd_loader.h"

namespace nav3d::map {

struct GridIndex {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const GridIndex& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash {
  std::size_t operator()(const GridIndex& idx) const;
};

class VoxelGridMap final : public IMap {
public:
  explicit VoxelGridMap(double resolution);

  static VoxelGridMap fromPointCloud(const PointCloud& cloud, double resolution);

  void insertOccupied(const common::Point3D& p);
  void setExplicitBounds(const common::Point3D& min, const common::Point3D& max);

  GridIndex worldToGrid(const common::Point3D& p) const;
  common::Point3D gridToWorld(const GridIndex& idx) const;

  bool isOccupied(const common::Point3D& p) const override;
  bool isFree(const common::Point3D& p) const override;
  double getDistance(const common::Point3D& p) const override;
  bool isInBounds(const common::Point3D& p) const override;
  double getResolution() const override;
  common::BoundingBox getBounds() const override;

  const std::unordered_set<GridIndex, GridIndexHash>& occupiedCells() const;

private:
  double resolution_ = 1.0;
  common::BoundingBox bounds_;
  bool has_explicit_bounds_ = false;
  std::unordered_set<GridIndex, GridIndexHash> occupied_;
};

}  // namespace nav3d::map
