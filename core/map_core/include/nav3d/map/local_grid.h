#pragma once

#include <unordered_map>

#include "nav3d/map/i_map.h"
#include "nav3d/map/voxel_grid_map.h"

namespace nav3d::map {

enum class CellState {
  Unknown,
  Free,
  Occupied,
};

class LocalGrid final : public IMap {
public:
  LocalGrid(double resolution, const common::Point3D& min_bound, const common::Point3D& max_bound);

  void markFree(const common::Point3D& p);
  void markOccupied(const common::Point3D& p);
  void markRayFreeAndOccupied(
    const common::Point3D& origin,
    const common::Point3D& endpoint);
  bool hasObservation(const common::Point3D& p) const;
  CellState getCellState(const common::Point3D& p) const;
  void clear();
  bool hasOccupiedCells() const;
  const std::unordered_map<GridIndex, CellState, GridIndexHash>& observedCells() const;

  GridIndex worldToGrid(const common::Point3D& p) const;
  common::Point3D gridToWorld(const GridIndex& idx) const;

  bool isOccupied(const common::Point3D& p) const override;
  bool isFree(const common::Point3D& p) const override;
  double getDistance(const common::Point3D& p) const override;
  bool isInBounds(const common::Point3D& p) const override;
  double getResolution() const override;
  common::BoundingBox getBounds() const override;

private:
  void setCell(const common::Point3D& p, CellState state);

  double resolution_ = 1.0;
  common::BoundingBox bounds_;
  std::unordered_map<GridIndex, CellState, GridIndexHash> cells_;
};

}  // namespace nav3d::map
