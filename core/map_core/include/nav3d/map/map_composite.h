#pragma once

#include "nav3d/map/i_map.h"
#include "nav3d/map/local_grid.h"

namespace nav3d::map {

class MapComposite final : public IMap {
public:
  MapComposite(const IMap& global_map, const LocalGrid& local_grid);

  bool isOccupied(const common::Point3D& p) const override;
  bool isFree(const common::Point3D& p) const override;
  double getDistance(const common::Point3D& p) const override;
  bool isInBounds(const common::Point3D& p) const override;
  double getResolution() const override;
  common::BoundingBox getBounds() const override;

private:
  const IMap& global_map_;
  const LocalGrid& local_grid_;
};

}  // namespace nav3d::map
