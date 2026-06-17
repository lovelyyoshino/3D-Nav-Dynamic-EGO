#pragma once

#include "nav3d/map/i_map.h"

namespace nav3d::collision {

class InflationLayer final : public map::IMap {
public:
  InflationLayer(const map::IMap& map, double radius);

  bool isOccupied(const common::Point3D& p) const override;
  bool isFree(const common::Point3D& p) const override;
  double getDistance(const common::Point3D& p) const override;
  bool isInBounds(const common::Point3D& p) const override;
  double getResolution() const override;
  common::BoundingBox getBounds() const override;

  double radius() const;

private:
  const map::IMap& map_;
  double radius_ = 0.0;
};

}  // namespace nav3d::collision
