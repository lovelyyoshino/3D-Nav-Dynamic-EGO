#pragma once

#include "nav3d/common/types.h"

namespace nav3d::map {

class IMap {
public:
  virtual ~IMap() = default;

  virtual bool isOccupied(const common::Point3D& p) const = 0;
  virtual bool isFree(const common::Point3D& p) const = 0;
  virtual double getDistance(const common::Point3D& p) const = 0;
  virtual bool isInBounds(const common::Point3D& p) const = 0;
  virtual double getResolution() const = 0;
  virtual common::BoundingBox getBounds() const = 0;
};

}  // namespace nav3d::map
