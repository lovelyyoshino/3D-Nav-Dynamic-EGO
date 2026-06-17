#pragma once

#include "nav3d/common/types.h"
#include "nav3d/map/i_map.h"

namespace nav3d::collision {

struct SphereCollisionResult {
  bool in_collision = false;
  double nearest_obstacle_distance = 0.0;
  double clearance = 0.0;
};

class SphereChecker {
public:
  explicit SphereChecker(double radius);

  SphereCollisionResult check(
    const map::IMap& map,
    const common::Point3D& center) const;

  double radius() const;

private:
  double radius_ = 0.0;
};

}  // namespace nav3d::collision
