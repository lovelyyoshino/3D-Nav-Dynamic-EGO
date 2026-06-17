#pragma once

#include <string>
#include <vector>

#include "nav3d/common/result.h"
#include "nav3d/common/types.h"

namespace nav3d::map {

struct PointCloud {
  std::vector<common::Point3D> points;
};

struct PointCloudFrame {
  PointCloud cloud;
  common::Point3D sensor_origin;
};

class PcdLoader {
public:
  static common::Result<PointCloud> load(const std::string& path);
};

}  // namespace nav3d::map
