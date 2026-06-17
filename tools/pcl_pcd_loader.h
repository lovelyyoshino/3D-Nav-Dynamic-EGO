#pragma once

#include <string>

#include "nav3d/common/result.h"
#include "nav3d/map/pcd_loader.h"

namespace nav3d::tools {

class PclPcdLoader {
public:
  static common::Result<map::PointCloud> load(const std::string& path);
};

}  // namespace nav3d::tools
