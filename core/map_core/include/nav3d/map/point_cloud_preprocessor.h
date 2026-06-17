#pragma once

#include "nav3d/map/pcd_loader.h"

namespace nav3d::map {

struct PointCloudPreprocessorConfig {
  double resolution = 1.0;
  int min_points_per_voxel = 1;
  int min_cluster_voxels = 1;
};

class PointCloudPreprocessor {
public:
  static PointCloud filter(const PointCloud& cloud, const PointCloudPreprocessorConfig& config);
};

}  // namespace nav3d::map
