#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "nav3d/common/result.h"
#include "nav3d/common/types.h"
#include "nav3d/map/point_cloud_preprocessor.h"
#include "nav3d/map/voxel_grid_map.h"

namespace nav3d::map {

struct MapBuildConfig {
  std::string pcd_path;
  PointCloudPreprocessorConfig preprocessor;
  bool insert_free_space_rays = false;
  std::optional<common::Point3D> sensor_origin;
};

struct VoxelMapBuildResult {
  VoxelGridMap map{1.0};
  std::size_t raw_point_count = 0;
  std::size_t filtered_point_count = 0;
  std::size_t occupied_voxel_count = 0;
};

class MapBuilder {
public:
  static common::Result<VoxelMapBuildResult> buildVoxelMap(const MapBuildConfig& config);
  static VoxelMapBuildResult buildVoxelMapFromPointCloud(
    const PointCloud& cloud,
    const MapBuildConfig& config);
};

}  // namespace nav3d::map
