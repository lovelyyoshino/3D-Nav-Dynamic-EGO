#include "nav3d/map/map_builder.h"

#include <utility>

#include "nav3d/map/pcd_loader.h"

namespace nav3d::map {

common::Result<VoxelMapBuildResult> MapBuilder::buildVoxelMap(const MapBuildConfig& config)
{
  const auto loaded = PcdLoader::load(config.pcd_path);
  if (!loaded.ok()) {
    return common::Result<VoxelMapBuildResult>::failure(loaded.error());
  }

  return common::Result<VoxelMapBuildResult>::success(
    buildVoxelMapFromPointCloud(loaded.value(), config));
}

VoxelMapBuildResult MapBuilder::buildVoxelMapFromPointCloud(
  const PointCloud& cloud,
  const MapBuildConfig& config)
{
  const auto filtered = PointCloudPreprocessor::filter(cloud, config.preprocessor);
  auto map = VoxelGridMap::fromPointCloud(filtered, config.preprocessor.resolution);
  const auto occupied_voxel_count = map.occupiedCells().size();

  return {
    std::move(map),
    cloud.points.size(),
    filtered.points.size(),
    occupied_voxel_count,
  };
}

}  // namespace nav3d::map
