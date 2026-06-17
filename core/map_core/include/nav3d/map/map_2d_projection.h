#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <cstddef>
#include <vector>

#include "nav3d/common/types.h"
#include "nav3d/map/voxel_grid_map.h"

namespace nav3d::map {

struct OccupancyGrid2D {
  double resolution = 1.0;
  common::Point3D origin;
  int width = 0;
  int height = 0;
  std::size_t occupied_count = 0;
  std::vector<std::int8_t> data;
};

struct Map2DProjectionOptions {
  int max_cells = 0;
  std::optional<double> min_z;
  std::optional<double> max_z;
  bool unknown_by_default = false;
  std::optional<double> free_cell_z;
  std::function<bool(const common::Point3D&)> is_free_cell;
};

class Map2DProjection {
public:
  static std::optional<OccupancyGrid2D> projectOccupiedVoxels(
    const VoxelGridMap& map,
    int max_cells);

  static std::optional<OccupancyGrid2D> projectOccupiedVoxels(
    const VoxelGridMap& map,
    const Map2DProjectionOptions& options);
};

}  // namespace nav3d::map
