#include "nav3d/map/map_2d_projection.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace nav3d::map {

std::optional<OccupancyGrid2D> Map2DProjection::projectOccupiedVoxels(
  const VoxelGridMap& map,
  int max_cells)
{
  Map2DProjectionOptions options;
  options.max_cells = max_cells;
  return projectOccupiedVoxels(map, options);
}

std::optional<OccupancyGrid2D> Map2DProjection::projectOccupiedVoxels(
  const VoxelGridMap& map,
  const Map2DProjectionOptions& options)
{
  if (options.max_cells <= 0) {
    return std::nullopt;
  }
  if (options.min_z.has_value() && options.max_z.has_value() &&
      *options.max_z < *options.min_z) {
    return std::nullopt;
  }

  const auto bounds = map.getBounds();
  if (!bounds.valid) {
    return std::nullopt;
  }

  const double resolution = map.getResolution();
  const int min_x = static_cast<int>(std::floor(bounds.min.x / resolution + 1e-9));
  const int max_x = static_cast<int>(std::floor(bounds.max.x / resolution + 1e-9));
  const int min_y = static_cast<int>(std::floor(bounds.min.y / resolution + 1e-9));
  const int max_y = static_cast<int>(std::floor(bounds.max.y / resolution + 1e-9));
  const int width = max_x - min_x + 1;
  const int height = max_y - min_y + 1;
  if (width <= 0 || height <= 0) {
    return std::nullopt;
  }

  const auto cell_count =
    static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
  if (cell_count > static_cast<std::uint64_t>(options.max_cells) ||
      cell_count > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return std::nullopt;
  }

  OccupancyGrid2D grid;
  grid.resolution = resolution;
  grid.origin = {
    static_cast<double>(min_x) * resolution,
    static_cast<double>(min_y) * resolution,
    0.0,
  };
  grid.width = width;
  grid.height = height;
  grid.data.assign(
    static_cast<std::size_t>(cell_count),
    (options.unknown_by_default || options.is_free_cell)
      ? static_cast<std::int8_t>(-1)
      : static_cast<std::int8_t>(0));

  if (options.is_free_cell) {
    const double free_z = options.free_cell_z.value_or(grid.origin.z);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const common::Point3D point{
          grid.origin.x + (static_cast<double>(x) + 0.5) * resolution,
          grid.origin.y + (static_cast<double>(y) + 0.5) * resolution,
          free_z,
        };
        if (options.is_free_cell(point)) {
          const auto index =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x);
          grid.data[index] = 0;
        }
      }
    }
  }

  for (const auto& cell : map.occupiedCells()) {
    const auto corner = map.gridToWorld(cell);
    const double center_z = corner.z + resolution * 0.5;
    if (options.min_z.has_value() && center_z < *options.min_z) {
      continue;
    }
    if (options.max_z.has_value() && center_z > *options.max_z) {
      continue;
    }

    const int x = cell.x - min_x;
    const int y = cell.y - min_y;
    if (x < 0 || y < 0 || x >= width || y >= height) {
      continue;
    }
    const auto index =
      static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(x);
    if (grid.data[index] != 100) {
      ++grid.occupied_count;
    }
    grid.data[index] = 100;
  }

  return grid;
}

}  // namespace nav3d::map
