#include "nav3d/map/point_cloud_preprocessor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace nav3d::map {
namespace {

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const
  {
    std::size_t seed = std::hash<int>{}(key.x);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

VoxelKey pointToVoxel(const common::Point3D& point, double resolution)
{
  return {
    static_cast<int>(std::floor(point.x / resolution + 1e-9)),
    static_cast<int>(std::floor(point.y / resolution + 1e-9)),
    static_cast<int>(std::floor(point.z / resolution + 1e-9)),
  };
}

common::Point3D voxelToPoint(const VoxelKey& key, double resolution)
{
  return {
    static_cast<double>(key.x) * resolution,
    static_cast<double>(key.y) * resolution,
    static_cast<double>(key.z) * resolution,
  };
}

bool isFinite(const common::Point3D& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

void validateConfig(const PointCloudPreprocessorConfig& config)
{
  if (!std::isfinite(config.resolution) || config.resolution <= 0.0) {
    throw std::invalid_argument("PointCloudPreprocessor resolution must be positive and finite");
  }
  if (config.min_points_per_voxel <= 0) {
    throw std::invalid_argument("PointCloudPreprocessor min_points_per_voxel must be positive");
  }
  if (config.min_cluster_voxels <= 0) {
    throw std::invalid_argument("PointCloudPreprocessor min_cluster_voxels must be positive");
  }
}

std::unordered_set<VoxelKey, VoxelKeyHash> filterByPointCount(
  const PointCloud& cloud,
  const PointCloudPreprocessorConfig& config)
{
  std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_counts;
  for (const auto& point : cloud.points) {
    if (!isFinite(point)) {
      continue;
    }
    ++voxel_counts[pointToVoxel(point, config.resolution)];
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  occupied.reserve(voxel_counts.size());
  for (const auto& entry : voxel_counts) {
    if (entry.second >= config.min_points_per_voxel) {
      occupied.insert(entry.first);
    }
  }
  return occupied;
}

std::unordered_set<VoxelKey, VoxelKeyHash> filterByConnectedClusters(
  const std::unordered_set<VoxelKey, VoxelKeyHash>& occupied,
  int min_cluster_voxels)
{
  if (min_cluster_voxels <= 1 || occupied.empty()) {
    return occupied;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> filtered;
  std::unordered_set<VoxelKey, VoxelKeyHash> visited;
  filtered.reserve(occupied.size());
  visited.reserve(occupied.size());

  for (const auto& seed : occupied) {
    if (visited.find(seed) != visited.end()) {
      continue;
    }

    std::deque<VoxelKey> queue;
    std::vector<VoxelKey> cluster;
    queue.push_back(seed);
    visited.insert(seed);

    while (!queue.empty()) {
      const auto current = queue.front();
      queue.pop_front();
      cluster.push_back(current);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }

            const VoxelKey neighbor{current.x + dx, current.y + dy, current.z + dz};
            if (occupied.find(neighbor) != occupied.end() && visited.find(neighbor) == visited.end()) {
              visited.insert(neighbor);
              queue.push_back(neighbor);
            }
          }
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= min_cluster_voxels) {
      filtered.insert(cluster.begin(), cluster.end());
    }
  }

  return filtered;
}

}  // namespace

PointCloud PointCloudPreprocessor::filter(
  const PointCloud& cloud,
  const PointCloudPreprocessorConfig& config)
{
  validateConfig(config);

  const auto occupied = filterByConnectedClusters(filterByPointCount(cloud, config), config.min_cluster_voxels);

  std::vector<VoxelKey> sorted_keys(occupied.begin(), occupied.end());
  std::sort(sorted_keys.begin(), sorted_keys.end(), [](const auto& a, const auto& b) {
    if (a.x != b.x) {
      return a.x < b.x;
    }
    if (a.y != b.y) {
      return a.y < b.y;
    }
    return a.z < b.z;
  });

  PointCloud cleaned;
  cleaned.points.reserve(sorted_keys.size());
  for (const auto& key : sorted_keys) {
    cleaned.points.push_back(voxelToPoint(key, config.resolution));
  }
  return cleaned;
}

}  // namespace nav3d::map
