#pragma once

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

#include "nav3d/common/result.h"
#include "nav3d/map/i_map.h"
#include "nav3d/map/map_builder.h"

namespace nav3d::map {

struct OctomapBuildResult;

class OctomapManager final : public IMap {
public:
  OctomapManager();
  explicit OctomapManager(double resolution);
  OctomapManager(OctomapManager&&) noexcept = default;
  OctomapManager& operator=(OctomapManager&&) noexcept = default;
  OctomapManager(const OctomapManager&) = delete;
  OctomapManager& operator=(const OctomapManager&) = delete;

  static common::Result<OctomapManager> loadBinary(const std::string& path);
  static common::Result<OctomapManager> deserializeBinary(
    const std::vector<std::uint8_t>& payload);
  static common::Result<OctomapBuildResult> buildFromPcd(const MapBuildConfig& config);
  static common::Result<OctomapBuildResult> buildFromPointCloud(
    const PointCloud& cloud,
    const MapBuildConfig& config);
  static common::Result<OctomapBuildResult> buildFromPointCloudFrames(
    const std::vector<PointCloudFrame>& frames,
    const MapBuildConfig& config);

  common::Result<bool> saveBinary(const std::string& path) const;
  common::Result<std::vector<std::uint8_t>> serializeBinary() const;

  void insertOccupied(const common::Point3D& point);
  void markRayFreeAndOccupied(
    const common::Point3D& origin,
    const common::Point3D& endpoint);
  void setExplicitBounds(const common::BoundingBox& bounds);

  std::size_t occupiedLeafCount() const;
  const octomap::OcTree& tree() const;

  bool isOccupied(const common::Point3D& p) const override;
  bool isFree(const common::Point3D& p) const override;
  double getDistance(const common::Point3D& p) const override;
  bool isInBounds(const common::Point3D& p) const override;
  double getResolution() const override;
  common::BoundingBox getBounds() const override;

private:
  explicit OctomapManager(std::unique_ptr<octomap::OcTree> tree);

  void refreshMetricBounds();

  std::unique_ptr<octomap::OcTree> tree_;
  common::BoundingBox bounds_;
  bool has_explicit_bounds_ = false;
};

struct OctomapBuildResult {
  OctomapManager map;
  std::size_t raw_point_count = 0;
  std::size_t filtered_point_count = 0;
  std::size_t occupied_leaf_count = 0;
};

}  // namespace nav3d::map
