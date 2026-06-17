#include "nav3d/map/octomap_manager.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <octomap/OcTreeKey.h>

#include "nav3d/map/pcd_loader.h"

namespace nav3d::map {

namespace {

common::BoundingBox boundsFromPointCloud(const PointCloud& cloud)
{
  common::BoundingBox bounds;
  for (const auto& point : cloud.points) {
    bounds.expandToInclude(point);
  }
  return bounds;
}

common::BoundingBox expandBounds(common::BoundingBox bounds, double margin)
{
  if (!bounds.valid) {
    return bounds;
  }
  bounds.min.x -= margin;
  bounds.min.y -= margin;
  bounds.min.z -= margin;
  bounds.max.x += margin;
  bounds.max.y += margin;
  bounds.max.z += margin;
  return bounds;
}

octomap::point3d toOctomapPoint(const common::Point3D& point)
{
  return {
    static_cast<float>(point.x),
    static_cast<float>(point.y),
    static_cast<float>(point.z),
  };
}

common::Point3D fromOctomapPoint(double x, double y, double z)
{
  return {x, y, z};
}

void insertFilteredPoints(
  OctomapManager& map,
  const PointCloud& filtered,
  bool insert_free_space_rays,
  const common::Point3D& sensor_origin)
{
  for (const auto& point : filtered.points) {
    if (insert_free_space_rays) {
      map.markRayFreeAndOccupied(sensor_origin, point);
    } else {
      map.insertOccupied(point);
    }
  }
}

}  // namespace

OctomapManager::OctomapManager() : OctomapManager(1.0) {}

OctomapManager::OctomapManager(double resolution)
  : tree_(std::make_unique<octomap::OcTree>(resolution))
{
  if (resolution <= 0.0) {
    throw std::invalid_argument("OctomapManager resolution must be positive");
  }
}

OctomapManager::OctomapManager(std::unique_ptr<octomap::OcTree> tree)
  : tree_(std::move(tree))
{
  refreshMetricBounds();
}

common::Result<OctomapBuildResult> OctomapManager::buildFromPcd(const MapBuildConfig& config)
{
  const auto loaded = PcdLoader::load(config.pcd_path);
  if (!loaded.ok()) {
    return common::Result<OctomapBuildResult>::failure(loaded.error());
  }

  return buildFromPointCloud(loaded.value(), config);
}

common::Result<OctomapBuildResult> OctomapManager::buildFromPointCloud(
  const PointCloud& cloud,
  const MapBuildConfig& config)
{
  const auto filtered = PointCloudPreprocessor::filter(cloud, config.preprocessor);
  OctomapManager map(config.preprocessor.resolution);
  auto bounds = boundsFromPointCloud(cloud);
  if (config.sensor_origin.has_value()) {
    bounds.expandToInclude(*config.sensor_origin);
  }
  map.setExplicitBounds(expandBounds(bounds, config.preprocessor.resolution));
  if (config.insert_free_space_rays && !config.sensor_origin.has_value()) {
    return common::Result<OctomapBuildResult>::failure(
      "sensor_origin is required when insert_free_space_rays is enabled");
  }
  insertFilteredPoints(
    map,
    filtered,
    config.insert_free_space_rays,
    config.sensor_origin.value_or(common::Point3D{}));
  const auto occupied_leaf_count = map.occupiedLeafCount();

  OctomapBuildResult result{
    std::move(map),
    cloud.points.size(),
    filtered.points.size(),
    occupied_leaf_count,
  };
  return common::Result<OctomapBuildResult>::success(std::move(result));
}

common::Result<OctomapBuildResult> OctomapManager::buildFromPointCloudFrames(
  const std::vector<PointCloudFrame>& frames,
  const MapBuildConfig& config)
{
  if (frames.empty()) {
    return common::Result<OctomapBuildResult>::failure(
      "at least one point cloud frame is required");
  }

  if (!config.insert_free_space_rays) {
    return common::Result<OctomapBuildResult>::failure(
      "insert_free_space_rays is required for point cloud frame ray insertion");
  }

  OctomapManager map(config.preprocessor.resolution);
  common::BoundingBox bounds;
  std::size_t raw_point_count = 0;
  std::size_t filtered_point_count = 0;
  std::vector<PointCloud> filtered_frames;
  filtered_frames.reserve(frames.size());

  for (const auto& frame : frames) {
    raw_point_count += frame.cloud.points.size();
    bounds.expandToInclude(frame.sensor_origin);
    for (const auto& point : frame.cloud.points) {
      bounds.expandToInclude(point);
    }

    filtered_frames.push_back(PointCloudPreprocessor::filter(frame.cloud, config.preprocessor));
    filtered_point_count += filtered_frames.back().points.size();
  }

  map.setExplicitBounds(expandBounds(bounds, config.preprocessor.resolution));
  for (std::size_t i = 0; i < frames.size(); ++i) {
    insertFilteredPoints(
      map,
      filtered_frames[i],
      true,
      frames[i].sensor_origin);
  }

  OctomapBuildResult result{
    std::move(map),
    raw_point_count,
    filtered_point_count,
    0,
  };
  result.occupied_leaf_count = result.map.occupiedLeafCount();
  return common::Result<OctomapBuildResult>::success(std::move(result));
}

common::Result<OctomapManager> OctomapManager::loadBinary(const std::string& path)
{
  auto tree = std::make_unique<octomap::OcTree>(1.0);
  if (!tree->readBinary(path)) {
    return common::Result<OctomapManager>::failure("failed to load OctoMap binary: " + path);
  }
  OctomapManager map(std::move(tree));
  return common::Result<OctomapManager>::success(std::move(map));
}

common::Result<OctomapManager> OctomapManager::deserializeBinary(
  const std::vector<std::uint8_t>& payload)
{
  if (payload.empty()) {
    return common::Result<OctomapManager>::failure("failed to deserialize OctoMap binary: empty payload");
  }

  std::string bytes(payload.begin(), payload.end());
  std::stringstream stream(bytes, std::ios::in | std::ios::out | std::ios::binary);
  auto tree = std::make_unique<octomap::OcTree>(1.0);
  if (!tree->readBinary(stream)) {
    return common::Result<OctomapManager>::failure("failed to deserialize OctoMap binary payload");
  }

  OctomapManager map(std::move(tree));
  return common::Result<OctomapManager>::success(std::move(map));
}

common::Result<bool> OctomapManager::saveBinary(const std::string& path) const
{
  if (!tree_->writeBinary(path)) {
    return common::Result<bool>::failure("failed to save OctoMap binary: " + path);
  }
  return common::Result<bool>::success(true);
}

common::Result<std::vector<std::uint8_t>> OctomapManager::serializeBinary() const
{
  std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
  if (!tree_->writeBinaryConst(stream)) {
    return common::Result<std::vector<std::uint8_t>>::failure(
      "failed to serialize OctoMap binary payload");
  }

  const std::string bytes = stream.str();
  return common::Result<std::vector<std::uint8_t>>::success(
    std::vector<std::uint8_t>(bytes.begin(), bytes.end()));
}

void OctomapManager::insertOccupied(const common::Point3D& point)
{
  tree_->updateNode(toOctomapPoint(point), true);
  if (!has_explicit_bounds_) {
    bounds_.expandToInclude(point);
  }
}

void OctomapManager::markRayFreeAndOccupied(
  const common::Point3D& origin,
  const common::Point3D& endpoint)
{
  if (has_explicit_bounds_ && (!isInBounds(origin) || !isInBounds(endpoint))) {
    throw std::out_of_range("OctomapManager ray endpoint is outside map bounds");
  }

  octomap::KeyRay ray;
  if (!tree_->computeRayKeys(toOctomapPoint(origin), toOctomapPoint(endpoint), ray)) {
    throw std::out_of_range("OctomapManager ray cannot be represented by the OcTree key space");
  }

  for (const auto& key : ray) {
    tree_->updateNode(key, false);
  }
  tree_->updateNode(toOctomapPoint(endpoint), true);

  if (!has_explicit_bounds_) {
    bounds_.expandToInclude(origin);
    bounds_.expandToInclude(endpoint);
  }
}

void OctomapManager::setExplicitBounds(const common::BoundingBox& bounds)
{
  bounds_ = bounds;
  has_explicit_bounds_ = bounds.valid;
}

std::size_t OctomapManager::occupiedLeafCount() const
{
  std::size_t count = 0;
  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    if (tree_->isNodeOccupied(*it)) {
      ++count;
    }
  }
  return count;
}

const octomap::OcTree& OctomapManager::tree() const
{
  return *tree_;
}

bool OctomapManager::isOccupied(const common::Point3D& p) const
{
  const auto* node = tree_->search(toOctomapPoint(p));
  return node != nullptr && tree_->isNodeOccupied(node);
}

bool OctomapManager::isFree(const common::Point3D& p) const
{
  return isInBounds(p) && !isOccupied(p);
}

double OctomapManager::getDistance(const common::Point3D& p) const
{
  double best = std::numeric_limits<double>::infinity();
  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    if (!tree_->isNodeOccupied(*it)) {
      continue;
    }
    best = std::min(best, common::distance(p, fromOctomapPoint(it.getX(), it.getY(), it.getZ())));
  }
  return best;
}

bool OctomapManager::isInBounds(const common::Point3D& p) const
{
  if (!bounds_.valid) {
    return true;
  }

  const double eps = 1e-9;
  return p.x >= bounds_.min.x - eps && p.x <= bounds_.max.x + eps &&
         p.y >= bounds_.min.y - eps && p.y <= bounds_.max.y + eps &&
         p.z >= bounds_.min.z - eps && p.z <= bounds_.max.z + eps;
}

double OctomapManager::getResolution() const
{
  return tree_->getResolution();
}

common::BoundingBox OctomapManager::getBounds() const
{
  return bounds_;
}

void OctomapManager::refreshMetricBounds()
{
  if (tree_->size() == 0) {
    bounds_ = {};
    return;
  }

  double min_x = 0.0;
  double min_y = 0.0;
  double min_z = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  double max_z = 0.0;
  tree_->getMetricMin(min_x, min_y, min_z);
  tree_->getMetricMax(max_x, max_y, max_z);
  bounds_.min = {min_x, min_y, min_z};
  bounds_.max = {max_x, max_y, max_z};
  bounds_.valid = true;
  bounds_ = expandBounds(bounds_, tree_->getResolution());
}

}  // namespace nav3d::map
