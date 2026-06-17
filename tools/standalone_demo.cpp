#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav3d/collision/trajectory_checker.h"
#include "nav3d/map/map_builder.h"
#include "nav3d/map/octomap_manager.h"
#include "nav3d/map/pcd_loader.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/bspline/uniform_bspline.h"
#include "nav3d/planner/path_searching/astar_3d.h"

#ifdef NAV3D_HAS_PCL
#include "pcl_pcd_loader.h"
#endif

namespace {

enum class PcdLoaderBackend {
  Builtin,
  Pcl,
};

const char* toString(PcdLoaderBackend backend)
{
  switch (backend) {
    case PcdLoaderBackend::Builtin:
      return "builtin";
    case PcdLoaderBackend::Pcl:
      return "pcl";
  }
  return "unknown";
}

PcdLoaderBackend parsePcdLoaderBackend(const std::string& value)
{
  if (value == "builtin") {
    return PcdLoaderBackend::Builtin;
  }
  if (value == "pcl") {
    return PcdLoaderBackend::Pcl;
  }
  throw std::invalid_argument("--pcd-loader expects one of: builtin, pcl");
}

nav3d::map::VoxelGridMap makeFallbackMap(double resolution)
{
  nav3d::map::VoxelGridMap map(resolution);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {6.0, 4.0, 2.0});
  map.insertOccupied({2.0, 0.0, 0.0});
  map.insertOccupied({2.0, 1.0, 0.0});
  map.insertOccupied({2.0, 2.0, 0.0});
  return map;
}

double parseDoubleArg(const std::string& value, const std::string& option)
{
  try {
    return std::stod(value);
  } catch (const std::exception&) {
    throw std::invalid_argument(option + " expects a numeric value");
  }
}

int parseIntArg(const std::string& value, const std::string& option)
{
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    throw std::invalid_argument(option + " expects an integer value");
  }
}

nav3d::common::Point3D parsePoint3DArg(const std::string& value, const std::string& option)
{
  std::string normalized = value;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::istringstream stream(normalized);
  nav3d::common::Point3D point;
  if (!(stream >> point.x >> point.y >> point.z)) {
    throw std::invalid_argument(option + " expects three numeric values: x,y,z");
  }
  std::string extra;
  if (stream >> extra) {
    throw std::invalid_argument(option + " expects exactly three numeric values: x,y,z");
  }
  return point;
}

std::string nextArg(int& index, int argc, char** argv, const std::string& option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(option + " expects a value");
  }
  ++index;
  return argv[index];
}

nav3d::common::Result<nav3d::map::PointCloud> loadPointCloud(
  PcdLoaderBackend backend,
  const std::string& pcd_path)
{
  if (backend == PcdLoaderBackend::Builtin) {
    return nav3d::map::PcdLoader::load(pcd_path);
  }

#ifdef NAV3D_HAS_PCL
  return nav3d::tools::PclPcdLoader::load(pcd_path);
#else
  (void)pcd_path;
  return nav3d::common::Result<nav3d::map::PointCloud>::failure(
    "PCL loader requested but nav3d_standalone_demo was built without PCL");
#endif
}

std::string trim(const std::string& value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

struct PointCloudFrameSpec {
  std::string pcd_path;
  nav3d::common::Point3D sensor_origin;
};

nav3d::common::Result<std::vector<PointCloudFrameSpec>> loadFrameManifest(
  const std::string& manifest_path)
{
  std::ifstream in(manifest_path);
  if (!in) {
    return nav3d::common::Result<std::vector<PointCloudFrameSpec>>::failure(
      "failed to open point cloud frame manifest: " + manifest_path);
  }

  const auto base_dir = std::filesystem::absolute(manifest_path).parent_path();
  std::vector<PointCloudFrameSpec> frames;
  std::string line;
  int line_number = 0;
  while (std::getline(in, line)) {
    ++line_number;
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped[0] == '#') {
      continue;
    }

    std::istringstream stream(stripped);
    std::string pcd_path;
    nav3d::common::Point3D origin;
    if (!(stream >> pcd_path >> origin.x >> origin.y >> origin.z)) {
      return nav3d::common::Result<std::vector<PointCloudFrameSpec>>::failure(
        "invalid point cloud frame manifest line " + std::to_string(line_number) +
        ": expected '<pcd_path> <origin_x> <origin_y> <origin_z>'");
    }
    std::string extra;
    if (stream >> extra) {
      return nav3d::common::Result<std::vector<PointCloudFrameSpec>>::failure(
        "invalid point cloud frame manifest line " + std::to_string(line_number) +
        ": too many fields");
    }

    std::filesystem::path resolved_path(pcd_path);
    if (resolved_path.is_relative()) {
      resolved_path = base_dir / resolved_path;
    }
    frames.push_back({resolved_path.lexically_normal().string(), origin});
  }

  if (frames.empty()) {
    return nav3d::common::Result<std::vector<PointCloudFrameSpec>>::failure(
      "point cloud frame manifest contains no frames: " + manifest_path);
  }
  return nav3d::common::Result<std::vector<PointCloudFrameSpec>>::success(std::move(frames));
}

}  // namespace

int main(int argc, char** argv)
{
  double resolution = 0.5;
  int min_points_per_voxel = 1;
  int min_cluster_voxels = 1;
  bool load_only = false;
  bool insert_free_space_rays = false;
  std::optional<nav3d::common::Point3D> sensor_origin;
  std::string save_octomap_path;
  std::string frame_manifest_path;
  PcdLoaderBackend pcd_loader_backend = PcdLoaderBackend::Builtin;
  std::string pcd_path;

  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--load-only") {
        load_only = true;
      } else if (arg == "--resolution") {
        resolution = parseDoubleArg(nextArg(i, argc, argv, arg), arg);
      } else if (arg == "--min-points-per-voxel") {
        min_points_per_voxel = parseIntArg(nextArg(i, argc, argv, arg), arg);
      } else if (arg == "--min-cluster-voxels") {
        min_cluster_voxels = parseIntArg(nextArg(i, argc, argv, arg), arg);
      } else if (arg == "--insert-free-space-rays") {
        insert_free_space_rays = true;
      } else if (arg == "--sensor-origin") {
        sensor_origin = parsePoint3DArg(nextArg(i, argc, argv, arg), arg);
      } else if (arg == "--pcd-loader") {
        pcd_loader_backend = parsePcdLoaderBackend(nextArg(i, argc, argv, arg));
      } else if (arg == "--save-octomap") {
        save_octomap_path = nextArg(i, argc, argv, arg);
      } else if (arg == "--pointcloud-frame-manifest") {
        frame_manifest_path = nextArg(i, argc, argv, arg);
      } else if (!arg.empty() && arg[0] == '-') {
        throw std::invalid_argument("unknown option: " + arg);
      } else {
        pcd_path = arg;
      }
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  nav3d::map::VoxelGridMap map = makeFallbackMap(resolution);

  if (!frame_manifest_path.empty()) {
    if (!pcd_path.empty()) {
      std::cerr << "--pointcloud-frame-manifest cannot be combined with a positional .pcd path\n";
      return 1;
    }
    if (save_octomap_path.empty()) {
      std::cerr << "--pointcloud-frame-manifest currently requires --save-octomap\n";
      return 1;
    }
    const auto manifest = loadFrameManifest(frame_manifest_path);
    if (!manifest.ok()) {
      std::cerr << manifest.error() << "\n";
      return 1;
    }

    nav3d::map::MapBuildConfig config;
    config.preprocessor.resolution = resolution;
    config.preprocessor.min_points_per_voxel = min_points_per_voxel;
    config.preprocessor.min_cluster_voxels = min_cluster_voxels;
    config.insert_free_space_rays = true;

    std::vector<nav3d::map::PointCloudFrame> frames;
    frames.reserve(manifest.value().size());
    for (const auto& frame_spec : manifest.value()) {
      auto loaded_cloud = loadPointCloud(pcd_loader_backend, frame_spec.pcd_path);
      if (!loaded_cloud.ok()) {
        std::cerr << loaded_cloud.error() << "\n";
        return 1;
      }
      frames.push_back({std::move(loaded_cloud.value()), frame_spec.sensor_origin});
    }

    auto built_octomap = nav3d::map::OctomapManager::buildFromPointCloudFrames(frames, config);
    if (!built_octomap.ok()) {
      std::cerr << built_octomap.error() << "\n";
      return 1;
    }
    std::cout << "loaded_pointcloud_frames=" << frames.size()
              << " pcd_loader=" << toString(pcd_loader_backend)
              << " raw_points=" << built_octomap.value().raw_point_count
              << " filtered_points=" << built_octomap.value().filtered_point_count
              << " resolution=" << resolution
              << " min_points_per_voxel=" << min_points_per_voxel
              << " min_cluster_voxels=" << min_cluster_voxels << "\n";

    const auto saved = built_octomap.value().map.saveBinary(save_octomap_path);
    if (!saved.ok()) {
      std::cerr << saved.error() << "\n";
      return 1;
    }
    std::cout << "saved_octomap=" << save_octomap_path
              << " occupied_leafs=" << built_octomap.value().occupied_leaf_count << "\n";

    if (load_only) {
      return 0;
    }
    std::cerr << "--pointcloud-frame-manifest currently supports --load-only export smoke only\n";
    return 1;
  } else if (!pcd_path.empty()) {
    nav3d::map::MapBuildConfig config;
    config.pcd_path = pcd_path;
    config.preprocessor.resolution = resolution;
    config.preprocessor.min_points_per_voxel = min_points_per_voxel;
    config.preprocessor.min_cluster_voxels = min_cluster_voxels;
    config.insert_free_space_rays = insert_free_space_rays;
    config.sensor_origin = sensor_origin;

    auto loaded_cloud = loadPointCloud(pcd_loader_backend, pcd_path);
    if (!loaded_cloud.ok()) {
      std::cerr << loaded_cloud.error() << "\n";
      return 1;
    }

    auto built_map = nav3d::map::MapBuilder::buildVoxelMapFromPointCloud(
      loaded_cloud.value(),
      config);
    std::cout << "loaded PCD pcd_loader=" << toString(pcd_loader_backend)
              << " raw_points=" << built_map.raw_point_count
              << " filtered_points=" << built_map.filtered_point_count
              << " occupied_voxels=" << built_map.occupied_voxel_count
              << " resolution=" << resolution
              << " min_points_per_voxel=" << min_points_per_voxel
              << " min_cluster_voxels=" << min_cluster_voxels << "\n";

    if (!save_octomap_path.empty()) {
      auto built_octomap = nav3d::map::OctomapManager::buildFromPointCloud(
        loaded_cloud.value(),
        config);
      if (!built_octomap.ok()) {
        std::cerr << built_octomap.error() << "\n";
        return 1;
      }
      const auto saved = built_octomap.value().map.saveBinary(save_octomap_path);
      if (!saved.ok()) {
        std::cerr << saved.error() << "\n";
        return 1;
      }
      std::cout << "saved_octomap=" << save_octomap_path
                << " occupied_leafs=" << built_octomap.value().occupied_leaf_count << "\n";
    }

    map = std::move(built_map.map);

    if (load_only) {
      return 0;
    }
  } else {
    if (!save_octomap_path.empty()) {
      std::cerr << "--save-octomap requires a .pcd input path\n";
      return 1;
    }
    std::cout << "using built-in fallback map; pass a .pcd path to load point cloud input\n";
  }

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = true;

  const auto plan = astar.search(map, {0.0, 0.0, 0.0}, {5.0, 3.0, 0.0}, options);
  if (plan.status != nav3d::planner::SearchStatus::Success) {
    std::cerr << "A* failed; status=" << static_cast<int>(plan.status)
              << " iterations=" << plan.iterations << "\n";
    return 2;
  }

  const auto spline = nav3d::planner::UniformBspline::fitThroughWaypoints(plan.path, 0.2);
  nav3d::collision::TrajectoryChecker checker(0.05);
  const auto collision = checker.check(map, spline);

  std::cout << "path_waypoints=" << plan.path.size()
            << " spline_duration=" << spline.duration()
            << " collision=" << (collision.in_collision ? "yes" : "no") << "\n";
  return collision.in_collision ? 3 : 0;
}
