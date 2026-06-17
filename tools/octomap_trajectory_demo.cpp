#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "nav3d/collision/trajectory_checker.h"
#include "nav3d/map/i_map.h"
#include "nav3d/map/map_builder.h"
#include "nav3d/map/octomap_manager.h"
#include "nav3d/map/pcd_loader.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/ego_planner_core.h"

#ifdef NAV3D_HAS_PCL
#include "pcl_pcd_loader.h"
#endif

namespace {

enum class PcdLoaderBackend {
  Builtin,
  Pcl,
};

enum class MapBackend {
  Voxel,
  Octomap,
};

struct Options {
  double resolution = 0.5;
  int min_points_per_voxel = 1;
  int min_cluster_voxels = 1;
  int max_optimizer_iterations = 80;
  int max_fallback_attempts = 8;
  int max_search_iterations = 200000;
  int ground_snap_radius_cells = 12;
  int ground_support_xy_radius_cells = 1;
  int ground_support_depth_cells = 1;
  double ground_robot_radius = 0.25;
  double trajectory_sample_step = 0.05;
  nav3d::common::Point3D start{-13.0, 8.0, 1.0};
  nav3d::common::Point3D goal{13.0, 8.0, 1.0};
  nav3d::planner::PlanningMode planning_mode = nav3d::planner::PlanningMode::Mode3D;
  nav3d::planner::SearchAlgorithm search_algorithm = nav3d::planner::SearchAlgorithm::AStar;
  PcdLoaderBackend pcd_loader_backend = PcdLoaderBackend::Builtin;
  MapBackend map_backend = MapBackend::Octomap;
  bool insert_free_space_rays = false;
  bool require_full_goal = false;
  bool allow_diagonal = true;
  std::optional<nav3d::common::Point3D> sensor_origin;
  std::string save_octomap_path;
  std::string save_trajectory_path;
  std::string save_trajectory_plot_path;
  std::string pcd_path;
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

const char* toString(MapBackend backend)
{
  switch (backend) {
    case MapBackend::Voxel:
      return "voxel";
    case MapBackend::Octomap:
      return "octomap";
  }
  return "unknown";
}

const char* toString(nav3d::planner::EgoPlanStatus status)
{
  switch (status) {
    case nav3d::planner::EgoPlanStatus::Success:
      return "success";
    case nav3d::planner::EgoPlanStatus::SearchFailed:
      return "search_failed";
    case nav3d::planner::EgoPlanStatus::OptimizationFailed:
      return "optimization_failed";
    case nav3d::planner::EgoPlanStatus::DynamicFeasibilityViolation:
      return "dynamic_feasibility_violation";
    case nav3d::planner::EgoPlanStatus::TrajectoryCollision:
      return "trajectory_collision";
    case nav3d::planner::EgoPlanStatus::EmergencyStop:
      return "emergency_stop";
  }
  return "unknown";
}

const char* toString(nav3d::planner::PlanningMode mode)
{
  switch (mode) {
    case nav3d::planner::PlanningMode::Mode2D:
      return "2d";
    case nav3d::planner::PlanningMode::Mode3D:
      return "3d";
  }
  return "unknown";
}

const char* toString(nav3d::planner::SearchStatus status)
{
  switch (status) {
    case nav3d::planner::SearchStatus::Success:
      return "success";
    case nav3d::planner::SearchStatus::NoPath:
      return "no_path";
    case nav3d::planner::SearchStatus::InvalidInput:
      return "invalid_input";
    case nav3d::planner::SearchStatus::IterationLimit:
      return "iteration_limit";
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

MapBackend parseMapBackend(const std::string& value)
{
  if (value == "voxel") {
    return MapBackend::Voxel;
  }
  if (value == "octomap") {
    return MapBackend::Octomap;
  }
  throw std::invalid_argument("--map-backend expects one of: voxel, octomap");
}

nav3d::planner::PlanningMode parsePlanningMode(const std::string& value)
{
  if (value == "2d") {
    return nav3d::planner::PlanningMode::Mode2D;
  }
  if (value == "3d") {
    return nav3d::planner::PlanningMode::Mode3D;
  }
  throw std::invalid_argument("--planning-mode expects one of: 2d, 3d");
}

nav3d::planner::SearchAlgorithm parseSearchAlgorithm(const std::string& value)
{
  if (value == "astar") {
    return nav3d::planner::SearchAlgorithm::AStar;
  }
  if (value == "jps") {
    return nav3d::planner::SearchAlgorithm::Jps;
  }
  throw std::invalid_argument("--search-algorithm expects one of: astar, jps");
}

double parseDoubleArg(const std::string& value, const std::string& option)
{
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(value, &consumed);
    if (consumed != value.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return parsed;
  } catch (const std::exception&) {
    throw std::invalid_argument(option + " expects a numeric value");
  }
}

int parseIntArg(const std::string& value, const std::string& option)
{
  try {
    std::size_t consumed = 0;
    const int parsed = std::stoi(value, &consumed);
    if (consumed != value.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return parsed;
  } catch (const std::exception&) {
    throw std::invalid_argument(option + " expects an integer value");
  }
}

std::string nextArg(int& index, int argc, char** argv, const std::string& option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(option + " expects a value");
  }
  ++index;
  return argv[index];
}

nav3d::common::Point3D parsePoint3DString(const std::string& value, const std::string& option)
{
  std::string normalized = value;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::istringstream stream(normalized);
  nav3d::common::Point3D point;
  if (!(stream >> point.x >> point.y >> point.z)) {
    throw std::invalid_argument(option + " expects three numeric values");
  }
  std::string extra;
  if (stream >> extra) {
    throw std::invalid_argument(option + " expects exactly three numeric values");
  }
  return point;
}

nav3d::common::Point3D parsePoint3DArgs(int& index, int argc, char** argv, const std::string& option)
{
  const std::string first = nextArg(index, argc, argv, option);
  if (first.find(',') != std::string::npos) {
    return parsePoint3DString(first, option);
  }
  if (index + 2 >= argc) {
    throw std::invalid_argument(option + " expects either x,y,z or x y z");
  }
  const std::string second = argv[++index];
  const std::string third = argv[++index];
  return {
    parseDoubleArg(first, option),
    parseDoubleArg(second, option),
    parseDoubleArg(third, option),
  };
}

std::string formatPoint(const nav3d::common::Point3D& point)
{
  std::ostringstream out;
  out << std::setprecision(10) << point.x << "," << point.y << "," << point.z;
  return out.str();
}

std::string svgNumber(double value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

double pathLength(const nav3d::common::Path3D& path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += nav3d::common::distance(path[i - 1], path[i]);
  }
  return length;
}

double snappedGoalTolerance(double resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return 1e-6;
  }
  return std::sqrt(3.0) * resolution * 0.5 + 1e-6;
}

double zSpan(const nav3d::common::Path3D& path)
{
  if (path.empty()) {
    return 0.0;
  }
  auto minmax = std::minmax_element(path.begin(), path.end(), [](const auto& a, const auto& b) {
    return a.z < b.z;
  });
  return minmax.second->z - minmax.first->z;
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
    "PCL loader requested but nav3d_octomap_trajectory_demo was built without PCL");
#endif
}

nav3d::map::VoxelGridMap makeFallbackVoxelMap(double resolution)
{
  nav3d::map::VoxelGridMap map(resolution);
  map.setExplicitBounds({-3.0, -1.0, 0.0}, {3.0, 1.0, 0.0});
  for (const auto& point : {
         nav3d::common::Point3D{-3.0, -1.0, 0.0},
         nav3d::common::Point3D{-3.0, 1.0, 0.0},
         nav3d::common::Point3D{-1.0, -1.0, 0.0},
         nav3d::common::Point3D{-1.0, 1.0, 0.0},
         nav3d::common::Point3D{1.0, -1.0, 0.0},
         nav3d::common::Point3D{1.0, 1.0, 0.0},
         nav3d::common::Point3D{3.0, -1.0, 0.0},
         nav3d::common::Point3D{3.0, 1.0, 0.0},
       }) {
    map.insertOccupied(point);
  }
  return map;
}

nav3d::map::OctomapManager makeOctomapFromVoxelMap(const nav3d::map::VoxelGridMap& voxel_map)
{
  nav3d::map::OctomapManager octomap(voxel_map.getResolution());
  octomap.setExplicitBounds(voxel_map.getBounds());
  for (const auto& cell : voxel_map.occupiedCells()) {
    octomap.insertOccupied(voxel_map.gridToWorld(cell));
  }
  return octomap;
}

std::vector<nav3d::common::Point3D> sampleTrajectory(
  const nav3d::planner::UniformBspline& trajectory,
  double sample_step)
{
  std::vector<nav3d::common::Point3D> samples;
  const double duration = trajectory.duration();
  if (duration <= 0.0) {
    samples.push_back(trajectory.evaluate(0.0));
    return samples;
  }

  for (double t = 0.0; t < duration; t += sample_step) {
    samples.push_back(trajectory.evaluate(t));
  }
  samples.push_back(trajectory.evaluate(duration));
  return samples;
}

struct GroundIndex {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const GroundIndex& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GroundIndexHash {
  std::size_t operator()(const GroundIndex& idx) const
  {
    const std::size_t h1 = std::hash<int>{}(idx.x);
    const std::size_t h2 = std::hash<int>{}(idx.y);
    const std::size_t h3 = std::hash<int>{}(idx.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 2U);
  }
};

struct GroundQueueNode {
  GroundIndex idx;
  double f = 0.0;
  double g = 0.0;
};

struct GroundQueueNodeCompare {
  bool operator()(const GroundQueueNode& a, const GroundQueueNode& b) const
  {
    constexpr double kEpsilon = 1e-12;
    if (std::abs(a.f - b.f) > kEpsilon) {
      return a.f > b.f;
    }
    return a.g < b.g;
  }
};

struct GroundSearchResult {
  nav3d::planner::SearchStatus status = nav3d::planner::SearchStatus::NoPath;
  nav3d::common::Path3D path;
  int iterations = 0;
  double path_cost = 0.0;
};

struct GroundSnapCandidate {
  GroundIndex idx;
  double distance = std::numeric_limits<double>::infinity();
  double xy_distance = std::numeric_limits<double>::infinity();
};

constexpr int kGroundWarmStartMultiplicity = 3;

GroundIndex worldToGroundIndex(const nav3d::common::Point3D& point, double resolution)
{
  return {
    static_cast<int>(std::floor(point.x / resolution + 1e-9)),
    static_cast<int>(std::floor(point.y / resolution + 1e-9)),
    static_cast<int>(std::floor(point.z / resolution + 1e-9)),
  };
}

nav3d::common::Point3D groundIndexToWorld(const GroundIndex& idx, double resolution)
{
  return {
    (static_cast<double>(idx.x) + 0.5) * resolution,
    (static_cast<double>(idx.y) + 0.5) * resolution,
    (static_cast<double>(idx.z) + 0.5) * resolution,
  };
}

double groundIndexDistance(const GroundIndex& a, const GroundIndex& b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool hasGroundSupport(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const Options& options)
{
  const double resolution = map.getResolution();
  for (int dz = 1; dz <= std::max(1, options.ground_support_depth_cells); ++dz) {
    for (int dx = -options.ground_support_xy_radius_cells; dx <= options.ground_support_xy_radius_cells; ++dx) {
      for (int dy = -options.ground_support_xy_radius_cells; dy <= options.ground_support_xy_radius_cells; ++dy) {
        const GroundIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
        const auto below_world = groundIndexToWorld(below, resolution);
        if (map.isInBounds(below_world) && map.isOccupied(below_world)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool hasBodyClearance(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const Options& options)
{
  const double resolution = map.getResolution();
  const int radius_cells = std::max(0, static_cast<int>(std::ceil(options.ground_robot_radius / resolution)));
  const double radius_sq = options.ground_robot_radius * options.ground_robot_radius;
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dz = 0; dz <= radius_cells; ++dz) {
        const double dist_x = static_cast<double>(dx) * resolution;
        const double dist_y = static_cast<double>(dy) * resolution;
        const double dist_z = static_cast<double>(dz) * resolution;
        if (dist_x * dist_x + dist_y * dist_y + dist_z * dist_z > radius_sq + 1e-12) {
          continue;
        }
        const GroundIndex nearby{idx.x + dx, idx.y + dy, idx.z + dz};
        const auto nearby_world = groundIndexToWorld(nearby, resolution);
        if (map.isInBounds(nearby_world) && map.isOccupied(nearby_world)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool isGroundTraversable(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const Options& options)
{
  const auto point = groundIndexToWorld(idx, map.getResolution());
  return map.isInBounds(point) &&
         !map.isOccupied(point) &&
         hasGroundSupport(map, idx, options) &&
         hasBodyClearance(map, idx, options);
}

GroundSnapCandidate makeGroundSnapCandidate(
  const GroundIndex& requested,
  const GroundIndex& candidate,
  double resolution)
{
  const auto requested_world = groundIndexToWorld(requested, resolution);
  const auto candidate_world = groundIndexToWorld(candidate, resolution);
  const double dx = candidate_world.x - requested_world.x;
  const double dy = candidate_world.y - requested_world.y;
  return {
    candidate,
    nav3d::common::distance(requested_world, candidate_world),
    std::sqrt(dx * dx + dy * dy),
  };
}

bool isBetterGroundSnapCandidate(
  const GroundSnapCandidate& candidate,
  const GroundSnapCandidate& best)
{
  constexpr double kEpsilon = 1e-12;
  if (candidate.distance < best.distance - kEpsilon) {
    return true;
  }
  if (candidate.distance > best.distance + kEpsilon) {
    return false;
  }
  if (candidate.xy_distance < best.xy_distance - kEpsilon) {
    return true;
  }
  if (candidate.xy_distance > best.xy_distance + kEpsilon) {
    return false;
  }
  if (candidate.idx.z != best.idx.z) {
    return candidate.idx.z > best.idx.z;
  }
  if (candidate.idx.y != best.idx.y) {
    return candidate.idx.y < best.idx.y;
  }
  return candidate.idx.x < best.idx.x;
}

GroundIndex resolveGroundIndex(
  const nav3d::map::IMap& map,
  const GroundIndex& requested,
  const Options& options)
{
  if (isGroundTraversable(map, requested, options)) {
    return requested;
  }

  std::optional<GroundSnapCandidate> best;
  const int radius = options.ground_snap_radius_cells;
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        const GroundIndex candidate{
          requested.x + dx,
          requested.y + dy,
          requested.z + dz,
        };
        if (!isGroundTraversable(map, candidate, options)) {
          continue;
        }
        const auto snap_candidate =
          makeGroundSnapCandidate(requested, candidate, map.getResolution());
        if (!best.has_value() || isBetterGroundSnapCandidate(snap_candidate, *best)) {
          best = snap_candidate;
        }
      }
    }
  }
  return best.has_value() ? best->idx : requested;
}

std::vector<GroundIndex> makeGroundDirections(
  nav3d::planner::PlanningMode mode,
  bool allow_diagonal)
{
  std::vector<GroundIndex> directions;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        if (mode == nav3d::planner::PlanningMode::Mode2D && dz != 0) {
          continue;
        }
        if (!allow_diagonal && std::abs(dx) + std::abs(dy) + std::abs(dz) != 1) {
          continue;
        }
        directions.push_back({dx, dy, dz});
      }
    }
  }
  return directions;
}

bool canMoveGround(
  const nav3d::map::IMap& map,
  const GroundIndex& from,
  const GroundIndex& direction,
  const Options& options)
{
  const GroundIndex target{
    from.x + direction.x,
    from.y + direction.y,
    from.z + direction.z,
  };
  if (!isGroundTraversable(map, target, options)) {
    return false;
  }

  for (int x_step = 0; x_step <= (direction.x != 0 ? 1 : 0); ++x_step) {
    for (int y_step = 0; y_step <= (direction.y != 0 ? 1 : 0); ++y_step) {
      for (int z_step = 0; z_step <= (direction.z != 0 ? 1 : 0); ++z_step) {
        if (x_step == 0 && y_step == 0 && z_step == 0) {
          continue;
        }
        const GroundIndex intermediate{
          from.x + x_step * direction.x,
          from.y + y_step * direction.y,
          from.z + z_step * direction.z,
        };
        if (!isGroundTraversable(map, intermediate, options)) {
          return false;
        }
      }
    }
  }
  return true;
}

std::vector<GroundIndex> reconstructGroundPath(
  const std::unordered_map<GroundIndex, GroundIndex, GroundIndexHash>& came_from,
  GroundIndex current)
{
  std::vector<GroundIndex> path{current};
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    path.push_back(current);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

GroundSearchResult searchGroundSupportedPath(
  const nav3d::map::IMap& map,
  const nav3d::common::Point3D& start,
  const nav3d::common::Point3D& goal,
  const Options& options)
{
  if (map.getResolution() <= 0.0) {
    return {nav3d::planner::SearchStatus::InvalidInput, {}, 0, 0.0};
  }

  const double resolution = map.getResolution();
  const GroundIndex start_idx = resolveGroundIndex(map, worldToGroundIndex(start, resolution), options);
  const GroundIndex goal_idx = resolveGroundIndex(map, worldToGroundIndex(goal, resolution), options);
  if (!isGroundTraversable(map, start_idx, options) || !isGroundTraversable(map, goal_idx, options)) {
    return {nav3d::planner::SearchStatus::InvalidInput, {}, 0, 0.0};
  }

  std::priority_queue<GroundQueueNode, std::vector<GroundQueueNode>, GroundQueueNodeCompare> open_set;
  std::unordered_map<GroundIndex, double, GroundIndexHash> g_score;
  std::unordered_map<GroundIndex, GroundIndex, GroundIndexHash> came_from;
  std::unordered_set<GroundIndex, GroundIndexHash> closed_set;

  g_score[start_idx] = 0.0;
  open_set.push({start_idx, groundIndexDistance(start_idx, goal_idx), 0.0});
  const auto directions = makeGroundDirections(options.planning_mode, options.allow_diagonal);
  int iterations = 0;

  while (!open_set.empty() && iterations < options.max_search_iterations) {
    const auto current = open_set.top();
    open_set.pop();
    ++iterations;

    const auto best_score = g_score.find(current.idx);
    if (best_score != g_score.end() && current.g > best_score->second + 1e-12) {
      continue;
    }
    if (closed_set.find(current.idx) != closed_set.end()) {
      continue;
    }
    closed_set.insert(current.idx);

    if (current.idx == goal_idx) {
      const auto index_path = reconstructGroundPath(came_from, current.idx);
      nav3d::common::Path3D path;
      path.reserve(index_path.size() * kGroundWarmStartMultiplicity);
      for (const auto& idx : index_path) {
        const auto point = groundIndexToWorld(idx, resolution);
        for (int repeat = 0; repeat < kGroundWarmStartMultiplicity; ++repeat) {
          path.push_back(point);
        }
      }
      const double cost = pathLength(path);
      return {
        nav3d::planner::SearchStatus::Success,
        std::move(path),
        iterations,
        cost,
      };
    }

    for (const auto& direction : directions) {
      const GroundIndex neighbor{
        current.idx.x + direction.x,
        current.idx.y + direction.y,
        current.idx.z + direction.z,
      };
      if (closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }
      if (!canMoveGround(map, current.idx, direction, options)) {
        continue;
      }
      const double tentative_g = current.g + groundIndexDistance(current.idx, neighbor);
      const auto existing = g_score.find(neighbor);
      if (existing == g_score.end() || tentative_g < existing->second) {
        came_from[neighbor] = current.idx;
        g_score[neighbor] = tentative_g;
        open_set.push({neighbor, tentative_g + groundIndexDistance(neighbor, goal_idx), tentative_g});
      }
    }
  }

  return {
    iterations >= options.max_search_iterations
      ? nav3d::planner::SearchStatus::IterationLimit
      : nav3d::planner::SearchStatus::NoPath,
    {},
    iterations,
    0.0,
  };
}

class GroundSupportedSearcher final : public nav3d::planner::IPathSearcher {
public:
  explicit GroundSupportedSearcher(Options options) : options_(std::move(options)) {}

  nav3d::planner::SearchResult search(
    const nav3d::map::IMap& map,
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal,
    const nav3d::planner::SearchOptions& search_options) const override
  {
    auto options = options_;
    options.planning_mode = search_options.mode;
    options.max_search_iterations = search_options.max_iterations;
    options.allow_diagonal = search_options.allow_diagonal;
    const auto ground_result = searchGroundSupportedPath(map, start, goal, options);
    return {
      ground_result.status,
      ground_result.path,
      ground_result.iterations,
      ground_result.path_cost,
    };
  }

private:
  Options options_;
};

bool writeTrajectoryCsv(
  const std::string& path,
  const nav3d::planner::UniformBspline& trajectory,
  double sample_step)
{
  std::ofstream out(path);
  if (!out) {
    std::cerr << "failed to open trajectory output path: " << path << "\n";
    return false;
  }

  out << "t,x,y,z\n";
  const double duration = trajectory.duration();
  out << std::fixed << std::setprecision(6);
  if (duration <= 0.0) {
    const auto point = trajectory.evaluate(0.0);
    out << "0," << point.x << "," << point.y << "," << point.z << "\n";
    return true;
  }
  for (double t = 0.0; t < duration; t += sample_step) {
    const auto point = trajectory.evaluate(t);
    out << t << "," << point.x << "," << point.y << "," << point.z << "\n";
  }
  const auto point = trajectory.evaluate(duration);
  out << duration << "," << point.x << "," << point.y << "," << point.z << "\n";
  return true;
}

std::vector<nav3d::common::Point3D> occupiedVoxelCenters(const nav3d::map::VoxelGridMap& map)
{
  std::vector<nav3d::common::Point3D> centers;
  centers.reserve(map.occupiedCells().size());
  const double center_offset = map.getResolution() * 0.5;
  for (const auto& cell : map.occupiedCells()) {
    auto center = map.gridToWorld(cell);
    center.x += center_offset;
    center.y += center_offset;
    center.z += center_offset;
    centers.push_back(center);
  }
  std::sort(centers.begin(), centers.end(), [](const auto& a, const auto& b) {
    if (a.z != b.z) {
      return a.z < b.z;
    }
    if (a.y != b.y) {
      return a.y < b.y;
    }
    return a.x < b.x;
  });
  return centers;
}

std::string rgb(int red, int green, int blue)
{
  std::ostringstream out;
  out << "rgb(" << std::clamp(red, 0, 255) << "," << std::clamp(green, 0, 255) << ","
      << std::clamp(blue, 0, 255) << ")";
  return out.str();
}

bool writeTrajectorySvg(
  const std::string& path,
  const std::string& mode_label,
  const nav3d::map::VoxelGridMap& voxel_map,
  const std::vector<nav3d::common::Point3D>& samples,
  const nav3d::common::Point3D& requested_start,
  const nav3d::common::Point3D& planned_start,
  const nav3d::common::Point3D& requested_goal,
  const nav3d::common::Point3D& planned_goal,
  bool partial)
{
  const auto occupied = occupiedVoxelCenters(voxel_map);
  nav3d::common::BoundingBox bounds;
  for (const auto& point : occupied) {
    bounds.expandToInclude(point);
  }
  for (const auto& point : samples) {
    bounds.expandToInclude(point);
  }
  bounds.expandToInclude(requested_start);
  bounds.expandToInclude(planned_start);
  bounds.expandToInclude(requested_goal);
  bounds.expandToInclude(planned_goal);
  if (!bounds.valid) {
    std::cerr << "cannot write trajectory plot without map or trajectory bounds\n";
    return false;
  }

  const double resolution = voxel_map.getResolution();
  const double xy_span = std::max(bounds.max.x - bounds.min.x, bounds.max.y - bounds.min.y);
  const double padding = std::max(resolution * 2.0, xy_span * 0.05);
  bounds.min.x -= padding;
  bounds.max.x += padding;
  bounds.min.y -= padding;
  bounds.max.y += padding;
  bounds.min.z -= std::max(resolution, 0.25);
  bounds.max.z += std::max(resolution, 0.25);

  const double span_z = std::max(1e-6, bounds.max.z - bounds.min.z);

  const double width = 1280.0;
  const double height = 760.0;
  const double map_left = 72.0;
  const double map_top = 92.0;
  const double map_width = 820.0;
  const double map_height = 560.0;
  const double elev_left = 962.0;
  const double elev_top = 150.0;
  const double elev_width = 246.0;
  const double elev_height = 420.0;

  struct SvgPoint {
    double x = 0.0;
    double y = 0.0;
  };

  const double center_x = (bounds.min.x + bounds.max.x) * 0.5;
  const double center_y = (bounds.min.y + bounds.max.y) * 0.5;
  const auto projectRaw = [&](const nav3d::common::Point3D& point) {
    const double x = point.x - center_x;
    const double y = point.y - center_y;
    const double z = point.z - bounds.min.z;
    return SvgPoint{x - y, 0.52 * (x + y) - 1.05 * z};
  };

  double projected_min_x = std::numeric_limits<double>::infinity();
  double projected_min_y = std::numeric_limits<double>::infinity();
  double projected_max_x = -std::numeric_limits<double>::infinity();
  double projected_max_y = -std::numeric_limits<double>::infinity();
  const auto includeProjection = [&](const nav3d::common::Point3D& point) {
    const auto projected = projectRaw(point);
    projected_min_x = std::min(projected_min_x, projected.x);
    projected_min_y = std::min(projected_min_y, projected.y);
    projected_max_x = std::max(projected_max_x, projected.x);
    projected_max_y = std::max(projected_max_y, projected.y);
  };
  for (const auto& point : occupied) {
    includeProjection(point);
  }
  for (const auto& point : samples) {
    includeProjection(point);
  }
  for (const auto& point : {requested_start, planned_start, requested_goal, planned_goal}) {
    includeProjection(point);
  }
  for (double x : {bounds.min.x, bounds.max.x}) {
    for (double y : {bounds.min.y, bounds.max.y}) {
      for (double z : {bounds.min.z, bounds.max.z}) {
        includeProjection({x, y, z});
      }
    }
  }
  const double projected_span_x = std::max(1e-6, projected_max_x - projected_min_x);
  const double projected_span_y = std::max(1e-6, projected_max_y - projected_min_y);
  const double projected_scale = 0.9 * std::min(map_width / projected_span_x, map_height / projected_span_y);
  const double projected_center_x = (projected_min_x + projected_max_x) * 0.5;
  const double projected_center_y = (projected_min_y + projected_max_y) * 0.5;
  const auto project = [&](const nav3d::common::Point3D& point) {
    const auto raw = projectRaw(point);
    return SvgPoint{
      map_left + map_width * 0.5 + (raw.x - projected_center_x) * projected_scale,
      map_top + map_height * 0.5 + (raw.y - projected_center_y) * projected_scale,
    };
  };
  const auto polygonPoints = [&](const std::vector<nav3d::common::Point3D>& points) {
    std::ostringstream polygon;
    for (const auto& point : points) {
      const auto projected = project(point);
      polygon << svgNumber(projected.x) << "," << svgNumber(projected.y) << " ";
    }
    return polygon.str();
  };
  const auto elevY = [&](double z) {
    return elev_top + ((bounds.max.z - z) / span_z) * elev_height;
  };

  try {
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
  } catch (const std::exception& error) {
    std::cerr << "failed to create trajectory plot directory for " << path << ": "
              << error.what() << "\n";
    return false;
  }

  std::ofstream out(path);
  if (!out) {
    std::cerr << "failed to open trajectory plot output path: " << path << "\n";
    return false;
  }

  out << std::fixed << std::setprecision(3);
  out << "<svg id=\"nav3d-trajectory-plot\" xmlns=\"http://www.w3.org/2000/svg\" "
      << "viewBox=\"0 0 " << svgNumber(width) << " " << svgNumber(height) << "\" "
      << "width=\"" << svgNumber(width) << "\" height=\"" << svgNumber(height) << "\">\n";
  out << "<style>"
      << "text{font-family:Inter,Arial,sans-serif;fill:#dfffe9;font-size:18px}"
      << ".small{font-size:13px;fill:#9bd8b2}"
      << ".panel{fill:#050706;stroke:#1f8f50;stroke-width:2}"
      << ".grid{stroke:#1f8f50;stroke-width:1;opacity:.28}"
      << ".occupied-voxel{stroke:#0f2418;stroke-width:.45;opacity:.86}"
      << ".voxel-left{opacity:.66}"
      << ".voxel-right{opacity:.76}"
      << ".voxel-top{opacity:.92}"
      << ".trajectory-path{fill:none;stroke:#00ffff;stroke-width:5;stroke-linejoin:round;stroke-linecap:round}"
      << ".elevation-path{fill:none;stroke:#00ffff;stroke-width:4;stroke-linejoin:round;stroke-linecap:round}"
      << ".start-marker{fill:#00ffff;stroke:#022;stroke-width:3}"
      << ".planned-start-marker{fill:#ff8a00;stroke:#2a1200;stroke-width:3}"
      << ".goal-marker{fill:#00e676;stroke:#021;stroke-width:3}"
      << ".planned-goal-marker{fill:#ffde59;stroke:#2a2100;stroke-width:3}"
      << "</style>\n";
  out << "<rect width=\"100%\" height=\"100%\" fill=\"#030504\"/>\n";
  out << "<text x=\"72\" y=\"46\">Nav3D OctoMap trajectory plot</text>\n";
  out << "<text class=\"small\" x=\"72\" y=\"70\">isometric view: occupied voxels + planned path; "
      << "right: trajectory elevation profile; " << mode_label << "</text>\n";
  out << "<rect class=\"panel\" x=\"" << map_left << "\" y=\"" << map_top
      << "\" width=\"" << map_width << "\" height=\"" << map_height << "\" rx=\"4\"/>\n";
  out << "<rect class=\"panel\" x=\"" << elev_left << "\" y=\"" << elev_top
      << "\" width=\"" << elev_width << "\" height=\"" << elev_height << "\" rx=\"4\"/>\n";

  for (int tick = 1; tick < 6; ++tick) {
    const double x = map_left + map_width * static_cast<double>(tick) / 6.0;
    const double y = map_top + map_height * static_cast<double>(tick) / 6.0;
    out << "<line class=\"grid\" x1=\"" << x << "\" y1=\"" << map_top << "\" x2=\"" << x
        << "\" y2=\"" << (map_top + map_height) << "\"/>\n";
    out << "<line class=\"grid\" x1=\"" << map_left << "\" y1=\"" << y << "\" x2=\""
        << (map_left + map_width) << "\" y2=\"" << y << "\"/>\n";
  }
  for (int tick = 1; tick < 5; ++tick) {
    const double y = elev_top + elev_height * static_cast<double>(tick) / 5.0;
    out << "<line class=\"grid\" x1=\"" << elev_left << "\" y1=\"" << y << "\" x2=\""
        << (elev_left + elev_width) << "\" y2=\"" << y << "\"/>\n";
  }

  out << "<g id=\"isometric-trajectory-view\" data-projection=\"isometric-oblique\">\n";
  auto draw_ordered_occupied = occupied;
  std::sort(draw_ordered_occupied.begin(), draw_ordered_occupied.end(), [](const auto& a, const auto& b) {
    const double da = a.x + a.y + a.z;
    const double db = b.x + b.y + b.z;
    if (std::abs(da - db) > 1e-9) {
      return da < db;
    }
    if (a.z != b.z) {
      return a.z < b.z;
    }
    if (a.y != b.y) {
      return a.y < b.y;
    }
    return a.x < b.x;
  });
  const auto cubeCorner = [&](const nav3d::common::Point3D& center, double dx, double dy, double dz) {
    const double half = resolution * 0.5;
    return nav3d::common::Point3D{
      center.x + dx * half,
      center.y + dy * half,
      center.z + dz * half,
    };
  };
  for (const auto& voxel : draw_ordered_occupied) {
    const double z_norm = std::clamp((voxel.z - bounds.min.z) / span_z, 0.0, 1.0);
    const auto left_color = rgb(8, 75 + static_cast<int>(65 * z_norm), 50 + static_cast<int>(95 * z_norm));
    const auto right_color = rgb(12, 102 + static_cast<int>(80 * z_norm), 62 + static_cast<int>(115 * z_norm));
    const auto top_color = rgb(24, 145 + static_cast<int>(80 * z_norm), 95 + static_cast<int>(125 * z_norm));
    const std::vector<nav3d::common::Point3D> left_face{
      cubeCorner(voxel, -1, -1, -1),
      cubeCorner(voxel, -1, -1, 1),
      cubeCorner(voxel, -1, 1, 1),
      cubeCorner(voxel, -1, 1, -1),
    };
    const std::vector<nav3d::common::Point3D> right_face{
      cubeCorner(voxel, -1, 1, -1),
      cubeCorner(voxel, -1, 1, 1),
      cubeCorner(voxel, 1, 1, 1),
      cubeCorner(voxel, 1, 1, -1),
    };
    const std::vector<nav3d::common::Point3D> top_face{
      cubeCorner(voxel, -1, -1, 1),
      cubeCorner(voxel, 1, -1, 1),
      cubeCorner(voxel, 1, 1, 1),
      cubeCorner(voxel, -1, 1, 1),
    };
    out << "<polygon class=\"occupied-voxel voxel-left\" points=\"" << polygonPoints(left_face)
        << "\" fill=\"" << left_color << "\"><title>voxel " << formatPoint(voxel)
        << "</title></polygon>\n";
    out << "<polygon class=\"occupied-voxel voxel-right\" points=\"" << polygonPoints(right_face)
        << "\" fill=\"" << right_color << "\"><title>voxel " << formatPoint(voxel)
        << "</title></polygon>\n";
    out << "<polygon class=\"occupied-voxel voxel-top\" points=\"" << polygonPoints(top_face)
        << "\" fill=\"" << top_color << "\"><title>voxel " << formatPoint(voxel)
        << "</title></polygon>\n";
  }

  if (!samples.empty()) {
    out << "<polyline class=\"trajectory-path\" points=\"";
    for (const auto& point : samples) {
      const auto projected = project(point);
      out << svgNumber(projected.x) << "," << svgNumber(projected.y) << " ";
    }
    out << "\"/>\n";
  }

  const auto requested_start_projected = project(requested_start);
  out << "<circle class=\"start-marker\" cx=\"" << requested_start_projected.x << "\" cy=\""
      << requested_start_projected.y << "\" r=\"8\"><title>requested_start "
      << formatPoint(requested_start) << "</title></circle>\n";
  if (nav3d::common::distance(planned_start, requested_start) > resolution * 0.25) {
    const auto planned_start_projected = project(planned_start);
    out << "<circle class=\"planned-start-marker\" cx=\"" << planned_start_projected.x << "\" cy=\""
        << planned_start_projected.y << "\" r=\"7\"><title>planned_start "
        << formatPoint(planned_start) << "</title></circle>\n";
  }
  const auto requested_goal_projected = project(requested_goal);
  out << "<circle class=\"goal-marker\" cx=\"" << requested_goal_projected.x << "\" cy=\""
      << requested_goal_projected.y << "\" r=\"8\"><title>requested_goal "
      << formatPoint(requested_goal) << "</title></circle>\n";
  if (partial) {
    const auto planned_goal_projected = project(planned_goal);
    out << "<circle class=\"planned-goal-marker\" cx=\"" << planned_goal_projected.x << "\" cy=\""
        << planned_goal_projected.y << "\" r=\"7\"><title>planned_goal "
        << formatPoint(planned_goal) << "</title></circle>\n";
  }
  out << "</g>\n";

  const double total_length = samples.empty() ? 0.0 : pathLength(samples);
  if (!samples.empty()) {
    out << "<polyline class=\"elevation-path\" points=\"";
    double traveled = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (i > 0) {
        traveled += nav3d::common::distance(samples[i - 1], samples[i]);
      }
      const double x = total_length > 1e-9
        ? elev_left + (traveled / total_length) * elev_width
        : elev_left + elev_width / 2.0;
      out << svgNumber(x) << "," << svgNumber(elevY(samples[i].z)) << " ";
    }
    out << "\"/>\n";
  }

  out << "<text class=\"small\" x=\"" << map_left << "\" y=\"" << (map_top + map_height + 30)
      << "\">X " << svgNumber(bounds.min.x) << " .. " << svgNumber(bounds.max.x)
      << " m, Y " << svgNumber(bounds.min.y) << " .. " << svgNumber(bounds.max.y)
      << " m, occupied voxels " << occupied.size() << "</text>\n";
  out << "<text class=\"small\" x=\"" << elev_left << "\" y=\"" << (elev_top - 18)
      << "\">Z elevation " << svgNumber(bounds.min.z) << " .. "
      << svgNumber(bounds.max.z) << " m</text>\n";
  out << "<text class=\"small\" x=\"" << elev_left << "\" y=\"" << (elev_top + elev_height + 30)
      << "\">path " << svgNumber(total_length) << " m, samples " << samples.size()
      << ", partial=" << (partial ? "true" : "false") << "</text>\n";
  out << "<text class=\"small\" x=\"72\" y=\"724\">cyan=requested start, orange=planned start, green=requested goal"
      << (partial ? ", yellow=shortened planned goal" : "") << "</text>\n";
  out << "</svg>\n";
  return true;
}

void printUsage()
{
  std::cout
    << "Usage: nav3d_octomap_trajectory_demo [options] [map.pcd]\n\n"
    << "Options:\n"
    << "  --start X Y Z | X,Y,Z          requested start, default -13 8 1\n"
    << "  --goal X Y Z | X,Y,Z           requested goal, default 13 8 1\n"
    << "  --map-backend voxel|octomap    planning map backend, default octomap\n"
    << "  --planning-mode 2d|3d          search mode, default 3d\n"
    << "  --search-algorithm astar|jps   path searcher, default astar\n"
    << "  --resolution M                 voxel/OctoMap resolution, default 0.5\n"
    << "  --min-points-per-voxel N       sparse voxel filter, default 1\n"
    << "  --min-cluster-voxels N         isolated cluster filter, default 1\n"
    << "  --pcd-loader builtin|pcl       PCD loader backend, default builtin\n"
    << "  --save-octomap PATH            write cleaned OctoMap .bt\n"
    << "  --save-trajectory-plot PATH    write viewable SVG trajectory plot, default ./nav3d_octomap_trajectory_demo.svg\n"
    << "  --save-trajectory PATH         optional sampled trajectory CSV\n"
    << "  --trajectory-sample-step SEC   trajectory sample interval for CSV/SVG, default 0.05\n"
    << "  --max-fallback-attempts N      maximum EGO fallback attempts, default 8\n"
    << "  --require-full-goal            return non-zero if fallback only reaches a shortened goal\n";
}

Options parseOptions(int argc, char** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else if (arg == "--start") {
      options.start = parsePoint3DArgs(i, argc, argv, arg);
    } else if (arg == "--goal") {
      options.goal = parsePoint3DArgs(i, argc, argv, arg);
    } else if (arg == "--map-backend") {
      options.map_backend = parseMapBackend(nextArg(i, argc, argv, arg));
    } else if (arg == "--planning-mode") {
      options.planning_mode = parsePlanningMode(nextArg(i, argc, argv, arg));
    } else if (arg == "--search-algorithm") {
      options.search_algorithm = parseSearchAlgorithm(nextArg(i, argc, argv, arg));
    } else if (arg == "--resolution") {
      options.resolution = parseDoubleArg(nextArg(i, argc, argv, arg), arg);
    } else if (arg == "--min-points-per-voxel") {
      options.min_points_per_voxel = parseIntArg(nextArg(i, argc, argv, arg), arg);
    } else if (arg == "--min-cluster-voxels") {
      options.min_cluster_voxels = parseIntArg(nextArg(i, argc, argv, arg), arg);
    } else if (arg == "--pcd-loader") {
      options.pcd_loader_backend = parsePcdLoaderBackend(nextArg(i, argc, argv, arg));
    } else if (arg == "--insert-free-space-rays") {
      options.insert_free_space_rays = true;
    } else if (arg == "--sensor-origin") {
      options.sensor_origin = parsePoint3DArgs(i, argc, argv, arg);
    } else if (arg == "--save-octomap") {
      options.save_octomap_path = nextArg(i, argc, argv, arg);
    } else if (arg == "--save-trajectory") {
      options.save_trajectory_path = nextArg(i, argc, argv, arg);
    } else if (arg == "--save-trajectory-plot" || arg == "--save-trajectory-image") {
      options.save_trajectory_plot_path = nextArg(i, argc, argv, arg);
    } else if (arg == "--trajectory-sample-step") {
      options.trajectory_sample_step = parseDoubleArg(nextArg(i, argc, argv, arg), arg);
    } else if (arg == "--max-optimizer-iterations") {
      options.max_optimizer_iterations = parseIntArg(nextArg(i, argc, argv, arg), arg);
    } else if (arg == "--max-fallback-attempts") {
      options.max_fallback_attempts = parseIntArg(nextArg(i, argc, argv, arg), arg);
    } else if (arg == "--require-full-goal") {
      options.require_full_goal = true;
    } else if (!arg.empty() && arg[0] == '-') {
      throw std::invalid_argument("unknown option: " + arg);
    } else {
      options.pcd_path = arg;
    }
  }

  if (options.resolution <= 0.0) {
    throw std::invalid_argument("--resolution must be positive");
  }
  if (options.min_points_per_voxel <= 0) {
    throw std::invalid_argument("--min-points-per-voxel must be positive");
  }
  if (options.min_cluster_voxels <= 0) {
    throw std::invalid_argument("--min-cluster-voxels must be positive");
  }
  if (options.trajectory_sample_step <= 0.0) {
    throw std::invalid_argument("--trajectory-sample-step must be positive");
  }
  if (options.max_optimizer_iterations <= 0) {
    throw std::invalid_argument("--max-optimizer-iterations must be positive");
  }
  if (options.max_fallback_attempts <= 0) {
    throw std::invalid_argument("--max-fallback-attempts must be positive");
  }
  if (options.insert_free_space_rays && !options.sensor_origin.has_value()) {
    throw std::invalid_argument("--sensor-origin is required with --insert-free-space-rays");
  }
  if (options.save_trajectory_plot_path.empty()) {
    options.save_trajectory_plot_path = "nav3d_octomap_trajectory_demo.svg";
  }
  return options;
}

}  // namespace

int main(int argc, char** argv)
{
  Options options;
  try {
    options = parseOptions(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  nav3d::map::MapBuildConfig config;
  config.pcd_path = options.pcd_path;
  config.preprocessor.resolution = options.resolution;
  config.preprocessor.min_points_per_voxel = options.min_points_per_voxel;
  config.preprocessor.min_cluster_voxels = options.min_cluster_voxels;
  config.insert_free_space_rays = options.insert_free_space_rays;
  config.sensor_origin = options.sensor_origin;

  nav3d::map::VoxelGridMap voxel_map = makeFallbackVoxelMap(options.resolution);
  nav3d::map::OctomapManager octomap = makeOctomapFromVoxelMap(voxel_map);
  std::size_t raw_points = 0;
  std::size_t filtered_points = voxel_map.occupiedCells().size();

  if (!options.pcd_path.empty()) {
    auto loaded_cloud = loadPointCloud(options.pcd_loader_backend, options.pcd_path);
    if (!loaded_cloud.ok()) {
      std::cerr << loaded_cloud.error() << "\n";
      return 1;
    }
    raw_points = loaded_cloud.value().points.size();

    auto built_map = nav3d::map::MapBuilder::buildVoxelMapFromPointCloud(
      loaded_cloud.value(),
      config);
    filtered_points = built_map.filtered_point_count;
    voxel_map = std::move(built_map.map);

    auto built_octomap = nav3d::map::OctomapManager::buildFromPointCloud(
      loaded_cloud.value(),
      config);
    if (!built_octomap.ok()) {
      std::cerr << built_octomap.error() << "\n";
      return 1;
    }
    octomap = std::move(built_octomap.value().map);
  } else {
    std::cout << "using built-in fallback corridor map; pass a .pcd path to load point cloud input\n";
  }

  if (!options.save_octomap_path.empty()) {
    const auto saved = octomap.saveBinary(options.save_octomap_path);
    if (!saved.ok()) {
      std::cerr << saved.error() << "\n";
      return 1;
    }
    std::cout << "saved_octomap=" << options.save_octomap_path
              << " occupied_leafs=" << octomap.occupiedLeafCount() << "\n";
  }

  const nav3d::map::IMap& planning_map =
    options.map_backend == MapBackend::Octomap
      ? static_cast<const nav3d::map::IMap&>(octomap)
      : static_cast<const nav3d::map::IMap&>(voxel_map);

  bool success = false;
  nav3d::planner::EgoPlanStatus plan_status = nav3d::planner::EgoPlanStatus::SearchFailed;
  int attempts = 1;
  nav3d::common::Point3D requested_goal = options.goal;
  nav3d::common::Point3D planned_start = options.start;
  nav3d::common::Point3D planned_goal = options.goal;
  nav3d::planner::SearchStatus search_status = nav3d::planner::SearchStatus::NoPath;
  nav3d::common::Path3D search_path;
  int search_iterations = 0;
  double search_path_m = 0.0;
  double trajectory_duration_s = 0.0;
  bool collision = false;
  std::size_t fallback_history_count = 0;
  nav3d::planner::UniformBspline bspline_trajectory;
  std::vector<nav3d::common::Point3D> samples;

  nav3d::planner::EgoPlannerCoreConfig planner_config;
  planner_config.search.mode = options.planning_mode;
  planner_config.search.algorithm = options.search_algorithm;
  planner_config.search.allow_diagonal = true;
  planner_config.search.max_iterations = options.max_search_iterations;
  planner_config.max_fallback_attempts = options.max_fallback_attempts;
  planner_config.optimizer.max_iterations = options.max_optimizer_iterations;
  planner_config.optimizer.rebound_search = planner_config.search;
  planner_config.trajectory_sample_step_seconds = options.trajectory_sample_step;
  std::unique_ptr<nav3d::planner::EgoPlannerCore> planner;
  if (options.planning_mode == nav3d::planner::PlanningMode::Mode3D) {
    planner = std::make_unique<nav3d::planner::EgoPlannerCore>(
      planner_config,
      std::make_shared<GroundSupportedSearcher>(options));
  } else {
    planner = std::make_unique<nav3d::planner::EgoPlannerCore>(planner_config);
  }

  const auto result = planner->planWithFallbacks(planning_map, options.start, options.goal);
  success = result.success;
  plan_status = result.status;
  attempts = result.attempts;
  requested_goal = result.requested_goal;
  planned_goal = result.planned_goal;
  search_status = result.search.status;
  search_path = result.search.path;
  search_iterations = result.search.iterations;
  search_path_m = pathLength(result.search.path);
  trajectory_duration_s = result.success ? result.trajectory.duration() : 0.0;
  collision = result.collision.in_collision;
  fallback_history_count = result.fallback_history.size();
  if (result.success) {
    if (!result.search.path.empty()) {
      planned_start = result.search.path.front();
    }
    bspline_trajectory = result.trajectory;
    samples = sampleTrajectory(result.trajectory, options.trajectory_sample_step);
  }

  const double endpoint_error = success
    ? nav3d::common::distance(planned_goal, requested_goal)
    : nav3d::common::distance(options.start, options.goal);
  const bool partial = success && endpoint_error > snappedGoalTolerance(planning_map.getResolution());

  std::cout << "loaded_map pcd_loader=" << toString(options.pcd_loader_backend)
            << " map_backend=" << toString(options.map_backend)
            << " planning_mode=" << toString(options.planning_mode)
            << " pcd_path=" << (options.pcd_path.empty() ? "<fallback>" : options.pcd_path)
            << " raw_points=" << raw_points
            << " filtered_points=" << filtered_points
            << " occupied_voxels=" << voxel_map.occupiedCells().size()
            << " occupied_leafs=" << octomap.occupiedLeafCount()
            << " resolution=" << planning_map.getResolution() << "\n";

  std::cout << "trajectory_result success=" << (success ? "true" : "false")
            << " status=" << toString(plan_status)
            << " planning_mode=" << toString(options.planning_mode)
            << " attempts=" << attempts
            << " partial=" << (partial ? "true" : "false")
            << " search_status=" << toString(search_status)
            << " search_waypoints=" << search_path.size()
            << " search_iterations=" << search_iterations
            << " search_path_m=" << search_path_m
            << " search_z_span_m=" << zSpan(search_path)
            << " optimization_success=" << (result.optimization.success ? "true" : "false")
            << " optimization_rebound=" << (result.optimization.used_rebound ? "true" : "false")
            << " rebound_segments=" << result.optimization.rebound_segments
            << " optimization_initial_cost=" << result.optimization.initial_cost
            << " optimization_final_cost=" << result.optimization.final_cost
            << " trajectory_duration_s=" << trajectory_duration_s
            << " trajectory_samples=" << samples.size()
            << " requested_start=" << formatPoint(options.start)
            << " planned_start=" << formatPoint(planned_start)
            << " requested_goal=" << formatPoint(requested_goal)
            << " planned_goal=" << formatPoint(planned_goal)
            << " endpoint_error_m=" << endpoint_error
            << " z_span_m=" << zSpan(samples)
            << " collision=" << (collision ? "true" : "false")
            << " collision_time_s="
            << (result.collision.first_collision_time.has_value()
                  ? std::to_string(*result.collision.first_collision_time)
                  : std::string("none"))
            << " collision_point="
            << (result.collision.first_collision_point.has_value()
                  ? formatPoint(*result.collision.first_collision_point)
                  : std::string("none"))
            << "\n";

  if (fallback_history_count > 0) {
    std::cout << "fallback_history_count=" << fallback_history_count << "\n";
  }

  if (success && !options.save_trajectory_path.empty()) {
    const bool saved = writeTrajectoryCsv(
      options.save_trajectory_path,
      bspline_trajectory,
      options.trajectory_sample_step);
    if (!saved) {
      return 1;
    }
    std::cout << "saved_trajectory=" << options.save_trajectory_path
              << " samples=" << samples.size() << "\n";
  }

  if (success && !options.save_trajectory_plot_path.empty()) {
    if (!writeTrajectorySvg(
          options.save_trajectory_plot_path,
          std::string("mode=") + toString(options.planning_mode),
          voxel_map,
          samples,
          options.start,
          planned_start,
          requested_goal,
          planned_goal,
          partial)) {
      return 1;
    }
    std::cout << "saved_trajectory_plot=" << options.save_trajectory_plot_path
              << " samples=" << samples.size()
              << " occupied_voxels=" << voxel_map.occupiedCells().size() << "\n";
  }

  if (!success) {
    return 2;
  }
  if (options.require_full_goal && partial) {
    return 3;
  }
  return 0;
}
