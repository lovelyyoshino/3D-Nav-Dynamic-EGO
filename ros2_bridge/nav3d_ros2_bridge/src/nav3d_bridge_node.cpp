#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <exception>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef NAV3D_HAS_OCTOMAP_MSGS
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#endif

#include "nav3d/collision/inflation_layer.h"
#include "nav3d/collision/trajectory_checker.h"
#include "nav3d/controller/safety_monitor.h"
#include "nav3d/controller/trajectory_tracker.h"
#include "nav3d/common/types.h"
#include "nav3d/map/local_grid.h"
#include "nav3d/map/map_2d_projection.h"
#include "nav3d/map/map_builder.h"
#include "nav3d/map/map_composite.h"
#include "nav3d/map/octomap_manager.h"
#include "nav3d/map/pcd_loader.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/ego_planner_core.h"
#include "nav3d/planner/path_searching/astar_3d.h"

#ifdef NAV3D_HAS_PCL
#include "pcl_pcd_loader.h"
#endif

namespace {

enum class PcdLoaderBackend {
  Builtin,
  Pcl,
};

enum class PlanningTraversabilityMode {
  Uav,
  Ground,
};

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

struct GroundSearchOptions {
  nav3d::planner::PlanningMode mode = nav3d::planner::PlanningMode::Mode3D;
  int max_iterations = 200000;
  int snap_radius_cells = 12;
  int support_xy_radius_cells = 1;
  int support_depth_cells = 1;
  double robot_radius = 0.25;
  bool allow_diagonal = true;
  bool strict_direct_ground_support = false;
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

nav3d::common::Point3D toPoint(const geometry_msgs::msg::PoseStamped& msg)
{
  return {
    msg.pose.position.x,
    msg.pose.position.y,
    msg.pose.position.z,
  };
}

nav3d::common::Point3D toPoint(const geometry_msgs::msg::PointStamped& msg)
{
  return {
    msg.point.x,
    msg.point.y,
    msg.point.z,
  };
}

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string trim(const std::string& value)
{
  auto begin = value.begin();
  while (begin != value.end() &&
         std::isspace(static_cast<unsigned char>(*begin))) {
    ++begin;
  }
  auto end = value.end();
  while (end != begin &&
         std::isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  return {begin, end};
}

double pathLength(const nav3d::common::Path3D& path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += nav3d::common::distance(path[i - 1], path[i]);
  }
  return length;
}

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
  const GroundSearchOptions& options)
{
  const double resolution = map.getResolution();
  if (options.strict_direct_ground_support) {
    const auto below = groundIndexToWorld({idx.x, idx.y, idx.z - 1}, resolution);
    return map.isInBounds(below) && map.isOccupied(below);
  }

  for (int dz = 1; dz <= std::max(1, options.support_depth_cells); ++dz) {
    for (int dx = -options.support_xy_radius_cells; dx <= options.support_xy_radius_cells; ++dx) {
      for (int dy = -options.support_xy_radius_cells; dy <= options.support_xy_radius_cells; ++dy) {
        const auto below = groundIndexToWorld({idx.x + dx, idx.y + dy, idx.z - dz}, resolution);
        if (map.isInBounds(below) && map.isOccupied(below)) {
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
  const GroundSearchOptions& options)
{
  const double resolution = map.getResolution();
  const int radius_cells = std::max(0, static_cast<int>(std::ceil(options.robot_radius / resolution)));
  const double radius_sq = options.robot_radius * options.robot_radius;
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dz = 0; dz <= radius_cells; ++dz) {
        const double dist_x = static_cast<double>(dx) * resolution;
        const double dist_y = static_cast<double>(dy) * resolution;
        const double dist_z = static_cast<double>(dz) * resolution;
        if (dist_x * dist_x + dist_y * dist_y + dist_z * dist_z > radius_sq + 1e-12) {
          continue;
        }
        const auto nearby = groundIndexToWorld({idx.x + dx, idx.y + dy, idx.z + dz}, resolution);
        if (map.isInBounds(nearby) && map.isOccupied(nearby)) {
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
  const GroundSearchOptions& options)
{
  const auto point = groundIndexToWorld(idx, map.getResolution());
  return map.isFree(point) &&
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
  const GroundSearchOptions& options)
{
  if (isGroundTraversable(map, requested, options)) {
    return requested;
  }

  std::optional<GroundSnapCandidate> best;
  const int radius = options.snap_radius_cells;
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
  const GroundSearchOptions& options)
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
  const GroundSearchOptions& options)
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
  const auto directions = makeGroundDirections(options.mode, options.allow_diagonal);
  int iterations = 0;

  while (!open_set.empty() && iterations < options.max_iterations) {
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
      return {nav3d::planner::SearchStatus::Success, std::move(path), iterations, cost};
    }

    for (const auto& direction : directions) {
      const GroundIndex neighbor{
        current.idx.x + direction.x,
        current.idx.y + direction.y,
        current.idx.z + direction.z,
      };
      if (closed_set.find(neighbor) != closed_set.end() ||
          !canMoveGround(map, current.idx, direction, options)) {
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
    iterations >= options.max_iterations
      ? nav3d::planner::SearchStatus::IterationLimit
      : nav3d::planner::SearchStatus::NoPath,
    {},
    iterations,
    0.0,
  };
}

class GroundTraversabilityMap final : public nav3d::map::IMap {
public:
  GroundTraversabilityMap(
    const nav3d::map::IMap& base,
    GroundSearchOptions options)
    : base_(base),
      options_(std::move(options)),
      resolution_(base.getResolution()),
      bounds_(base.getBounds())
  {
    if (resolution_ <= 0.0) {
      throw std::invalid_argument("GroundTraversabilityMap requires a positive resolution");
    }
  }

  bool isOccupied(const nav3d::common::Point3D& point) const override
  {
    return base_.isOccupied(point);
  }

  bool isFree(const nav3d::common::Point3D& point) const override
  {
    return isCellTraversable(worldToGroundIndex(point, resolution_));
  }

  double getDistance(const nav3d::common::Point3D& point) const override
  {
    return base_.getDistance(point);
  }

  bool isInBounds(const nav3d::common::Point3D& point) const override
  {
    return base_.isInBounds(point);
  }

  double getResolution() const override
  {
    return resolution_;
  }

  nav3d::common::BoundingBox getBounds() const override
  {
    return bounds_;
  }

private:
  bool isInsideIndex(const GroundIndex& idx) const
  {
    return base_.isInBounds(groundIndexToWorld(idx, resolution_));
  }

  bool isBaseOccupiedCell(const GroundIndex& idx) const
  {
    return isInsideIndex(idx) && base_.isOccupied(groundIndexToWorld(idx, resolution_));
  }

  bool hasGroundSupportAt(const GroundIndex& idx) const
  {
    if (options_.strict_direct_ground_support) {
      const GroundIndex below{idx.x, idx.y, idx.z - 1};
      return isInsideIndex(below) && isBaseOccupiedCell(below);
    }

    for (int dz = 1; dz <= std::max(1, options_.support_depth_cells); ++dz) {
      for (int dx = -options_.support_xy_radius_cells; dx <= options_.support_xy_radius_cells; ++dx) {
        for (int dy = -options_.support_xy_radius_cells; dy <= options_.support_xy_radius_cells; ++dy) {
          const GroundIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
          if (isInsideIndex(below) && isBaseOccupiedCell(below)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  bool hasBodyClearanceAt(const GroundIndex& idx) const
  {
    const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(options_.robot_radius / resolution_)));
    const double radius_sq = options_.robot_radius * options_.robot_radius;
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dz = 0; dz <= radius_cells; ++dz) {
          const double dist_x = static_cast<double>(dx) * resolution_;
          const double dist_y = static_cast<double>(dy) * resolution_;
          const double dist_z = static_cast<double>(dz) * resolution_;
          if (dist_x * dist_x + dist_y * dist_y + dist_z * dist_z > radius_sq + 1e-12) {
            continue;
          }

          const GroundIndex nearby{idx.x + dx, idx.y + dy, idx.z + dz};
          if (isBaseOccupiedCell(nearby)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  bool isCellTraversable(const GroundIndex& idx) const
  {
    if (!isInsideIndex(idx) || isBaseOccupiedCell(idx)) {
      return false;
    }
    if (!hasGroundSupportAt(idx)) {
      return false;
    }
    return hasBodyClearanceAt(idx);
  }

  const nav3d::map::IMap& base_;
  GroundSearchOptions options_;
  double resolution_ = 1.0;
  nav3d::common::BoundingBox bounds_;
};

class GroundSupportedSearcher final : public nav3d::planner::IPathSearcher {
public:
  explicit GroundSupportedSearcher(GroundSearchOptions options)
    : options_(std::move(options))
  {
  }

  nav3d::planner::SearchResult search(
    const nav3d::map::IMap& map,
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal,
    const nav3d::planner::SearchOptions& search_options) const override
  {
    auto options = options_;
    options.mode = search_options.mode;
    options.max_iterations = search_options.max_iterations;
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
  GroundSearchOptions options_;
};

nav3d::planner::PlanningMode parsePlanningMode(const std::string& raw_mode)
{
  const std::string mode = toLower(raw_mode);
  if (mode == "2d") {
    return nav3d::planner::PlanningMode::Mode2D;
  }
  if (mode == "3d") {
    return nav3d::planner::PlanningMode::Mode3D;
  }
  throw std::invalid_argument("planning.mode must be '2d' or '3d'");
}

PlanningTraversabilityMode parsePlanningTraversabilityMode(const std::string& raw_mode)
{
  const std::string mode = toLower(raw_mode);
  if (mode == "uav" || mode == "air" || mode == "free_space" || mode == "free-space") {
    return PlanningTraversabilityMode::Uav;
  }
  if (mode == "ground" || mode == "grounded" || mode == "terrain") {
    return PlanningTraversabilityMode::Ground;
  }
  throw std::invalid_argument("planning.traversability must be 'uav' or 'ground'");
}

const char* planningTraversabilityModeName(PlanningTraversabilityMode mode)
{
  switch (mode) {
    case PlanningTraversabilityMode::Uav:
      return "uav";
    case PlanningTraversabilityMode::Ground:
      return "ground";
  }
  return "unknown";
}

nav3d::planner::SearchAlgorithm parseSearchAlgorithm(const std::string& raw_algorithm)
{
  const std::string algorithm = toLower(raw_algorithm);
  if (algorithm == "astar" || algorithm == "a_star" || algorithm == "a*") {
    return nav3d::planner::SearchAlgorithm::AStar;
  }
  if (algorithm == "jps") {
    return nav3d::planner::SearchAlgorithm::Jps;
  }
  throw std::invalid_argument("planning.search_algorithm must be 'astar' or 'jps'");
}

nav3d::planner::DynamicFeasibilityMode parseDynamicFeasibilityMode(
  const std::string& raw_mode)
{
  const std::string mode = toLower(raw_mode);
  if (mode == "sampled" || mode == "sampling") {
    return nav3d::planner::DynamicFeasibilityMode::Sampled;
  }
  if (mode == "analytic" || mode == "analytic_bounds" || mode == "bounds") {
    return nav3d::planner::DynamicFeasibilityMode::AnalyticBounds;
  }
  if (mode == "analytic_exact" || mode == "exact" || mode == "continuous") {
    return nav3d::planner::DynamicFeasibilityMode::AnalyticExact;
  }
  throw std::invalid_argument(
    "planning.dynamic_feasibility_mode must be 'sampled', 'analytic_bounds', or 'analytic_exact'");
}

PcdLoaderBackend parsePcdLoaderBackend(const std::string& raw_backend)
{
  const std::string backend = toLower(raw_backend);
  if (backend == "builtin") {
    return PcdLoaderBackend::Builtin;
  }
  if (backend == "pcl") {
    return PcdLoaderBackend::Pcl;
  }
  throw std::invalid_argument("map.pcd_loader must be 'builtin' or 'pcl'");
}

const char* pcdLoaderBackendName(PcdLoaderBackend backend)
{
  switch (backend) {
    case PcdLoaderBackend::Builtin:
      return "builtin";
    case PcdLoaderBackend::Pcl:
      return "pcl";
  }
  return "unknown";
}

nav3d::controller::CommandFrame parseCommandFrame(const std::string& raw_frame)
{
  const std::string frame = toLower(raw_frame);
  if (frame == "map") {
    return nav3d::controller::CommandFrame::Map;
  }
  if (frame == "body" || frame == "base" || frame == "base_link") {
    return nav3d::controller::CommandFrame::Body;
  }
  throw std::invalid_argument("controller.command_frame must be 'body' or 'map'");
}

nav3d::controller::MotionModelType parseMotionModelType(const std::string& raw_model)
{
  const std::string model = toLower(raw_model);
  if (model == "differential" || model == "differential_drive" || model == "diff_drive") {
    return nav3d::controller::MotionModelType::DifferentialDrive;
  }
  if (model == "omni" || model == "holonomic") {
    return nav3d::controller::MotionModelType::Omni;
  }
  if (model == "uav" || model == "quadrotor") {
    return nav3d::controller::MotionModelType::Uav;
  }
  if (model == "ackermann" || model == "car") {
    return nav3d::controller::MotionModelType::Ackermann;
  }
  throw std::invalid_argument(
    "controller.motion_model must be 'differential', 'omni', 'uav', or 'ackermann'");
}

std::string statusToString(nav3d::planner::EgoPlanStatus status)
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

std::string searchStatusToString(nav3d::planner::SearchStatus status)
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

double snappedGoalTolerance(double resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return 1e-6;
  }
  return std::sqrt(3.0) * resolution * 0.5 + 1e-6;
}

bool isPartialPlan(const nav3d::planner::EgoPlanResult& result, double resolution)
{
  return nav3d::common::distance(result.requested_goal, result.planned_goal) >
         snappedGoalTolerance(resolution);
}

void appendPlanEndpointStatus(
  std::ostringstream& status,
  const nav3d::planner::EgoPlanResult& result,
  double resolution)
{
  status << " planned_goal="
         << result.planned_goal.x << ","
         << result.planned_goal.y << ","
         << result.planned_goal.z
         << " requested_goal="
         << result.requested_goal.x << ","
         << result.requested_goal.y << ","
         << result.requested_goal.z
         << " partial=" << (isPartialPlan(result, resolution) ? "true" : "false");
}

void appendPlanFailureDebug(
  std::ostringstream& status,
  const nav3d::planner::EgoPlanResult& result,
  double resolution)
{
  status << " search_status=" << searchStatusToString(result.search.status)
         << " search_waypoints=" << result.search.path.size()
         << " search_iterations=" << result.search.iterations
         << " optimization_success=" << (result.optimization.success ? "true" : "false")
         << " optimization_rebound=" << (result.optimization.used_rebound ? "true" : "false")
         << " rebound_segments=" << result.optimization.rebound_segments;
  appendPlanEndpointStatus(status, result, resolution);
  if (result.collision.first_collision_time.has_value()) {
    status << " collision_time=" << *result.collision.first_collision_time;
  }
  if (result.collision.first_collision_point.has_value()) {
    const auto& point = *result.collision.first_collision_point;
    status << " collision_point="
           << point.x << ","
           << point.y << ","
           << point.z;
  }
}

}  // namespace

class Nav3DBridgeNode final : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using GetPlan = nav_msgs::srv::GetPlan;

  Nav3DBridgeNode() : Node("nav3d_bridge_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const std::string clicked_point_topic =
      declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
    publish_sample_dt_ = declare_parameter<double>("trajectory.sample_dt", 0.05);
    if (publish_sample_dt_ <= 0.0) {
      throw std::invalid_argument("trajectory.sample_dt must be positive");
    }

    auto latched_visualization_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latched_visualization_qos.transient_local().reliable();

    status_pub_ = create_publisher<std_msgs::msg::String>("/nav3d/status", 10);
    trajectory_pub_ =
      create_publisher<nav_msgs::msg::Path>("/nav3d/trajectory", latched_visualization_qos);
    trajectory_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav3d/trajectory_marker",
      latched_visualization_qos);
    start_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav3d/start_marker",
      latched_visualization_qos);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav3d/goal_marker",
      latched_visualization_qos);
    occupied_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/nav3d/occupied_grid",
      latched_visualization_qos);
    planning_occupied_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/nav3d/planning_occupied_markers",
      latched_visualization_qos);
#ifdef NAV3D_HAS_OCTOMAP_MSGS
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/octomap",
      latched_visualization_qos);
#endif
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    start_ = {
      declare_parameter<double>("start.x", 0.0),
      declare_parameter<double>("start.y", 0.0),
      declare_parameter<double>("start.z", 0.0),
    };
    occupancy_grid_max_cells_ =
      declare_parameter<int>("map.occupancy_grid_max_cells", occupancy_grid_max_cells_);
    occupancy_grid_min_z_ =
      declare_parameter<double>("map.occupancy_grid_min_z", occupancy_grid_min_z_);
    occupancy_grid_max_z_ =
      declare_parameter<double>("map.occupancy_grid_max_z", occupancy_grid_max_z_);
    if (!std::isfinite(occupancy_grid_min_z_) || !std::isfinite(occupancy_grid_max_z_) ||
        occupancy_grid_max_z_ < occupancy_grid_min_z_) {
      throw std::invalid_argument(
        "map.occupancy_grid_min_z/max_z must be finite and max_z must be >= min_z");
    }

    map_build_config_ = readMapBuildConfig();
    map_ = std::make_unique<nav3d::map::VoxelGridMap>(buildMap(map_build_config_));
    octomap_ = std::make_unique<nav3d::map::OctomapManager>(buildOctomap(map_build_config_));
    local_grid_ray_clearing_enabled_ =
      declare_parameter<bool>("local_grid.ray_clearing.enabled", true);
    local_grid_enabled_ = declare_parameter<bool>("local_grid.enabled", true);
    local_grid_resolution_ =
      declare_parameter<double>("local_grid.resolution", map_->getResolution());
    if (local_grid_resolution_ <= 0.0) {
      throw std::invalid_argument("local_grid.resolution must be positive");
    }
    local_grid_retention_sec_ =
      declare_parameter<double>("local_grid.retention_sec", local_grid_retention_sec_);
    local_grid_max_retained_frames_ =
      declare_parameter<int>("local_grid.max_retained_frames", local_grid_max_retained_frames_);
    local_grid_max_observation_age_sec_ =
      declare_parameter<double>("local_grid.max_observation_age_sec", local_grid_max_observation_age_sec_);
    local_grid_max_future_stamp_sec_ =
      declare_parameter<double>("local_grid.max_future_stamp_sec", local_grid_max_future_stamp_sec_);
    if (local_grid_retention_sec_ < 0.0) {
      throw std::invalid_argument("local_grid.retention_sec must be non-negative");
    }
    if (local_grid_max_retained_frames_ <= 0) {
      throw std::invalid_argument("local_grid.max_retained_frames must be positive");
    }
    if (local_grid_max_observation_age_sec_ < 0.0) {
      throw std::invalid_argument("local_grid.max_observation_age_sec must be non-negative");
    }
    if (local_grid_max_future_stamp_sec_ < 0.0) {
      throw std::invalid_argument("local_grid.max_future_stamp_sec must be non-negative");
    }
    if (local_grid_enabled_) {
      local_grid_ = makeLocalGrid();
    }
    auto planner_config = makePlannerConfig();
    if (groundTraversabilityEnabled()) {
      planner_ = std::make_unique<nav3d::planner::EgoPlannerCore>(
        planner_config,
        std::make_shared<GroundSupportedSearcher>(ground_search_options_));
    } else {
      planner_ = std::make_unique<nav3d::planner::EgoPlannerCore>(planner_config);
    }
    publishStaticMapVisualization();
    if (declare_parameter<bool>("safety.enabled", true)) {
      safety_monitor_ = makeSafetyMonitor();
    }
    configureController();
    if (controller_enabled_) {
      controller_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(controller_control_dt_)),
        [this]() {
          onControllerTimer();
        });
    }
    startStaticVisualizationRepublish();

    if (local_grid_) {
      local_pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/nav3d/local_pointcloud",
        1,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          onLocalPointCloud(*msg);
        });
    }
    map_load_sub_ = create_subscription<std_msgs::msg::String>(
      "/nav3d/load_pcd_path",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        onLoadPcdPath(*msg);
      });
    start_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/nav3d/start",
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        onStart(*msg);
      });
    if (safety_monitor_ || controller_enabled_) {
      current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/nav3d/current_pose",
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          onCurrentPose(*msg);
        });
    }
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/nav3d/goal",
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        onGoal(*msg);
      });
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic,
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        onClickedPoint(*msg);
      });
    plan_service_ = create_service<GetPlan>(
      "/nav3d/plan_path",
      [this](
        const std::shared_ptr<GetPlan::Request> request,
        std::shared_ptr<GetPlan::Response> response) {
        onPlanPath(request, response);
      });
    navigate_action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "/nav3d/navigate_to_pose",
      [this](
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal) {
        return handleNavigateGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<NavigateGoalHandle> goal_handle) {
        return handleNavigateCancel(goal_handle);
      },
      [this](const std::shared_ptr<NavigateGoalHandle> goal_handle) {
        handleNavigateAccepted(goal_handle);
      });

    RCLCPP_INFO(
      get_logger(),
      "nav3d bridge ready: frame_id=%s start=(%.3f, %.3f, %.3f) traversability=%s planning_map=%s",
      frame_id_.c_str(),
      start_.x,
      start_.y,
      start_.z,
      planningTraversabilityModeName(planning_traversability_mode_),
      planningMapBackendName());
  }

private:
  nav3d::map::MapBuildConfig readMapBuildConfig()
  {
    nav3d::map::MapBuildConfig config;
    config.pcd_path = declare_parameter<std::string>("map.pcd_path", "");
    pcd_loader_backend_ =
      parsePcdLoaderBackend(declare_parameter<std::string>("map.pcd_loader", "builtin"));
    config.preprocessor.resolution = declare_parameter<double>("map.resolution", 0.5);
    config.preprocessor.min_points_per_voxel =
      declare_parameter<int>("map.min_points_per_voxel", 1);
    config.preprocessor.min_cluster_voxels =
      declare_parameter<int>("map.min_cluster_voxels", 1);
    config.insert_free_space_rays =
      declare_parameter<bool>("map.insert_free_space_rays", false);
    if (config.insert_free_space_rays) {
      config.sensor_origin = nav3d::common::Point3D{
        declare_parameter<double>("map.sensor_origin.x", start_.x),
        declare_parameter<double>("map.sensor_origin.y", start_.y),
        declare_parameter<double>("map.sensor_origin.z", start_.z),
      };
    }

    validateMapBuildConfig(config);
    validatePcdLoaderBackend();
    return config;
  }

  void validateMapBuildConfig(const nav3d::map::MapBuildConfig& config) const
  {
    if (config.pcd_path.empty()) {
      throw std::invalid_argument("map.pcd_path parameter is required");
    }
    if (config.preprocessor.resolution <= 0.0) {
      throw std::invalid_argument("map.resolution must be positive");
    }
    if (config.preprocessor.min_points_per_voxel <= 0) {
      throw std::invalid_argument("map.min_points_per_voxel must be positive");
    }
    if (config.preprocessor.min_cluster_voxels <= 0) {
      throw std::invalid_argument("map.min_cluster_voxels must be positive");
    }
    if (config.insert_free_space_rays && !config.sensor_origin.has_value()) {
      throw std::invalid_argument("map.sensor_origin is required when map.insert_free_space_rays is true");
    }
  }

  void validatePcdLoaderBackend() const
  {
#ifndef NAV3D_HAS_PCL
    if (pcd_loader_backend_ == PcdLoaderBackend::Pcl) {
      throw std::invalid_argument("map.pcd_loader=pcl requested but bridge was built without PCL");
    }
#endif
  }

  nav3d::common::Result<nav3d::map::PointCloud> loadPointCloud(
    const nav3d::map::MapBuildConfig& config) const
  {
    if (pcd_loader_backend_ == PcdLoaderBackend::Builtin) {
      return nav3d::map::PcdLoader::load(config.pcd_path);
    }

#ifdef NAV3D_HAS_PCL
    return nav3d::tools::PclPcdLoader::load(config.pcd_path);
#else
    (void)config;
    return nav3d::common::Result<nav3d::map::PointCloud>::failure(
      "map.pcd_loader=pcl requested but bridge was built without PCL");
#endif
  }

  nav3d::map::VoxelGridMap buildMap(const nav3d::map::MapBuildConfig& config)
  {
    validateMapBuildConfig(config);
    validatePcdLoaderBackend();
    auto loaded = loadPointCloud(config);
    if (!loaded.ok()) {
      throw std::runtime_error("failed to load point cloud: " + loaded.error());
    }
    auto result = nav3d::map::MapBuilder::buildVoxelMapFromPointCloud(loaded.value(), config);

    std::ostringstream status;
    status << "map_loaded pcd_loader=" << pcdLoaderBackendName(pcd_loader_backend_)
           << " raw_points=" << result.raw_point_count
           << " filtered_points=" << result.filtered_point_count
           << " occupied_voxels=" << result.occupied_voxel_count
           << " resolution=" << config.preprocessor.resolution;
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    return std::move(result.map);
  }

  nav3d::map::OctomapManager buildOctomap(const nav3d::map::MapBuildConfig& config)
  {
    validateMapBuildConfig(config);
    validatePcdLoaderBackend();
    auto loaded = loadPointCloud(config);
    if (!loaded.ok()) {
      throw std::runtime_error("failed to load point cloud for octomap: " + loaded.error());
    }
    auto result = nav3d::map::OctomapManager::buildFromPointCloud(loaded.value(), config);
    if (!result.ok()) {
      throw std::runtime_error("failed to build octomap: " + result.error());
    }

    std::ostringstream status;
    status << "octomap_loaded pcd_loader=" << pcdLoaderBackendName(pcd_loader_backend_)
           << " raw_points=" << result.value().raw_point_count
           << " filtered_points=" << result.value().filtered_point_count
           << " occupied_leafs=" << result.value().occupied_leaf_count
           << " resolution=" << config.preprocessor.resolution;
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    return std::move(result.value().map);
  }

  std::unique_ptr<nav3d::map::LocalGrid> makeLocalGrid()
  {
    return makeLocalGridForMap(*map_);
  }

  std::unique_ptr<nav3d::map::LocalGrid> makeLocalGridForMap(
    const nav3d::map::VoxelGridMap& map) const
  {
    const auto bounds = map.getBounds();
    if (!bounds.valid) {
      throw std::runtime_error("local grid requires a valid global map bounds");
    }

    return std::make_unique<nav3d::map::LocalGrid>(
      local_grid_resolution_,
      bounds.min,
      bounds.max);
  }

  std::unique_ptr<nav3d::controller::SafetyMonitor> makeSafetyMonitor()
  {
    nav3d::controller::SafetyMonitorConfig config;
    config.trajectory_sample_step_seconds =
      declare_parameter<double>("safety.trajectory_sample_dt", publish_sample_dt_);
    config.emergency_stop_time_horizon =
      declare_parameter<double>("safety.emergency_stop_horizon", config.emergency_stop_time_horizon);
    config.lookahead_time_horizon =
      declare_parameter<double>("safety.lookahead_horizon", config.lookahead_time_horizon);
    safety_replan_cooldown_sec_ =
      declare_parameter<double>("safety.replan_cooldown_sec", safety_replan_cooldown_sec_);
    safety_local_replan_lookahead_distance_ = declare_parameter<double>(
      "safety.local_replan_lookahead_distance", safety_local_replan_lookahead_distance_);
    safety_local_replan_enabled_ = declare_parameter<bool>(
      "safety.local_replan_enabled", safety_local_replan_enabled_);
    safety_local_use_raw_astar_ = declare_parameter<bool>(
      "safety.local_use_raw_astar", safety_local_use_raw_astar_);
    safety_local_astar_max_iterations_ = declare_parameter<int>(
      "safety.local_astar_max_iterations", safety_local_astar_max_iterations_);
    return std::make_unique<nav3d::controller::SafetyMonitor>(config);
  }

  void configureController()
  {
    controller_enabled_ = declare_parameter<bool>("controller.enabled", true);
    if (occupancy_grid_max_cells_ <= 0) {
      throw std::invalid_argument("map.occupancy_grid_max_cells must be positive");
    }

    nav3d::controller::TrajectoryTrackerConfig config;
    config.sample_step_seconds = publish_sample_dt_;
    config.control_dt =
      declare_parameter<double>("controller.control_dt", config.control_dt);
    config.lookahead_time =
      declare_parameter<double>("controller.lookahead_time", config.lookahead_time);
    config.max_linear_speed =
      declare_parameter<double>("controller.max_speed", config.max_linear_speed);
    config.goal_tolerance =
      declare_parameter<double>("controller.goal_tolerance", config.goal_tolerance);
    controller_goal_tolerance_ = config.goal_tolerance;
    config.yaw_gain =
      declare_parameter<double>("controller.yaw_gain", config.yaw_gain);
    config.max_yaw_rate =
      declare_parameter<double>("controller.max_yaw_rate", config.max_yaw_rate);
    config.command_path_sample_step =
      declare_parameter<double>(
        "controller.command_path_sample_step",
        config.command_path_sample_step);
    config.command_frame =
      parseCommandFrame(declare_parameter<std::string>("controller.command_frame", "body"));
    config.motion_model.type =
      parseMotionModelType(declare_parameter<std::string>("controller.motion_model", "omni"));
    config.motion_model.max_linear_acceleration =
      declare_parameter<double>(
        "controller.max_linear_acceleration",
        config.motion_model.max_linear_acceleration);
    config.motion_model.min_turning_radius =
      declare_parameter<double>(
        "controller.min_turning_radius",
        config.motion_model.min_turning_radius);
    config.motion_model.max_yaw_acceleration =
      declare_parameter<double>(
        "controller.max_yaw_acceleration",
        config.motion_model.max_yaw_acceleration);
    trajectory_tracker_ =
      std::make_unique<nav3d::controller::TrajectoryTracker>(config);
    controller_control_dt_ = trajectory_tracker_->config().control_dt;
  }

  nav3d::planner::EgoPlannerCoreConfig makePlannerConfig()
  {
    nav3d::planner::EgoPlannerCoreConfig config;
    config.search.algorithm =
      parseSearchAlgorithm(declare_parameter<std::string>("planning.search_algorithm", "astar"));
    config.search.mode =
      parsePlanningMode(declare_parameter<std::string>("planning.mode", "3d"));
    config.search.allow_diagonal =
      declare_parameter<bool>("planning.allow_diagonal", true);
    config.search.max_iterations =
      declare_parameter<int>("planning.max_iterations", config.search.max_iterations);
    ground_search_options_.mode = config.search.mode;
    ground_search_options_.max_iterations = config.search.max_iterations;
    planning_traversability_mode_ =
      parsePlanningTraversabilityMode(
        declare_parameter<std::string>("planning.traversability", "uav"));
    planning_use_octomap_map_ =
      declare_parameter<bool>("planning.use_octomap_map", planning_use_octomap_map_);
    ground_search_options_.snap_radius_cells =
      declare_parameter<int>("planning.ground_snap_radius_cells", ground_search_options_.snap_radius_cells);
    ground_search_options_.support_xy_radius_cells =
      declare_parameter<int>(
        "planning.ground_support_xy_radius_cells",
        ground_search_options_.support_xy_radius_cells);
    ground_search_options_.support_depth_cells =
      declare_parameter<int>(
        "planning.ground_support_depth_cells",
        ground_search_options_.support_depth_cells);
    ground_search_options_.robot_radius =
      declare_parameter<double>("planning.ground_robot_radius", ground_search_options_.robot_radius);
    ground_search_options_.strict_direct_ground_support =
      declare_parameter<bool>(
        "planning.ground_strict_direct_support",
        ground_search_options_.strict_direct_ground_support);
    uav_endpoint_snap_radius_cells_ =
      declare_parameter<int>(
        "planning.uav_endpoint_snap_radius_cells",
        uav_endpoint_snap_radius_cells_);
    config.trajectory_sample_step_seconds = publish_sample_dt_;
    config.optimizer.interval =
      declare_parameter<double>("optimizer.interval", config.optimizer.interval);
    config.max_fallback_attempts =
      declare_parameter<int>("fallback.max_attempts", config.max_fallback_attempts);
    config.enable_dynamic_feasibility_check =
      declare_parameter<bool>(
        "planning.enable_dynamic_feasibility_check",
        config.enable_dynamic_feasibility_check);
    config.dynamic_feasibility_mode =
      parseDynamicFeasibilityMode(
        declare_parameter<std::string>(
          "planning.dynamic_feasibility_mode",
          "sampled"));
    config.dynamic_limits.max_velocity =
      declare_parameter<double>(
        "planning.max_velocity",
        config.dynamic_limits.max_velocity);
    config.dynamic_limits.max_acceleration =
      declare_parameter<double>(
        "planning.max_acceleration",
        config.dynamic_limits.max_acceleration);
    config.feasibility_sample_step_seconds =
      declare_parameter<double>(
        "planning.feasibility_sample_dt",
        config.feasibility_sample_step_seconds);
    config.max_dynamic_time_scale =
      declare_parameter<double>(
        "planning.max_dynamic_time_scale",
        config.max_dynamic_time_scale);
    config.enable_initial_time_allocation =
      declare_parameter<bool>(
        "planning.enable_initial_time_allocation",
        config.enable_initial_time_allocation);
    config.min_time_allocation_interval =
      declare_parameter<double>(
        "planning.min_time_allocation_interval",
        config.min_time_allocation_interval);
    planning_inflation_radius_ =
      declare_parameter<double>("planning.inflation_radius", planning_inflation_radius_);

    if (config.search.max_iterations <= 0) {
      throw std::invalid_argument("planning.max_iterations must be positive");
    }
    if (ground_search_options_.snap_radius_cells <= 0) {
      throw std::invalid_argument("planning.ground_snap_radius_cells must be positive");
    }
    if (ground_search_options_.support_xy_radius_cells < 0) {
      throw std::invalid_argument("planning.ground_support_xy_radius_cells must be non-negative");
    }
    if (ground_search_options_.support_depth_cells <= 0) {
      throw std::invalid_argument("planning.ground_support_depth_cells must be positive");
    }
    if (!std::isfinite(ground_search_options_.robot_radius) ||
        ground_search_options_.robot_radius < 0.0) {
      throw std::invalid_argument("planning.ground_robot_radius must be non-negative and finite");
    }
    if (config.optimizer.interval <= 0.0) {
      throw std::invalid_argument("optimizer.interval must be positive");
    }
    if (config.max_fallback_attempts <= 0) {
      throw std::invalid_argument("fallback.max_attempts must be positive");
    }
    if (config.dynamic_limits.max_velocity < 0.0) {
      throw std::invalid_argument("planning.max_velocity must be non-negative");
    }
    if (config.dynamic_limits.max_acceleration < 0.0) {
      throw std::invalid_argument("planning.max_acceleration must be non-negative");
    }
    if (config.feasibility_sample_step_seconds <= 0.0) {
      throw std::invalid_argument("planning.feasibility_sample_dt must be positive");
    }
    if (config.max_dynamic_time_scale <= 0.0) {
      throw std::invalid_argument("planning.max_dynamic_time_scale must be positive");
    }
    if (config.min_time_allocation_interval <= 0.0) {
      throw std::invalid_argument("planning.min_time_allocation_interval must be positive");
    }
    if (!std::isfinite(planning_inflation_radius_) || planning_inflation_radius_ < 0.0) {
      throw std::invalid_argument("planning.inflation_radius must be non-negative and finite");
    }
    if (uav_endpoint_snap_radius_cells_ < 0) {
      throw std::invalid_argument("planning.uav_endpoint_snap_radius_cells must be non-negative");
    }
    return config;
  }

  bool isExpectedFrame(const geometry_msgs::msg::PoseStamped& msg) const
  {
    return msg.header.frame_id == frame_id_;
  }

  bool isExpectedFrame(const geometry_msgs::msg::PointStamped& msg) const
  {
    return msg.header.frame_id == frame_id_;
  }

  bool isExpectedFrame(const sensor_msgs::msg::PointCloud2& msg) const
  {
    return msg.header.frame_id == frame_id_;
  }

  std::optional<nav_msgs::msg::OccupancyGrid> makeOccupancyGrid()
  {
    nav3d::map::Map2DProjectionOptions projection_options;
    projection_options.max_cells = occupancy_grid_max_cells_;
    projection_options.min_z = occupancy_grid_min_z_;
    projection_options.max_z = occupancy_grid_max_z_;
    std::optional<GroundTraversabilityMap> ground_traversability_map;
    if (planning_traversability_mode_ == PlanningTraversabilityMode::Ground) {
      ground_traversability_map.emplace(*map_, ground_search_options_);
      projection_options.free_cell_z = occupancy_grid_min_z_ + map_->getResolution() * 0.5;
      projection_options.is_free_cell =
        [&ground_traversability_map](const nav3d::common::Point3D& point) {
          return ground_traversability_map->isFree(point);
        };
    }
    const auto projected =
      nav3d::map::Map2DProjection::projectOccupiedVoxels(*map_, projection_options);
    if (!projected.has_value()) {
      const auto bounds = map_->getBounds();
      if (bounds.valid) {
        const double resolution = map_->getResolution();
        const int min_x = static_cast<int>(std::floor(bounds.min.x / resolution + 1e-9));
        const int max_x = static_cast<int>(std::floor(bounds.max.x / resolution + 1e-9));
        const int min_y = static_cast<int>(std::floor(bounds.min.y / resolution + 1e-9));
        const int max_y = static_cast<int>(std::floor(bounds.max.y / resolution + 1e-9));
        const auto cells =
          static_cast<std::uint64_t>(max_x - min_x + 1) *
          static_cast<std::uint64_t>(max_y - min_y + 1);
        if (cells > static_cast<std::uint64_t>(occupancy_grid_max_cells_)) {
          std::ostringstream status;
          status << "occupied_grid_skipped cells=" << cells
                 << " limit=" << occupancy_grid_max_cells_;
          publishStatus(status.str());
          RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
        }
      }
      return std::nullopt;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = frame_id_;
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(projected->resolution);
    grid.info.width = static_cast<std::uint32_t>(projected->width);
    grid.info.height = static_cast<std::uint32_t>(projected->height);
    grid.info.origin.position.x = projected->origin.x;
    grid.info.origin.position.y = projected->origin.y;
    grid.info.origin.position.z = projected->origin.z;
    grid.info.origin.orientation.w = 1.0;
    grid.data = projected->data;
    return grid;
  }

  void publishOccupancyGrid()
  {
    const auto grid = makeOccupancyGrid();
    if (!grid.has_value()) {
      return;
    }
    occupied_grid_pub_->publish(*grid);
    std::ostringstream status;
    status << "occupied_grid_published width=" << grid->info.width
           << " height=" << grid->info.height
           << " occupied=" << std::count(grid->data.begin(), grid->data.end(), 100)
           << " free=" << std::count(grid->data.begin(), grid->data.end(), 0)
           << " unknown=" << std::count(grid->data.begin(), grid->data.end(), -1)
           << " min_z=" << occupancy_grid_min_z_
           << " max_z=" << occupancy_grid_max_z_;
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  visualization_msgs::msg::Marker makeVoxelCubeList(
    const std_msgs::msg::Header& header,
    const std::string& ns,
    int id,
    double resolution,
    const std::vector<geometry_msgs::msg::Point>& points,
    float red,
    float green,
    float blue,
    float alpha) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.points = points;
    return marker;
  }

  visualization_msgs::msg::MarkerArray makePlanningOccupiedMarkerArray() const
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker cleanup;
    cleanup.header = header;
    cleanup.ns = "nav3d_planning_occupied_voxels_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(cleanup);

    const double resolution = map_->getResolution();
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(map_->occupiedCells().size());
    for (const auto& cell : map_->occupiedCells()) {
      const auto corner = map_->gridToWorld(cell);
      geometry_msgs::msg::Point point;
      point.x = corner.x + resolution * 0.5;
      point.y = corner.y + resolution * 0.5;
      point.z = corner.z + resolution * 0.5;
      points.push_back(point);
    }

    array.markers.push_back(makeVoxelCubeList(
      header,
      "nav3d_planning_occupied_voxels",
      0,
      resolution,
      points,
      0.1F,
      0.85F,
      0.25F,
      0.78F));

    if (local_grid_) {
      const double local_resolution = local_grid_->getResolution();
      std::vector<geometry_msgs::msg::Point> local_points;
      local_points.reserve(local_grid_->observedCells().size());
      for (const auto& [cell, state] : local_grid_->observedCells()) {
        if (state != nav3d::map::CellState::Occupied) {
          continue;
        }
        const auto corner = local_grid_->gridToWorld(cell);
        geometry_msgs::msg::Point point;
        point.x = corner.x + local_resolution * 0.5;
        point.y = corner.y + local_resolution * 0.5;
        point.z = corner.z + local_resolution * 0.5;
        local_points.push_back(point);
      }
      if (!local_points.empty()) {
        array.markers.push_back(makeVoxelCubeList(
          header,
          "nav3d_local_occupied_voxels",
          1,
          local_resolution,
          local_points,
          0.0F,
          0.65F,
          1.0F,
          0.92F));
      }
    }
    return array;
  }

  std::size_t localOccupiedVoxelCount() const
  {
    if (!local_grid_) {
      return 0;
    }
    return static_cast<std::size_t>(std::count_if(
      local_grid_->observedCells().begin(),
      local_grid_->observedCells().end(),
      [](const auto& entry) {
        return entry.second == nav3d::map::CellState::Occupied;
      }));
  }

  void publishPlanningOccupiedMarkers()
  {
    if (!map_) {
      publishStatus("planning_occupied_markers_skipped no_map");
      return;
    }

    const auto markers = makePlanningOccupiedMarkerArray();
    planning_occupied_markers_pub_->publish(markers);
    std::ostringstream status;
    status << "planning_occupied_markers_published voxels=" << map_->occupiedCells().size()
           << " local_voxels=" << localOccupiedVoxelCount();
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  void publishOctomapBinary()
  {
#ifdef NAV3D_HAS_OCTOMAP_MSGS
    if (!octomap_) {
      publishStatus("octomap_binary_skipped no_map");
      return;
    }

    octomap_msgs::msg::Octomap msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    if (!octomap_msgs::binaryMapToMsg(octomap_->tree(), msg)) {
      publishStatus("octomap_binary_failed conversion_failed");
      RCLCPP_WARN(get_logger(), "Failed to convert OctoMap to octomap_msgs/Octomap");
      return;
    }

    octomap_pub_->publish(msg);
    std::ostringstream status;
    status << "octomap_binary_published bytes=" << msg.data.size()
           << " leafs=" << octomap_->occupiedLeafCount();
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
#else
    publishStatus("octomap_binary_unavailable octomap_msgs_not_found");
#endif
  }

  void publishStaticMapVisualization()
  {
    publishOccupancyGrid();
    publishPlanningOccupiedMarkers();
    publishOctomapBinary();
  }

  void startStaticVisualizationRepublish()
  {
    static_visualization_republish_count_ = 0;
    static_visualization_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        publishStaticMapVisualization();
        ++static_visualization_republish_count_;
        if (static_visualization_republish_count_ >= 5 && static_visualization_timer_) {
          static_visualization_timer_->cancel();
        }
      });
  }

  nav3d::common::Point3D transformPointToMap(
    const nav3d::common::Point3D& point,
    const std::optional<geometry_msgs::msg::TransformStamped>& transform) const
  {
    if (!transform.has_value()) {
      return point;
    }

    geometry_msgs::msg::PointStamped source;
    geometry_msgs::msg::PointStamped target;
    source.header.frame_id = transform->child_frame_id;
    source.point.x = point.x;
    source.point.y = point.y;
    source.point.z = point.z;
    tf2::doTransform(source, target, *transform);
    return {target.point.x, target.point.y, target.point.z};
  }

  nav3d::common::Point3D transformOriginToMap(
    const geometry_msgs::msg::TransformStamped& transform) const
  {
    return {
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z,
    };
  }

  void onLocalPointCloud(const sensor_msgs::msg::PointCloud2& msg)
  {
    if (!local_grid_) {
      publishStatus("local_pointcloud_ignored local_grid_disabled");
      return;
    }

    const rclcpp::Time cloud_stamp(msg.header.stamp);
    const bool has_cloud_stamp = cloud_stamp.nanoseconds() != 0;
    const rclcpp::Time received_at = now();
    if (has_cloud_stamp) {
      const double age_sec = (received_at - cloud_stamp).seconds();
      if (local_grid_max_observation_age_sec_ > 0.0 &&
          age_sec > local_grid_max_observation_age_sec_) {
        std::ostringstream status;
        status << "local_pointcloud_rejected stale_stamp age_sec=" << age_sec
               << " max_age_sec=" << local_grid_max_observation_age_sec_;
        publishStatus(status.str());
        RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
        return;
      }
      if (local_grid_max_future_stamp_sec_ > 0.0 &&
          -age_sec > local_grid_max_future_stamp_sec_) {
        std::ostringstream status;
        status << "local_pointcloud_rejected future_stamp lead_sec=" << -age_sec
               << " max_future_sec=" << local_grid_max_future_stamp_sec_;
        publishStatus(status.str());
        RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
        return;
      }
    }

    std::optional<geometry_msgs::msg::TransformStamped> pointcloud_transform;
    if (msg.header.frame_id != frame_id_) {
      try {
        pointcloud_transform = tf_buffer_->lookupTransform(
          frame_id_,
          msg.header.frame_id,
          has_cloud_stamp ? cloud_stamp : rclcpp::Time(0));
      } catch (const tf2::TransformException& error) {
        publishStatus("local_pointcloud_rejected frame_transform_unavailable");
        RCLCPP_WARN(
          get_logger(),
          "Rejected local pointcloud in frame '%s'; expected '%s' or a TF transform: %s",
          msg.header.frame_id.c_str(),
          frame_id_.c_str(),
          error.what());
        return;
      }
    }

    std::size_t occupied = 0;
    std::size_t ray_cleared = 0;
    std::size_t skipped = 0;
    std::vector<nav3d::common::Point3D> frame_points;
    std::optional<nav3d::common::Point3D> ray_origin;
    if (pointcloud_transform.has_value()) {
      ray_origin = transformOriginToMap(*pointcloud_transform);
    } else if (current_position_.has_value()) {
      ray_origin = current_position_;
    }
    const bool can_ray_clear =
      local_grid_ray_clearing_enabled_ &&
      ray_origin.has_value() &&
      local_grid_->isInBounds(*ray_origin);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iter(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iter(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iter(msg, "z");
      for (; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
        const nav3d::common::Point3D raw_point{*x_iter, *y_iter, *z_iter};
        const nav3d::common::Point3D point =
          transformPointToMap(raw_point, pointcloud_transform);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !local_grid_->isInBounds(point)) {
          ++skipped;
          continue;
        }
        frame_points.push_back(point);
        if (can_ray_clear) {
          ++ray_cleared;
        }
        ++occupied;
      }
    } catch (const std::runtime_error& error) {
      publishStatus("local_pointcloud_rejected invalid_xyz_fields");
      RCLCPP_WARN(get_logger(), "Rejected local pointcloud: %s", error.what());
      return;
    }

    updateLocalObservationWindow(
      frame_points,
      can_ray_clear ? ray_origin : std::nullopt,
      has_cloud_stamp ? cloud_stamp : received_at);

    std::ostringstream status;
    status << "local_pointcloud_updated occupied=" << occupied
           << " ray_cleared=" << ray_cleared
           << " skipped=" << skipped
           << " retained_frames=" << local_pointcloud_frames_.size();
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    publishPlanningOccupiedMarkers();
    // A fresh local cloud is an obstacle update, not just visualization data.
    // Evaluate immediately so a newly spawned travel anchor can trigger a
    // replan without waiting for the next pose tick; safety_replan_cooldown_sec
    // still prevents repeated replans during bursty pointcloud publishes.
    if (current_position_.has_value() && active_trajectory_.has_value()) {
      evaluateActiveTrajectorySafety(*current_position_);
    }
  }

  void onLoadPcdPath(const std_msgs::msg::String& msg)
  {
    const std::string pcd_path = trim(msg.data);
    if (pcd_path.empty()) {
      publishStatus("map_reload_rejected empty_pcd_path");
      RCLCPP_WARN(get_logger(), "Rejected map reload with an empty PCD path");
      return;
    }

    nav3d::map::MapBuildConfig next_config = map_build_config_;
    next_config.pcd_path = pcd_path;
    try {
      auto next_map = std::make_unique<nav3d::map::VoxelGridMap>(buildMap(next_config));
      auto next_octomap = std::make_unique<nav3d::map::OctomapManager>(buildOctomap(next_config));
      std::unique_ptr<nav3d::map::LocalGrid> next_local_grid;
      if (local_grid_enabled_) {
        next_local_grid = makeLocalGridForMap(*next_map);
      }
      const bool had_active_tracking = tracking_active_;
      tracking_active_ = false;
      active_trajectory_.reset();
      active_goal_.reset();
      clearPublishedTrajectory();
      abortPendingNavigateGoal("navigate_aborted_map_reload");
      map_ = std::move(next_map);
      octomap_ = std::move(next_octomap);
      map_build_config_ = next_config;
      if (local_grid_enabled_) {
        local_grid_ = std::move(next_local_grid);
        clearLocalObservationWindow();
      } else {
        local_grid_.reset();
        local_pointcloud_frames_.clear();
      }
      publishStaticMapVisualization();
      startStaticVisualizationRepublish();
      if (had_active_tracking) {
        publishZeroCommand("map_reload_stop_tracking");
      }

      std::ostringstream status;
      status << "map_reload_success pcd_path=" << pcd_path
             << " occupied=" << map_->occupiedCells().size()
             << " resolution=" << map_->getResolution();
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    } catch (const std::exception& error) {
      std::ostringstream status;
      status << "map_reload_failed pcd_path=" << pcd_path
             << " error=" << error.what();
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
    }
  }

  void onStart(const geometry_msgs::msg::PoseStamped& msg)
  {
    if (!isExpectedFrame(msg)) {
      publishStatus("start_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected start in frame '%s'; expected '%s'",
        msg.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    start_ = toPoint(msg);
    // v3.8: setting a fresh start invalidates the previous current_position_.
    // Otherwise W-A's planToGoal would use the stale current_position_ (left
    // over from a prior follow-trajectory run that walked far past start_)
    // and the next /nav3d/goal would plan from there, producing an empty or
    // trivial path when the operator's intent was "robot is at this start
    // now, plan from here". The next /nav3d/current_pose tick will rewrite
    // current_position_ if the controller is still actively tracking.
    current_position_.reset();
    publishEndpointMarker(start_, "nav3d_start", *start_marker_pub_, 0.1F, 0.95F, 0.25F);
    std::ostringstream status;
    status << "start_updated x=" << start_.x << " y=" << start_.y << " z=" << start_.z;
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  nav3d::common::Point3D resolveUavEndpoint(
    const nav3d::map::IMap& planning_map,
    const nav3d::common::Point3D& requested) const
  {
    if (planning_traversability_mode_ != PlanningTraversabilityMode::Uav ||
        ground_search_options_.mode != nav3d::planner::PlanningMode::Mode3D ||
        uav_endpoint_snap_radius_cells_ == 0 ||
        planning_map.getResolution() <= 0.0) {
      return requested;
    }

    const double resolution = planning_map.getResolution();
    const auto requested_idx = worldToGroundIndex(requested, resolution);
    bool has_best = false;
    nav3d::common::Point3D best = requested;
    int best_below_rank = 1;
    int best_clearance_rank = 1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int radius = 0; radius <= uav_endpoint_snap_radius_cells_; ++radius) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dz = -radius; dz <= radius; ++dz) {
            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) {
              continue;
            }

            const auto candidate = groundIndexToWorld(
              {requested_idx.x + dx, requested_idx.y + dy, requested_idx.z + dz},
              resolution);
            if (!planning_map.isFree(candidate)) {
              continue;
            }

            const int below_rank = candidate.z + 1e-9 >= requested.z ? 0 : 1;
            const double distance = nav3d::common::distance(candidate, requested);
            const int clearance_rank =
              hasOccupiedNeighbor(planning_map, candidate, resolution) ? 1 : 0;
            constexpr double kEpsilon = 1e-12;
            const bool better =
              !has_best ||
              below_rank < best_below_rank ||
              (below_rank == best_below_rank && clearance_rank < best_clearance_rank) ||
              (below_rank == best_below_rank && clearance_rank == best_clearance_rank &&
               distance < best_distance - kEpsilon);
            if (better) {
              has_best = true;
              best = candidate;
              best_below_rank = below_rank;
              best_clearance_rank = clearance_rank;
              best_distance = distance;
            }
          }
        }
      }
      if (has_best && best_below_rank == 0 && best_clearance_rank == 0) {
        return best;
      }
    }

    return has_best ? best : requested;
  }

  nav3d::common::Point3D resolveGroundEndpoint(
    const nav3d::map::IMap& planning_map,
    const nav3d::common::Point3D& requested) const
  {
    if (!groundTraversabilityEnabled() || planning_map.getResolution() <= 0.0) {
      return requested;
    }

    const double resolution = planning_map.getResolution();
    const auto requested_idx = worldToGroundIndex(requested, resolution);
    const auto resolved_idx = resolveGroundIndex(planning_map, requested_idx, ground_search_options_);
    if (!isGroundTraversable(planning_map, resolved_idx, ground_search_options_)) {
      return requested;
    }
    return groundIndexToWorld(resolved_idx, resolution);
  }

  bool hasOccupiedNeighbor(
    const nav3d::map::IMap& planning_map,
    const nav3d::common::Point3D& point,
    double resolution) const
  {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const nav3d::common::Point3D neighbor{
            point.x + static_cast<double>(dx) * resolution,
            point.y + static_cast<double>(dy) * resolution,
            point.z + static_cast<double>(dz) * resolution,
          };
          if (planning_map.isInBounds(neighbor) && planning_map.isOccupied(neighbor)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped& msg)
  {
    if (!isExpectedFrame(msg)) {
      publishStatus("clicked_point_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected clicked point in frame '%s'; expected '%s'",
        msg.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    const auto requested_point = toPoint(msg);
    if (next_clicked_point_is_start_) {
      const auto point = resolveClickedPoint(requested_point, "start");
      start_ = point;
      next_clicked_point_is_start_ = false;
      clearPublishedTrajectory();
      publishEndpointMarker(start_, "nav3d_start", *start_marker_pub_, 0.1F, 0.95F, 0.25F);
      clearEndpointMarker("nav3d_goal", *goal_marker_pub_);
      std::ostringstream status;
      status << "clicked_start_updated x=" << start_.x
             << " y=" << start_.y
             << " z=" << start_.z;
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s; click the goal point next", status.str().c_str());
      return;
    }

    next_clicked_point_is_start_ = true;
    const auto point = resolveClickedPoint(requested_point, "goal");
    planToGoal(point, "clicked_plan");
  }

  void onCurrentPose(const geometry_msgs::msg::PoseStamped& msg)
  {
    if (!isExpectedFrame(msg)) {
      publishStatus("current_pose_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected current pose in frame '%s'; expected '%s'",
        msg.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    current_position_ = toPoint(msg);
    current_yaw_ = tf2::getYaw(msg.pose.orientation);
    evaluateActiveTrajectorySafety(*current_position_);
    if (handleCurrentPoseGoalReached(*current_position_)) {
      return;
    }
    publishRemainingTrajectory(*current_position_);
  }

  void onPlanPath(
    const std::shared_ptr<GetPlan::Request> request,
    std::shared_ptr<GetPlan::Response> response)
  {
    response->plan.header.stamp = now();
    response->plan.header.frame_id = frame_id_;
    if (!isExpectedFrame(request->start) || !isExpectedFrame(request->goal)) {
      publishStatus("service_plan_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected GetPlan request start_frame='%s' goal_frame='%s'; expected '%s'",
        request->start.header.frame_id.c_str(),
        request->goal.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    const auto result = planWithActiveMap(toPoint(request->start), toPoint(request->goal));
    if (!result.success) {
      std::ostringstream status;
      status << "service_plan_failed status=" << statusToString(result.status)
             << " attempts=" << result.attempts
             << " tolerance=" << request->tolerance;
      appendPlanFailureDebug(status, result, planningGoalTolerance());
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      return;
    }

    response->plan = sampleTrajectoryPath(result.trajectory);
    std::ostringstream status;
    status << "service_plan_success poses=" << response->plan.poses.size()
           << " attempts=" << result.attempts
           << " tolerance=" << request->tolerance;
    appendPlanEndpointStatus(status, result, planningGoalTolerance());
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  void onGoal(const geometry_msgs::msg::PoseStamped& msg)
  {
    if (!isExpectedFrame(msg)) {
      publishStatus("goal_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected goal in frame '%s'; expected '%s'",
        msg.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    // Snap goal to a free, traversable cell when the requested point is
    // occupied / unsupported. Web UI publishes /nav3d/goal directly without
    // going through /clicked_point, so without this call a click that lands
    // on an obstacle voxel falls through to plan_failed status=no_path with
    // no recovery. Match onClickedPoint behaviour: snap, log, then plan.
    const auto requested = toPoint(msg);
    const auto resolved = resolveClickedPoint(requested, "goal");
    planToGoal(resolved, "plan");
  }

  void planToGoal(const nav3d::common::Point3D& goal, const std::string& status_prefix)
  {
    publishEndpointMarker(goal, "nav3d_goal", *goal_marker_pub_, 0.95F, 0.2F, 0.12F);
    // W-A (v3.8): plan from the live current_position_ when available so a
    // mid-flight /nav3d/goal click does not re-issue the path from a stale
    // start_ (the original origin). Without this, when the robot has moved
    // past obstacles the planner re-plans through cells that now hold local
    // obstacle voxels, producing slow/no-path replans. action API at L2165
    // already does this; only the topic-driven onGoal path was wrong.
    const auto plan_start = current_position_.value_or(start_);
    const auto result = planWithActiveMap(plan_start, goal);
    if (!result.success) {
      tracking_active_ = false;
      active_trajectory_.reset();
      active_goal_.reset();
      clearPublishedTrajectory();
      std::ostringstream status;
      status << status_prefix << "_failed status=" << statusToString(result.status)
             << " attempts=" << result.attempts;
      appendPlanFailureDebug(status, result, planningGoalTolerance());
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      if (result.status == nav3d::planner::EgoPlanStatus::EmergencyStop ||
          result.fallback.action == nav3d::planner::FallbackAction::EmergencyStop) {
        publishZeroCommand("plan_emergency_stop");
      }
      return;
    }

    active_trajectory_ = result.trajectory;
    active_goal_ = result.planned_goal;
    tracking_active_ = controller_enabled_;
    publishTrajectory(result);
    std::ostringstream status;
    status << status_prefix << "_success poses=" << last_published_pose_count_
           << " attempts=" << result.attempts
           << " time_scale=" << result.time_scale;
    appendPlanEndpointStatus(status, result, planningGoalTolerance());
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  rclcpp_action::GoalResponse handleNavigateGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    (void)uuid;
    if (goal->pose.header.frame_id != frame_id_) {
      publishStatus("navigate_goal_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected NavigateToPose goal in frame '%s'; expected '%s'",
        goal->pose.header.frame_id.c_str(),
        frame_id_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (hasPendingNavigateGoal()) {
      publishStatus("navigate_goal_rejected active_goal");
      RCLCPP_WARN(get_logger(), "Rejected NavigateToPose goal because another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleNavigateCancel(
    const std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    (void)goal_handle;
    tracking_active_ = false;
    active_trajectory_.reset();
    active_goal_.reset();
    publishZeroCommand("navigate_cancel");
    std::thread{[this, goal_handle]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      cancelPendingNavigateGoal(goal_handle, "navigate_canceled");
    }}.detach();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleNavigateAccepted(const std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    std::thread{[this, goal_handle]() {
      executeNavigateGoal(goal_handle);
    }}.detach();
  }

  void executeNavigateGoal(const std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto goal_point = toPoint(goal->pose);
    const auto start = current_position_.value_or(start_);

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    std_msgs::msg::Header feedback_header;
    feedback_header.stamp = now();
    feedback_header.frame_id = frame_id_;
    feedback->current_pose = toPoseStamped(start, feedback_header);
    feedback->distance_remaining = static_cast<float>(nav3d::common::distance(start, goal_point));
    goal_handle->publish_feedback(feedback);

    const auto plan = planWithActiveMap(start, goal_point);
    auto result = std::make_shared<NavigateToPose::Result>();
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      publishStatus("navigate_canceled");
      return;
    }

    if (plan.success) {
      active_goal_ = plan.planned_goal;
      active_trajectory_ = plan.trajectory;
      tracking_active_ = controller_enabled_;
      publishTrajectory(plan);
      if (controller_enabled_) {
        setPendingNavigateGoal(
          goal_handle,
          plan.requested_goal,
          plan.planned_goal);
        std::ostringstream status;
        status << "navigate_tracking_active poses=" << last_published_pose_count_
               << " attempts=" << plan.attempts;
        appendPlanEndpointStatus(status, plan, planningGoalTolerance());
        publishStatus(status.str());
        RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
        return;
      }
      std::ostringstream status;
      status << (isPartialPlan(plan, planningGoalTolerance()) ? "navigate_partial_goal_reached" : "navigate_success")
             << " poses=" << last_published_pose_count_
             << " attempts=" << plan.attempts
             << " tracking_disabled=true";
      appendPlanEndpointStatus(status, plan, planningGoalTolerance());
      publishStatus(status.str());
      if (isPartialPlan(plan, planningGoalTolerance())) {
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      } else {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
      }
      return;
    }

    std::ostringstream status;
    status << "navigate_failed status=" << statusToString(plan.status)
           << " attempts=" << plan.attempts;
    publishStatus(status.str());
    tracking_active_ = false;
    if (plan.status == nav3d::planner::EgoPlanStatus::EmergencyStop ||
        plan.fallback.action == nav3d::planner::FallbackAction::EmergencyStop) {
      publishZeroCommand("navigate_emergency_stop");
    }
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
  }

  void evaluateActiveTrajectorySafety(const nav3d::common::Point3D& current_position)
  {
    if (!safety_monitor_ || !active_trajectory_.has_value() || !active_goal_.has_value()) {
      return;
    }

    const auto decision = evaluateSafetyWithActiveMap(current_position);

    if (decision.action == nav3d::controller::SafetyAction::EmergencyStop) {
      // W-B (v3.8): before declaring emergency stop, try one bounded local
      // replan with raw A* feasibility probe + EGO local ladder. The original
      // logic abandoned the trajectory immediately on EmergencyStop, which
      // produced the "click an obstacle in front of the robot and bridge just
      // stops, no replan" symptom the user reported. The cooldown gate inside
      // replanFromCurrentPose already prevents thrashing when the obstacle
      // genuinely cannot be avoided locally; if the local + global replan
      // both fail there, the existing failure path still publishes
      // safety_replan_emergency_stop.
      if (safety_local_replan_enabled_ && active_trajectory_.has_value()) {
        std::ostringstream pre;
        pre << "safety_replan_attempt_before_emergency_stop";
        if (decision.first_collision_time.has_value()) {
          pre << " first_collision_time=" << *decision.first_collision_time;
        }
        publishStatus(pre.str());
        RCLCPP_WARN(get_logger(), "%s", pre.str().c_str());
        replanFromCurrentPose(current_position);
        // If replanFromCurrentPose succeeded it set active_trajectory_; if it
        // failed it already cleared everything + published the appropriate
        // failure status. Either way we are done with this pose tick.
        return;
      }
      tracking_active_ = false;
      active_trajectory_.reset();
      active_goal_.reset();
      clearPublishedTrajectory();
      abortPendingNavigateGoal("navigate_aborted_safety_emergency_stop");
      publishZeroCommand("safety_emergency_stop");
      return;
    }
    if (decision.action == nav3d::controller::SafetyAction::ReplanNeeded) {
      std::ostringstream status;
      status << "safety_replan_needed";
      if (decision.first_collision_time.has_value()) {
        status << " first_collision_time=" << *decision.first_collision_time;
      }
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      replanFromCurrentPose(current_position);
    }
  }

  template <typename Callback>
  decltype(auto) makePlanningMap(Callback&& callback) const
  {
    return makeCollisionPlanningMap(
      [&](const nav3d::map::IMap& planning_map) -> decltype(auto) {
        return withGroundTraversability(planning_map, std::forward<Callback>(callback));
      });
  }

  template <typename Callback>
  decltype(auto) makeCollisionPlanningMap(Callback&& callback) const
  {
    const nav3d::map::IMap& global_map = activeGlobalPlanningMap();
    if (hasLocalObservations()) {
      nav3d::map::MapComposite composite(global_map, *local_grid_);
      if (planning_inflation_radius_ > 0.0) {
        nav3d::collision::InflationLayer planning_map(
          composite,
          planning_inflation_radius_);
        return std::forward<Callback>(callback)(planning_map);
      }
      return std::forward<Callback>(callback)(composite);
    }
    if (planning_inflation_radius_ > 0.0) {
      nav3d::collision::InflationLayer planning_map(
        global_map,
        planning_inflation_radius_);
      return std::forward<Callback>(callback)(planning_map);
    }
    return std::forward<Callback>(callback)(global_map);
  }

  template <typename Callback>
  decltype(auto) withGroundTraversability(
    const nav3d::map::IMap& planning_map,
    Callback&& callback) const
  {
    if (!groundTraversabilityEnabled()) {
      return std::forward<Callback>(callback)(planning_map);
    }

    GroundTraversabilityMap traversability_map(planning_map, ground_search_options_);
    return std::forward<Callback>(callback)(traversability_map);
  }

  bool groundTraversabilityEnabled() const
  {
    return planning_traversability_mode_ == PlanningTraversabilityMode::Ground;
  }

  nav3d::common::Point3D resolveClickedPoint(
    const nav3d::common::Point3D& requested,
    const std::string& role)
  {
    pruneExpiredLocalObservations();
    const auto resolved = makeCollisionPlanningMap([&](const nav3d::map::IMap& planning_map) {
      if (groundTraversabilityEnabled()) {
        return resolveGroundEndpoint(planning_map, requested);
      }
      return resolveUavEndpoint(planning_map, requested);
    });
    if (nav3d::common::distance(resolved, requested) > 1e-9) {
      std::ostringstream status;
      status << (groundTraversabilityEnabled() ? "ground_endpoint_snapped" : "uav_endpoint_snapped")
             << " role=" << role
             << " requested=" << requested.x << "," << requested.y << "," << requested.z
             << " snapped=" << resolved.x << "," << resolved.y << "," << resolved.z;
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    }
    return resolved;
  }

  const nav3d::map::IMap& activeGlobalPlanningMap() const
  {
    if (planning_use_octomap_map_ &&
        ground_search_options_.mode == nav3d::planner::PlanningMode::Mode3D &&
        octomap_) {
      return *octomap_;
    }
    return *map_;
  }

  const char* planningMapBackendName() const
  {
    return planning_use_octomap_map_ ? "octomap" : "voxel_grid";
  }

  double planningGoalTolerance() const
  {
    return snappedGoalTolerance(activeGlobalPlanningMap().getResolution());
  }

  nav3d::planner::EgoPlanResult planWithActiveMap(
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal)
  {
    pruneExpiredLocalObservations();
    return makePlanningMap([&](const nav3d::map::IMap& planning_map) {
      return planner_->planWithFallbacks(planning_map, start, goal);
    });
  }

  nav3d::controller::SafetyDecision evaluateSafetyWithActiveMap(
    const nav3d::common::Point3D& current_position)
  {
    pruneExpiredLocalObservations();
    return makePlanningMap([&](const nav3d::map::IMap& planning_map) {
      return safety_monitor_->evaluate(
        planning_map,
        current_position,
        *active_trajectory_,
        *active_goal_);
    });
  }

  nav3d::controller::TrajectoryCommand computeControllerCommandWithActiveMap(
    const nav3d::common::Point3D& current_position)
  {
    pruneExpiredLocalObservations();
    return makePlanningMap([&](const nav3d::map::IMap& planning_map) {
      return trajectory_tracker_->computeCommand(
        *active_trajectory_,
        current_position,
        current_yaw_.value_or(0.0),
        *active_goal_,
        planning_map,
        last_controller_command_);
    });
  }

  bool hasLocalObservations() const
  {
    return local_grid_ && !local_grid_->observedCells().empty();
  }

  struct LocalPointCloudFrame {
    rclcpp::Time received_at;
    std::vector<nav3d::common::Point3D> occupied_points;
    std::optional<nav3d::common::Point3D> ray_origin;
  };

  struct LookaheadTarget {
    nav3d::common::Point3D point;
    double time = 0.0;
  };

  void clearLocalObservationWindow()
  {
    local_pointcloud_frames_.clear();
    if (local_grid_) {
      local_grid_->clear();
    }
  }

  void updateLocalObservationWindow(
    const std::vector<nav3d::common::Point3D>& frame_points,
    const std::optional<nav3d::common::Point3D>& ray_origin,
    const rclcpp::Time& observation_time)
  {
    if (!local_grid_) {
      return;
    }

    if (frame_points.empty()) {
      clearLocalObservationWindow();
      return;
    }

    if (local_grid_retention_sec_ <= 0.0 || local_grid_max_retained_frames_ <= 1) {
      local_pointcloud_frames_.clear();
    }

    local_pointcloud_frames_.push_back(LocalPointCloudFrame{
      observation_time,
      frame_points,
      ray_origin,
    });
    pruneLocalObservationWindow();
    rebuildLocalGridFromObservationWindow();
  }

  void pruneLocalObservationWindow()
  {
    const rclcpp::Time prune_time = now();
    while (local_pointcloud_frames_.size() >
           static_cast<std::size_t>(local_grid_max_retained_frames_)) {
      local_pointcloud_frames_.pop_front();
    }

    if (local_grid_retention_sec_ <= 0.0) {
      return;
    }

    while (!local_pointcloud_frames_.empty() &&
           (prune_time - local_pointcloud_frames_.front().received_at).seconds() >
             local_grid_retention_sec_) {
      local_pointcloud_frames_.pop_front();
    }
  }

  void pruneExpiredLocalObservations()
  {
    if (local_grid_retention_sec_ <= 0.0 || local_pointcloud_frames_.empty()) {
      return;
    }
    const std::size_t before = local_pointcloud_frames_.size();
    pruneLocalObservationWindow();
    if (local_pointcloud_frames_.size() != before) {
      rebuildLocalGridFromObservationWindow();
    }
  }

  void rebuildLocalGridFromObservationWindow()
  {
    if (!local_grid_) {
      return;
    }

    local_grid_->clear();
    for (const auto& frame : local_pointcloud_frames_) {
      for (const auto& point : frame.occupied_points) {
        if (frame.ray_origin.has_value()) {
          local_grid_->markRayFreeAndOccupied(*frame.ray_origin, point);
        } else {
          local_grid_->markOccupied(point);
        }
      }
    }
  }

  // Pick a point along the currently-active B-spline that lies approximately
  // `lookahead_distance` past `current_position`. Returns std::nullopt when the
  // active trajectory is too short (use the real goal instead).
  std::optional<LookaheadTarget> pickLookaheadGoal(
    const nav3d::common::Point3D& current_position,
    double lookahead_distance) const
  {
    if (!active_trajectory_.has_value()) return std::nullopt;
    const auto& bspline = *active_trajectory_;
    const double duration = bspline.duration();
    if (!std::isfinite(duration) || duration <= 0.0) return std::nullopt;
    const double start_t = estimateTrajectoryTime(current_position, bspline);
    const double step = std::max(0.05, publish_sample_dt_);
    double traveled = 0.0;
    nav3d::common::Point3D prev = bspline.evaluate(start_t);
    for (double t = start_t + step; t <= duration; t += step) {
      const auto p = bspline.evaluate(t);
      traveled += nav3d::common::distance(prev, p);
      if (traveled >= lookahead_distance) return LookaheadTarget{p, t};
      prev = p;
    }
    return std::nullopt;  // remaining trajectory shorter than lookahead → use real goal
  }

  nav3d::common::Path3D sampleTrajectoryPoints(
    const nav3d::planner::UniformBspline& trajectory,
    double start_time,
    double end_time) const
  {
    nav3d::common::Path3D points;
    const double duration = trajectory.duration();
    if (duration <= 0.0 || !std::isfinite(duration)) {
      return points;
    }

    start_time = std::clamp(start_time, 0.0, duration);
    end_time = std::clamp(end_time, start_time, duration);
    points.push_back(trajectory.evaluate(start_time));
    for (double t = start_time + publish_sample_dt_; t < end_time; t += publish_sample_dt_) {
      points.push_back(trajectory.evaluate(t));
    }
    if (end_time > start_time + 1e-9) {
      points.push_back(trajectory.evaluate(end_time));
    }
    return points;
  }

  std::optional<nav3d::planner::UniformBspline> stitchLocalReplanTrajectory(
    const nav3d::planner::UniformBspline& local_trajectory,
    const nav3d::planner::UniformBspline& previous_trajectory,
    double previous_resume_time) const
  {
    auto stitched = sampleTrajectoryPoints(
      local_trajectory,
      0.0,
      local_trajectory.duration());
    auto remainder = sampleTrajectoryPoints(
      previous_trajectory,
      previous_resume_time,
      previous_trajectory.duration());
    if (stitched.empty()) {
      stitched = std::move(remainder);
    } else {
      for (const auto& point : remainder) {
        if (!stitched.empty() &&
            nav3d::common::distance(stitched.back(), point) <= map_->getResolution() * 0.25) {
          continue;
        }
        stitched.push_back(point);
      }
    }

    if (stitched.size() < 2) {
      return std::nullopt;
    }
    return nav3d::planner::UniformBspline(std::move(stitched), 1, publish_sample_dt_);
  }

  void replanFromCurrentPose(const nav3d::common::Point3D& current_position)
  {
    if (!active_goal_.has_value()) return;
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - last_safety_replan_at_).count();
    if (elapsed < safety_replan_cooldown_sec_) return;
    last_safety_replan_at_ = now;

    // Try a small-window local replan first to keep replan latency low.
    // active_goal_ stays untouched; the controller still tracks the real final
    // goal because handleCurrentPoseGoalReached uses active_goal_ for completion.
    const auto final_goal = *active_goal_;
    const auto previous_trajectory = active_trajectory_;
    bool local_attempted = false;
    if (safety_local_replan_enabled_) {
      const auto lookahead_goal = pickLookaheadGoal(
        current_position, safety_local_replan_lookahead_distance_);
      if (lookahead_goal.has_value()) {
        // W-B (v3.8): raw-A* feasibility probe. If A* finds no discrete corridor
        // within the lookahead window in <50 ms, we skip the expensive local
        // EGO ladder and go straight to the global replan. This is the main
        // "fast local replan" optimization the user noticed missing.
        int astar_iter = 0;
        const bool feasible = localAStarFeasible(current_position, lookahead_goal->point, astar_iter);
        if (!feasible) {
          std::ostringstream probe;
          probe << "safety_replan_local_astar_skip iterations=" << astar_iter
                << " lookahead=" << safety_local_replan_lookahead_distance_;
          publishStatus(probe.str());
          RCLCPP_INFO(get_logger(), "%s", probe.str().c_str());
        } else {
          local_attempted = true;
          const auto local = planWithActiveMap(current_position, lookahead_goal->point);
          if (local.success) {
            const auto stitched = previous_trajectory.has_value()
              ? stitchLocalReplanTrajectory(
                  local.trajectory,
                  *previous_trajectory,
                  lookahead_goal->time)
              : std::optional<nav3d::planner::UniformBspline>{};
            if (stitched.has_value()) {
              const auto stitched_collision = makePlanningMap(
                [&](const nav3d::map::IMap& planning_map) {
                  const nav3d::collision::TrajectoryChecker checker(publish_sample_dt_);
                  return checker.check(planning_map, *stitched);
                });
              if (!stitched_collision.in_collision) {
                auto stitched_plan = local;
                stitched_plan.requested_goal = final_goal;
                stitched_plan.planned_goal = final_goal;
                stitched_plan.trajectory = *stitched;
                stitched_plan.collision = stitched_collision;
                active_trajectory_ = stitched_plan.trajectory;
                active_goal_ = final_goal;
                tracking_active_ = controller_enabled_;
                publishTrajectory(stitched_plan);
                std::ostringstream status;
                status << "safety_replan_success_local_astar_stitched poses="
                       << last_published_pose_count_
                       << " attempts=" << local.attempts
                       << " astar_iter=" << astar_iter
                       << " lookahead=" << safety_local_replan_lookahead_distance_
                       << " resume_time=" << lookahead_goal->time;
                publishStatus(status.str());
                RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
                return;
              }
              std::ostringstream collision_debug;
              collision_debug << "safety_replan_local_stitch_collision falling_back_to_global";
              if (stitched_collision.first_collision_time.has_value()) {
                collision_debug << " first_collision_time="
                                << *stitched_collision.first_collision_time;
              }
              publishStatus(collision_debug.str());
              RCLCPP_WARN(get_logger(), "%s", collision_debug.str().c_str());
            } else {
              publishStatus("safety_replan_local_stitch_failed falling_back_to_global");
              RCLCPP_WARN(get_logger(), "safety_replan_local_stitch_failed falling_back_to_global");
            }
          } else {
            std::ostringstream debug;
            debug << "safety_replan_local_failed status=" << statusToString(local.status)
                  << " astar_iter=" << astar_iter
                  << " falling_back_to_global";
            publishStatus(debug.str());
            RCLCPP_WARN(get_logger(), "%s", debug.str().c_str());
          }
        }
      }
    }

    // Global fallback: original behavior, replan all the way to final goal.
    const auto result = planWithActiveMap(current_position, final_goal);
    if (result.success) {
      active_trajectory_ = result.trajectory;
      active_goal_ = result.planned_goal;
      updatePendingNavigatePlannedGoal(result.planned_goal);
      tracking_active_ = controller_enabled_;
      publishTrajectory(result);
      std::ostringstream status;
      status << "safety_replan_success poses=" << last_published_pose_count_
             << " attempts=" << result.attempts
             << " local_attempted=" << (local_attempted ? "true" : "false");
      appendPlanEndpointStatus(status, result, planningGoalTolerance());
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
      return;
    }

    std::ostringstream status;
    status << "safety_replan_failed status=" << statusToString(result.status)
           << " attempts=" << result.attempts;
    publishStatus(status.str());
    RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
    tracking_active_ = false;
    active_trajectory_.reset();
    active_goal_.reset();
    clearPublishedTrajectory();
    abortPendingNavigateGoal("navigate_aborted_safety_replan_failed");
    publishZeroCommand("safety_replan_emergency_stop");
  }

  double estimateTrajectoryTime(
    const nav3d::common::Point3D& current_position,
    const nav3d::planner::UniformBspline& trajectory) const
  {
    const double duration = trajectory.duration();
    if (duration <= 0.0 || !std::isfinite(duration)) {
      return 0.0;
    }

    const double step = std::max(0.02, publish_sample_dt_);
    double best_time = 0.0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (double t = 0.0; t < duration; t += step) {
      const double candidate_distance =
        nav3d::common::distance(current_position, trajectory.evaluate(t));
      if (candidate_distance < best_distance) {
        best_distance = candidate_distance;
        best_time = t;
      }
    }
    const double final_distance =
      nav3d::common::distance(current_position, trajectory.evaluate(duration));
    if (final_distance < best_distance) {
      best_time = duration;
    }
    return best_time;
  }

  void onControllerTimer()
  {
    if (!controller_enabled_ || !tracking_active_ ||
        !active_trajectory_.has_value() || !active_goal_.has_value() ||
        !current_position_.has_value() || !trajectory_tracker_) {
      return;
    }

    const auto command = computeControllerCommandWithActiveMap(*current_position_);
    if (command.goal_reached) {
      tracking_active_ = false;
      active_trajectory_.reset();
      active_goal_.reset();
      last_controller_command_.reset();
      clearPublishedTrajectory();
      publishZeroCommand("tracking_goal_reached");
      completeNavigateGoalReached();
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = command.linear.x;
    cmd.linear.y = command.linear.y;
    cmd.linear.z = command.linear.z;
    cmd.angular.z = command.angular_z;
    cmd_vel_pub_->publish(cmd);
    last_controller_command_ = command;
  }

  nav_msgs::msg::Path sampleTrajectoryPath(
    const nav3d::planner::UniformBspline& trajectory,
    double start_time = 0.0)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;

    const double duration = trajectory.duration();
    if (duration <= 0.0 || !std::isfinite(duration)) {
      return path;
    }

    start_time = std::clamp(start_time, 0.0, duration);
    for (double t = start_time; t < duration; t += publish_sample_dt_) {
      path.poses.push_back(toPoseStamped(trajectory.evaluate(t), path.header));
    }
    path.poses.push_back(toPoseStamped(trajectory.evaluate(duration), path.header));
    return path;
  }

  visualization_msgs::msg::Marker makeTrajectoryMarker(
    const nav_msgs::msg::Path& path) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "nav3d_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = path.poses.empty() ?
      visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.18;
    marker.color.r = 0.55F;
    marker.color.g = 0.18F;
    marker.color.b = 0.95F;
    marker.color.a = 1.0F;
    marker.points.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
      marker.points.push_back(pose.pose.position);
    }
    return marker;
  }

  void publishTrajectoryMarker(const nav_msgs::msg::Path& path)
  {
    trajectory_marker_pub_->publish(makeTrajectoryMarker(path));
  }

  void publishEndpointMarker(
    const nav3d::common::Point3D& point,
    const std::string& ns,
    rclcpp::Publisher<visualization_msgs::msg::Marker>& publisher,
    float red,
    float green,
    float blue) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    publisher.publish(marker);
  }

  void clearEndpointMarker(
    const std::string& ns,
    rclcpp::Publisher<visualization_msgs::msg::Marker>& publisher) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    publisher.publish(marker);
  }

  void publishTrajectory(const nav3d::planner::EgoPlanResult& result)
  {
    const auto path = sampleTrajectoryPath(result.trajectory);
    last_published_pose_count_ = path.poses.size();
    trajectory_pub_->publish(path);
    publishTrajectoryMarker(path);
  }

  // W-B (v3.8): raw-A* feasibility probe. Used by replanFromCurrentPose to
  // decide *whether to attempt a local replan at all* before paying for the
  // full EGO ladder + B-spline optimization. AStar3D::search runs in tight
  // bounds (max_iterations=20000, ~6 m lookahead) and returns Success/NoPath
  // in milliseconds. Outcomes:
  //   - Success → call planWithActiveMap(current, lookahead_goal). A* having
  //     succeeded means a feasible discrete corridor exists, so the EGO
  //     fallback ladder usually converges on the first attempt instead of
  //     iterating through kinodynamic search + JPS + spline trials.
  //   - NoPath → skip the local stage entirely; fall straight through to the
  //     global replan. Avoids wasting ~50-150 ms on a doomed local attempt.
  // The probe itself does NOT touch active_trajectory_ — controller tracking
  // is uninterrupted while we decide; the existing spline keeps the cmd_vel
  // stream alive until the actual replan publishes a new trajectory.
  bool localAStarFeasible(
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal,
    int& iterations_out) const
  {
    iterations_out = 0;
    if (!safety_local_use_raw_astar_) return true;  // probe disabled → always attempt local
    nav3d::planner::SearchOptions opts;
    opts.algorithm = nav3d::planner::SearchAlgorithm::AStar;
    opts.mode = ground_search_options_.mode;
    opts.allow_diagonal = true;
    opts.max_iterations = std::max(1000, safety_local_astar_max_iterations_);
    auto raw = makePlanningMap(
      [&](const nav3d::map::IMap& planning_map) {
        return local_astar_.search(planning_map, start, goal, opts);
      });
    iterations_out = raw.iterations;
    return raw.status == nav3d::planner::SearchStatus::Success && !raw.path.empty();
  }

  bool handleCurrentPoseGoalReached(const nav3d::common::Point3D& current_position)
  {
    if (!active_goal_.has_value()) {
      return false;
    }
    if (nav3d::common::distance(current_position, *active_goal_) > controller_goal_tolerance_) {
      return false;
    }

    tracking_active_ = false;
    active_trajectory_.reset();
    active_goal_.reset();
    last_controller_command_.reset();
    clearPublishedTrajectory();
    publishZeroCommand("tracking_goal_reached");
    completeNavigateGoalReached();
    return true;
  }

  void publishRemainingTrajectory(const nav3d::common::Point3D& current_position)
  {
    if (!active_trajectory_.has_value()) {
      return;
    }
    const double start_time = estimateTrajectoryTime(current_position, *active_trajectory_);
    const auto path = sampleTrajectoryPath(*active_trajectory_, start_time);
    if (path.poses.size() <= 2) {
      clearPublishedTrajectory();
      return;
    }
    last_published_pose_count_ = path.poses.size();
    trajectory_pub_->publish(path);
    publishTrajectoryMarker(path);
  }

  void clearPublishedTrajectory()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    last_published_pose_count_ = 0;
    trajectory_pub_->publish(path);
    publishTrajectoryMarker(path);
  }

  geometry_msgs::msg::PoseStamped toPoseStamped(
    const nav3d::common::Point3D& point,
    const std_msgs::msg::Header& header) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  void publishStatus(const std::string& text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  void publishZeroCommand(const std::string& reason)
  {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
    last_controller_command_.reset();
    publishStatus(reason);
    RCLCPP_WARN(get_logger(), "%s: published zero cmd_vel", reason.c_str());
  }

  bool hasPendingNavigateGoal()
  {
    std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
    return pending_navigate_goal_.has_value() &&
           pending_navigate_goal_->handle &&
           pending_navigate_goal_->handle->is_active();
  }

  struct PendingNavigateGoal {
    std::shared_ptr<NavigateGoalHandle> handle;
    nav3d::common::Point3D requested_goal;
    nav3d::common::Point3D planned_goal;
  };

  void setPendingNavigateGoal(
    const std::shared_ptr<NavigateGoalHandle>& goal_handle,
    const nav3d::common::Point3D& requested_goal,
    const nav3d::common::Point3D& planned_goal)
  {
    std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
    pending_navigate_goal_ = PendingNavigateGoal{
      goal_handle,
      requested_goal,
      planned_goal,
    };
  }

  void updatePendingNavigatePlannedGoal(const nav3d::common::Point3D& planned_goal)
  {
    std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
    if (pending_navigate_goal_.has_value()) {
      pending_navigate_goal_->planned_goal = planned_goal;
    }
  }

  void completeNavigateGoalReached()
  {
    std::optional<PendingNavigateGoal> pending_goal;
    {
      std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
      pending_goal = pending_navigate_goal_;
      pending_navigate_goal_.reset();
    }
    if (!pending_goal.has_value() ||
        !pending_goal->handle ||
        !pending_goal->handle->is_active()) {
      return;
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    if (nav3d::common::distance(
          pending_goal->requested_goal,
          pending_goal->planned_goal) > planningGoalTolerance()) {
      pending_goal->handle->abort(result);
      std::ostringstream status;
      status << "navigate_partial_goal_reached planned_goal="
             << pending_goal->planned_goal.x << ","
             << pending_goal->planned_goal.y << ","
             << pending_goal->planned_goal.z
             << " requested_goal="
             << pending_goal->requested_goal.x << ","
             << pending_goal->requested_goal.y << ","
             << pending_goal->requested_goal.z;
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      return;
    }

    pending_goal->handle->succeed(result);
    publishStatus("navigate_goal_reached");
    RCLCPP_INFO(get_logger(), "navigate_goal_reached");
  }

  void abortPendingNavigateGoal(const std::string& reason)
  {
    std::shared_ptr<NavigateGoalHandle> goal_handle;
    {
      std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
      if (pending_navigate_goal_.has_value()) {
        goal_handle = pending_navigate_goal_->handle;
      }
      pending_navigate_goal_.reset();
    }
    if (!goal_handle || !goal_handle->is_active()) {
      return;
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    goal_handle->abort(result);
    publishStatus(reason);
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
  }

  void cancelPendingNavigateGoal(
    const std::shared_ptr<NavigateGoalHandle>& goal_handle,
    const std::string& reason)
  {
    bool should_cancel = false;
    {
      std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
      should_cancel =
        pending_navigate_goal_.has_value() &&
        pending_navigate_goal_->handle == goal_handle;
      if (should_cancel) {
        pending_navigate_goal_.reset();
      }
    }
    if (!should_cancel || !goal_handle->is_active()) {
      return;
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    goal_handle->canceled(result);
    publishStatus(reason);
    RCLCPP_INFO(get_logger(), "%s", reason.c_str());
  }

  std::string frame_id_;
  double publish_sample_dt_ = 0.05;
  bool local_grid_ray_clearing_enabled_ = false;
  bool local_grid_enabled_ = false;
  double local_grid_resolution_ = 0.5;
  double local_grid_retention_sec_ = 0.0;
  int local_grid_max_retained_frames_ = 1;
  double local_grid_max_observation_age_sec_ = 2.0;
  double local_grid_max_future_stamp_sec_ = 0.2;
  bool controller_enabled_ = false;
  double safety_replan_cooldown_sec_ = 0.15;
  std::chrono::steady_clock::time_point last_safety_replan_at_{};
  double safety_local_replan_lookahead_distance_ = 6.0;
  bool safety_local_replan_enabled_ = true;
  // W-B (v3.8): raw A* fast path inside the local-replan window. When enabled,
  // replanFromCurrentPose first calls AStar3D::search on the active planning
  // map for a small lookahead goal and publishes the path directly as a
  // nav_msgs::Path, skipping the full EGO ladder (kinodynamic + JPS + spline
  // smoothing). Bypasses ~30-100 ms of optimization per replan, restoring the
  // promised "fast local replan" behaviour. Falls back to planWithActiveMap
  // when raw A* fails or returns < min_poses.
  bool safety_local_use_raw_astar_ = true;
  int safety_local_astar_max_iterations_ = 20000;
  nav3d::planner::AStar3D local_astar_;
  double controller_goal_tolerance_ = 0.15;
  bool tracking_active_ = false;
  double controller_control_dt_ = 0.1;
  int occupancy_grid_max_cells_ = 1000000;
  double occupancy_grid_min_z_ = 0.2;
  double occupancy_grid_max_z_ = 0.9;
  int uav_endpoint_snap_radius_cells_ = 12;
  double planning_inflation_radius_ = 0.0;
  std::size_t last_published_pose_count_ = 0;
  bool next_clicked_point_is_start_ = true;
  nav3d::map::MapBuildConfig map_build_config_;
  PcdLoaderBackend pcd_loader_backend_ = PcdLoaderBackend::Builtin;
  PlanningTraversabilityMode planning_traversability_mode_ = PlanningTraversabilityMode::Uav;
  bool planning_use_octomap_map_ = false;
  GroundSearchOptions ground_search_options_;
  nav3d::common::Point3D start_;
  std::optional<nav3d::common::Point3D> current_position_;
  std::optional<double> current_yaw_;
  std::optional<nav3d::common::Point3D> active_goal_;
  std::optional<nav3d::planner::UniformBspline> active_trajectory_;
  std::optional<nav3d::controller::TrajectoryCommand> last_controller_command_;
  std::unique_ptr<nav3d::map::VoxelGridMap> map_;
  std::unique_ptr<nav3d::map::OctomapManager> octomap_;
  std::unique_ptr<nav3d::map::LocalGrid> local_grid_;
  std::deque<LocalPointCloudFrame> local_pointcloud_frames_;
  std::unique_ptr<nav3d::planner::EgoPlannerCore> planner_;
  std::unique_ptr<nav3d::controller::SafetyMonitor> safety_monitor_;
  std::unique_ptr<nav3d::controller::TrajectoryTracker> trajectory_tracker_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupied_grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planning_occupied_markers_pub_;
#ifdef NAV3D_HAS_OCTOMAP_MSGS
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
#endif
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_load_sub_;
  rclcpp::Service<GetPlan>::SharedPtr plan_service_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr static_visualization_timer_;
  int static_visualization_republish_count_ = 0;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_action_server_;
  std::mutex pending_navigate_mutex_;
  std::optional<PendingNavigateGoal> pending_navigate_goal_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Nav3DBridgeNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("nav3d_bridge_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
