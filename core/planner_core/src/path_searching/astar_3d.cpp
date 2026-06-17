#include "nav3d/planner/path_searching/astar_3d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace nav3d::planner {
namespace {

struct SearchIndex {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const SearchIndex& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct SearchIndexHash {
  std::size_t operator()(const SearchIndex& idx) const
  {
    const std::size_t h1 = std::hash<int>{}(idx.x);
    const std::size_t h2 = std::hash<int>{}(idx.y);
    const std::size_t h3 = std::hash<int>{}(idx.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct QueueNode {
  SearchIndex idx;
  double f = 0.0;
  double g = 0.0;
};

struct QueueNodeCompare {
  bool operator()(const QueueNode& a, const QueueNode& b) const
  {
    constexpr double kEpsilon = 1e-12;
    if (std::abs(a.f - b.f) > kEpsilon) {
      return a.f > b.f;
    }
    return a.g < b.g;
  }
};

SearchIndex worldToIndex(const common::Point3D& p, double resolution)
{
  return {
    static_cast<int>(std::floor(p.x / resolution + 1e-9)),
    static_cast<int>(std::floor(p.y / resolution + 1e-9)),
    static_cast<int>(std::floor(p.z / resolution + 1e-9)),
  };
}

common::Point3D indexToWorld(const SearchIndex& idx, double resolution)
{
  return {
    (static_cast<double>(idx.x) + 0.5) * resolution,
    (static_cast<double>(idx.y) + 0.5) * resolution,
    (static_cast<double>(idx.z) + 0.5) * resolution,
  };
}

double indexDistance(const SearchIndex& a, const SearchIndex& b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool isFree(const map::IMap& map, const SearchIndex& idx, double resolution)
{
  return map.isFree(indexToWorld(idx, resolution));
}

bool segmentIsFree(
  const map::IMap& map,
  const common::Point3D& from,
  const common::Point3D& to,
  double resolution)
{
  const double step = std::max(1e-3, resolution * 0.5);
  const double distance = common::distance(from, to);
  if (!std::isfinite(distance) || distance <= step) {
    return true;
  }

  const int subdivisions = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int i = 1; i < subdivisions; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(subdivisions);
    if (!map.isFree(from + (to - from) * ratio)) {
      return false;
    }
  }
  return true;
}

bool canMove(
  const map::IMap& map,
  const SearchIndex& from,
  const SearchIndex& direction,
  double resolution)
{
  const SearchIndex target{
    from.x + direction.x,
    from.y + direction.y,
    from.z + direction.z,
  };
  if (!isFree(map, target, resolution)) {
    return false;
  }
  if (!segmentIsFree(map, indexToWorld(from, resolution), indexToWorld(target, resolution), resolution)) {
    return false;
  }

  for (int x_step = 0; x_step <= (direction.x != 0 ? 1 : 0); ++x_step) {
    for (int y_step = 0; y_step <= (direction.y != 0 ? 1 : 0); ++y_step) {
      for (int z_step = 0; z_step <= (direction.z != 0 ? 1 : 0); ++z_step) {
        if (x_step == 0 && y_step == 0 && z_step == 0) {
          continue;
        }
        const SearchIndex intermediate{
          from.x + x_step * direction.x,
          from.y + y_step * direction.y,
          from.z + z_step * direction.z,
        };
        if (!isFree(map, intermediate, resolution)) {
          return false;
        }
      }
    }
  }
  return true;
}

std::vector<SearchIndex> makeDirections(const SearchOptions& options)
{
  std::vector<SearchIndex> directions;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        if (options.mode == PlanningMode::Mode2D && dz != 0) {
          continue;
        }
        if (!options.allow_diagonal && std::abs(dx) + std::abs(dy) + std::abs(dz) != 1) {
          continue;
        }
        directions.push_back({dx, dy, dz});
      }
    }
  }
  return directions;
}

std::vector<SearchIndex> reconstructPath(
  const std::unordered_map<SearchIndex, SearchIndex, SearchIndexHash>& came_from,
  SearchIndex current)
{
  std::vector<SearchIndex> path{current};
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    path.push_back(current);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

double pathCost(const std::vector<SearchIndex>& path)
{
  double cost = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    cost += indexDistance(path[i - 1], path[i]);
  }
  return cost;
}

common::Point3D snapGoalForMode(const common::Point3D& start, const common::Point3D& goal, PlanningMode mode)
{
  if (mode == PlanningMode::Mode2D) {
    return {goal.x, goal.y, start.z};
  }
  return goal;
}

SearchIndex resolveFreeIndex(
  const map::IMap& map,
  const SearchIndex& requested,
  double resolution,
  const SearchOptions& options)
{
  if (isFree(map, requested, resolution)) {
    return requested;
  }

  const int max_radius = std::max(1, std::min(options.max_iterations, 96));
  SearchIndex best = requested;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int radius = 1; radius <= max_radius; ++radius) {
    bool found = false;
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        const int min_dz = options.mode == PlanningMode::Mode2D ? 0 : -radius;
        const int max_dz = options.mode == PlanningMode::Mode2D ? 0 : radius;
        for (int dz = min_dz; dz <= max_dz; ++dz) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) {
            continue;
          }
          const SearchIndex candidate{
            requested.x + dx,
            requested.y + dy,
            requested.z + dz,
          };
          if (!isFree(map, candidate, resolution)) {
            continue;
          }
          const double candidate_distance = indexDistance(candidate, requested);
          if (candidate_distance < best_distance) {
            best = candidate;
            best_distance = candidate_distance;
            found = true;
          }
        }
      }
    }
    if (found) {
      return best;
    }
  }

  return requested;
}

}  // namespace

SearchResult AStar3D::search(
  const map::IMap& map,
  const common::Point3D& start,
  const common::Point3D& goal,
  const SearchOptions& options) const
{
  if (map.getResolution() <= 0.0 || options.max_iterations <= 0) {
    return {SearchStatus::InvalidInput, {}, 0};
  }

  const common::Point3D effective_goal = snapGoalForMode(start, goal, options.mode);
  const double resolution = map.getResolution();
  const SearchIndex start_idx = resolveFreeIndex(
    map,
    worldToIndex(start, resolution),
    resolution,
    options);
  const SearchIndex goal_idx = resolveFreeIndex(
    map,
    worldToIndex(effective_goal, resolution),
    resolution,
    options);
  if (!isFree(map, start_idx, resolution) || !isFree(map, goal_idx, resolution)) {
    return {SearchStatus::InvalidInput, {}, 0};
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
  std::unordered_map<SearchIndex, double, SearchIndexHash> g_score;
  std::unordered_map<SearchIndex, SearchIndex, SearchIndexHash> came_from;
  std::unordered_set<SearchIndex, SearchIndexHash> closed_set;

  g_score[start_idx] = 0.0;
  open_set.push({start_idx, indexDistance(start_idx, goal_idx), 0.0});
  const auto directions = makeDirections(options);

  int iterations = 0;
  while (!open_set.empty() && iterations < options.max_iterations) {
    const QueueNode current = open_set.top();
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
      SearchResult result;
      result.status = SearchStatus::Success;
      result.iterations = iterations;
      const auto path = reconstructPath(came_from, current.idx);
      result.path_cost = pathCost(path) * resolution;
      for (const auto& idx : path) {
        result.path.push_back(indexToWorld(idx, resolution));
      }
      return result;
    }

    for (const auto& direction : directions) {
      const SearchIndex neighbor{
        current.idx.x + direction.x,
        current.idx.y + direction.y,
        current.idx.z + direction.z,
      };
      if (closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }
      if (!canMove(map, current.idx, direction, resolution)) {
        continue;
      }
      const double tentative_g = current.g + indexDistance(current.idx, neighbor);
      const auto existing = g_score.find(neighbor);
      if (existing == g_score.end() || tentative_g < existing->second) {
        came_from[neighbor] = current.idx;
        g_score[neighbor] = tentative_g;
        open_set.push({neighbor, tentative_g + indexDistance(neighbor, goal_idx), tentative_g});
      }
    }
  }

  return {
    iterations >= options.max_iterations ? SearchStatus::IterationLimit : SearchStatus::NoPath,
    {},
    iterations,
  };
}

}  // namespace nav3d::planner
