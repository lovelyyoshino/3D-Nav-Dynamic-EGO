#pragma once

#include "nav3d/common/types.h"
#include "nav3d/map/i_map.h"

namespace nav3d::planner {

enum class PlanningMode {
  Mode3D,
  Mode2D,
};

enum class SearchStatus {
  Success,
  NoPath,
  InvalidInput,
  IterationLimit,
};

enum class SearchAlgorithm {
  AStar,
  Jps,
};

struct SearchOptions {
  SearchAlgorithm algorithm = SearchAlgorithm::AStar;
  PlanningMode mode = PlanningMode::Mode3D;
  bool allow_diagonal = true;
  int max_iterations = 200000;
};

struct SearchResult {
  SearchStatus status = SearchStatus::NoPath;
  common::Path3D path;
  int iterations = 0;
  double path_cost = 0.0;
};

class IPathSearcher {
public:
  virtual ~IPathSearcher() = default;

  virtual SearchResult search(
    const map::IMap& map,
    const common::Point3D& start,
    const common::Point3D& goal,
    const SearchOptions& options) const = 0;
};

}  // namespace nav3d::planner
