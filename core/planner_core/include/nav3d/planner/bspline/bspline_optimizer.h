#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "nav3d/common/types.h"
#include "nav3d/map/i_map.h"
#include "nav3d/planner/bspline/uniform_bspline.h"
#include "nav3d/planner/path_searching/astar_3d.h"

namespace nav3d::planner {

struct BsplineOptimizerConfig {
  int max_iterations = 80;
  int memory_size = 8;
  double gradient_tolerance = 1e-4;
  double initial_step = 0.35;
  double smoothness_weight = 1.0;
  double collision_weight = 8.0;
  double terminal_weight = 10.0;
  double fitness_weight = 0.01;
  double feasibility_weight = 0.0;
  double max_velocity = std::numeric_limits<double>::infinity();
  double max_acceleration = std::numeric_limits<double>::infinity();
  double interval = 0.2;
  SearchOptions rebound_search;
};

struct BsplineCostBreakdown {
  double total = 0.0;
  double smoothness = 0.0;
  double collision = 0.0;
  double terminal = 0.0;
  double fitness = 0.0;
  double feasibility = 0.0;
};

struct BsplineOptimizeResult {
  bool success = false;
  UniformBspline warm_start;
  UniformBspline spline;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  int iterations = 0;
  bool used_rebound = false;
  int rebound_segments = 0;
};

class BsplineOptimizer {
public:
  static UniformBspline makeWarmStart(
    const std::vector<common::Point3D>& path,
    double interval);

  static BsplineCostBreakdown evaluateCost(
    const std::vector<common::Point3D>& control_points,
    const BsplineOptimizerConfig& config);

  static BsplineOptimizeResult optimize(
    const std::vector<common::Point3D>& warm_start_path,
    const map::IMap& map,
    const BsplineOptimizerConfig& config);
};

}  // namespace nav3d::planner
