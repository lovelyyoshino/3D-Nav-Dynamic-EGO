#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/i_path_searcher.h"
#include "nav3d/planner/path_searching/astar_3d.h"
#include "nav3d/planner/path_searching/jps_3d.h"

namespace {

struct SearcherCase {
  std::string name;
  nav3d::planner::SearchAlgorithm algorithm = nav3d::planner::SearchAlgorithm::AStar;
  std::unique_ptr<nav3d::planner::IPathSearcher> searcher;
};

struct ScenarioCase {
  std::string name;
  nav3d::map::VoxelGridMap map;
  nav3d::common::Point3D start;
  nav3d::common::Point3D goal;
  nav3d::planner::PlanningMode mode = nav3d::planner::PlanningMode::Mode2D;
  bool allow_diagonal = false;
};

std::vector<SearcherCase> makeSearchers()
{
  std::vector<SearcherCase> searchers;
  searchers.push_back({
    "astar",
    nav3d::planner::SearchAlgorithm::AStar,
    std::make_unique<nav3d::planner::AStar3D>(),
  });
  searchers.push_back({
    "jps",
    nav3d::planner::SearchAlgorithm::Jps,
    std::make_unique<nav3d::planner::Jps3D>(),
  });
  return searchers;
}

void addWallWithGap(
  nav3d::map::VoxelGridMap& map,
  int x,
  int y_min,
  int y_max,
  int gap_y,
  int z)
{
  for (int y = y_min; y <= y_max; ++y) {
    if (y == gap_y) {
      continue;
    }
    map.insertOccupied({static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)});
  }
}

ScenarioCase makeOffsetCorridorScenario(int corridor_half_width, int start_y, int goal_y)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, -4.0, 0.0}, {10.5, 4.5, 0.5});
  for (int x = 0; x <= 10; ++x) {
    map.insertOccupied({static_cast<double>(x), static_cast<double>(corridor_half_width + 1), 0.0});
    map.insertOccupied({static_cast<double>(x), static_cast<double>(-corridor_half_width - 1), 0.0});
  }
  addWallWithGap(map, 5, -corridor_half_width, corridor_half_width, 0, 0);

  std::ostringstream name;
  name << "corridor_width_" << corridor_half_width
       << "_start_y_" << start_y
       << "_goal_y_" << goal_y;
  return {
    name.str(),
    std::move(map),
    {0.0, static_cast<double>(start_y), 0.0},
    {10.0, static_cast<double>(goal_y), 0.0},
    nav3d::planner::PlanningMode::Mode2D,
    false,
  };
}

ScenarioCase makeNearObstacleScenario(int obstacle_x, int obstacle_y, bool occupied_start, bool occupied_goal)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, -3.0, 0.0}, {8.5, 3.5, 0.5});
  map.insertOccupied({static_cast<double>(obstacle_x), static_cast<double>(obstacle_y), 0.0});
  map.insertOccupied({static_cast<double>(obstacle_x), static_cast<double>(-obstacle_y), 0.0});
  if (occupied_start) {
    map.insertOccupied({0.0, 0.0, 0.0});
  }
  if (occupied_goal) {
    map.insertOccupied({8.0, 0.0, 0.0});
  }

  std::ostringstream name;
  name << "near_obstacle_x_" << obstacle_x
       << "_y_" << obstacle_y
       << "_occupied_start_" << occupied_start
       << "_occupied_goal_" << occupied_goal;
  return {
    name.str(),
    std::move(map),
    {0.0, 0.0, 0.0},
    {8.0, 0.0, 0.0},
    nav3d::planner::PlanningMode::Mode2D,
    true,
  };
}

ScenarioCase make3DOverpassScenario(int wall_x, int gap_z, bool diagonal)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, -2.0, 0.0}, {6.5, 2.5, 3.5});
  for (int y = -2; y <= 2; ++y) {
    for (int z = 0; z <= 3; ++z) {
      if (z == gap_z) {
        continue;
      }
      map.insertOccupied({static_cast<double>(wall_x), static_cast<double>(y), static_cast<double>(z)});
    }
  }

  std::ostringstream name;
  name << "overpass_wall_x_" << wall_x
       << "_gap_z_" << gap_z
       << "_diagonal_" << diagonal;
  return {
    name.str(),
    std::move(map),
    {0.0, 0.0, static_cast<double>(gap_z)},
    {6.0, 0.0, static_cast<double>(gap_z)},
    nav3d::planner::PlanningMode::Mode3D,
    diagonal,
  };
}

std::vector<ScenarioCase> makeScenarios()
{
  std::vector<ScenarioCase> scenarios;

  for (int half_width : {1, 2, 3}) {
    for (int start_y = -half_width; start_y <= half_width; ++start_y) {
      for (int goal_y = -half_width; goal_y <= half_width; ++goal_y) {
        scenarios.push_back(makeOffsetCorridorScenario(half_width, start_y, goal_y));
      }
    }
  }

  for (int obstacle_x : {1, 2, 4, 6}) {
    for (int obstacle_y : {1, 2}) {
      for (bool occupied_start : {false, true}) {
        for (bool occupied_goal : {false, true}) {
          scenarios.push_back(
            makeNearObstacleScenario(obstacle_x, obstacle_y, occupied_start, occupied_goal));
        }
      }
    }
  }

  for (int wall_x : {2, 3, 4}) {
    for (int gap_z : {1, 2, 3}) {
      for (bool diagonal : {false, true}) {
        scenarios.push_back(make3DOverpassScenario(wall_x, gap_z, diagonal));
      }
    }
  }

  return scenarios;
}

int lateralDirectionChanges(const nav3d::common::Path3D& path)
{
  int changes = 0;
  int previous_sign = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double delta_y = path[i].y - path[i - 1].y;
    const int sign = delta_y > 0.0 ? 1 : (delta_y < 0.0 ? -1 : 0);
    if (sign == 0) {
      continue;
    }
    if (previous_sign != 0 && sign != previous_sign) {
      ++changes;
    }
    previous_sign = sign;
  }
  return changes;
}

std::string describeScenario(
  const SearcherCase& searcher_case,
  const ScenarioCase& scenario,
  int scenario_index)
{
  std::ostringstream out;
  out << "case=" << scenario_index
      << " searcher=" << searcher_case.name
      << " scenario=" << scenario.name;
  return out.str();
}

}  // namespace

TEST(PlanningScenarioMatrix, GeneratedMatrixCoversMoreThanOneHundredCases)
{
  const auto scenarios = makeScenarios();
  const auto searchers = makeSearchers();
  EXPECT_GE(scenarios.size() * searchers.size(), 100u);
}

TEST(PlanningScenarioMatrix, SearchersProduceCollisionFreePathsAcrossGeneratedCases)
{
  auto scenarios = makeScenarios();
  auto searchers = makeSearchers();
  int executed_cases = 0;

  for (const auto& searcher_case : searchers) {
    for (std::size_t i = 0; i < scenarios.size(); ++i) {
      const auto& scenario = scenarios[i];
      nav3d::planner::SearchOptions options;
      options.algorithm = searcher_case.algorithm;
      options.mode = scenario.mode;
      options.allow_diagonal = scenario.allow_diagonal;
      options.max_iterations = 100000;

      const auto result =
        searcher_case.searcher->search(scenario.map, scenario.start, scenario.goal, options);
      SCOPED_TRACE(describeScenario(searcher_case, scenario, static_cast<int>(i)));
      ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
      ASSERT_FALSE(result.path.empty());
      EXPECT_FALSE(scenario.map.isOccupied(result.path.front()));
      EXPECT_FALSE(scenario.map.isOccupied(result.path.back()));
      EXPECT_LE(nav3d::common::distance(result.path.front(), scenario.start), 1.0);
      EXPECT_LE(nav3d::common::distance(result.path.back(), scenario.goal), 1.0);
      for (const auto& point : result.path) {
        EXPECT_TRUE(scenario.map.isInBounds(point));
        EXPECT_FALSE(scenario.map.isOccupied(point));
      }
      ++executed_cases;
    }
  }

  EXPECT_GE(executed_cases, 100);
}

TEST(PlanningScenarioMatrix, NarrowCorridorPathsDoNotOscillateSideToSide)
{
  auto searchers = makeSearchers();
  for (const auto& searcher_case : searchers) {
    for (int half_width : {1, 2, 3}) {
      auto scenario = makeOffsetCorridorScenario(half_width, 0, 0);
      nav3d::planner::SearchOptions options;
      options.algorithm = searcher_case.algorithm;
      options.mode = scenario.mode;
      options.allow_diagonal = false;

      const auto result =
        searcher_case.searcher->search(scenario.map, scenario.start, scenario.goal, options);
      SCOPED_TRACE(describeScenario(searcher_case, scenario, half_width));
      ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
      EXPECT_LE(lateralDirectionChanges(result.path), 2);
    }
  }
}
