#include <gtest/gtest.h>

#include <cmath>

#include "nav3d/map/i_map.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/path_searching/astar_3d.h"
#include "nav3d/planner/path_searching/jps_3d.h"

namespace {

double pathCost(const nav3d::common::Path3D& path)
{
  double cost = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    cost += nav3d::common::distance(path[i - 1], path[i]);
  }
  return cost;
}

class MidSegmentObstacleMap final : public nav3d::map::IMap {
public:
  bool isOccupied(const nav3d::common::Point3D& p) const override
  {
    return p.x > 0.9 && p.x < 1.1 &&
           p.y > 0.4 && p.y < 0.6 &&
           p.z > 0.4 && p.z < 0.6;
  }

  bool isFree(const nav3d::common::Point3D& p) const override
  {
    return isInBounds(p) && !isOccupied(p);
  }

  double getDistance(const nav3d::common::Point3D&) const override { return 0.0; }

  bool isInBounds(const nav3d::common::Point3D& p) const override
  {
    return p.x >= 0.0 && p.x <= 2.0 &&
           p.y >= 0.0 && p.y <= 1.0 &&
           p.z >= 0.0 && p.z <= 1.0;
  }

  double getResolution() const override { return 1.0; }

  nav3d::common::BoundingBox getBounds() const override
  {
    return {{0.0, 0.0, 0.0}, {2.0, 1.0, 1.0}, true};
  }
};

}  // namespace

TEST(Jps3D, JumpsAcrossOpenStraightLineWithFewerIterationsThanAStar)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {8.5, 0.5, 0.5});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode3D;
  options.allow_diagonal = false;

  const auto astar_result =
    nav3d::planner::AStar3D{}.search(map, {0, 0, 0}, {8, 0, 0}, options);
  const auto jps_result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {8, 0, 0}, options);

  ASSERT_EQ(astar_result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_EQ(jps_result.status, nav3d::planner::SearchStatus::Success);
  EXPECT_EQ(jps_result.path.front(), nav3d::common::Point3D(0.5, 0.5, 0.5));
  EXPECT_EQ(jps_result.path.back(), nav3d::common::Point3D(8.5, 0.5, 0.5));
  EXPECT_LT(jps_result.iterations, astar_result.iterations);
}

TEST(Jps3D, PrunesLargeOpenStraightLineWithSameCostAsAStar)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {80.5, 0.5, 0.5});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode3D;
  options.allow_diagonal = false;

  const auto astar_result =
    nav3d::planner::AStar3D{}.search(map, {0, 0, 0}, {80, 0, 0}, options);
  const auto jps_result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {80, 0, 0}, options);

  ASSERT_EQ(astar_result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_EQ(jps_result.status, nav3d::planner::SearchStatus::Success);
  EXPECT_NEAR(jps_result.path_cost, astar_result.path_cost, 1e-9);
  EXPECT_EQ(jps_result.path.front(), nav3d::common::Point3D(0.5, 0.5, 0.5));
  EXPECT_EQ(jps_result.path.back(), nav3d::common::Point3D(80.5, 0.5, 0.5));
  EXPECT_LT(jps_result.iterations * 4, astar_result.iterations);
}

TEST(Jps3D, Finds3DPathAroundBlockedCell)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {4.5, 4.5, 2.5});
  map.insertOccupied({1, 0, 0});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode3D;
  options.allow_diagonal = false;

  const auto result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {2, 0, 0}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  EXPECT_EQ(result.path.front(), nav3d::common::Point3D(0.5, 0.5, 0.5));
  EXPECT_EQ(result.path.back(), nav3d::common::Point3D(2.5, 0.5, 0.5));
  for (const auto& point : result.path) {
    EXPECT_FALSE(map.isOccupied(point));
  }
}

TEST(Jps3D, ResolvesOccupiedGoalToAdjacentFreeCell)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, -2, 0}, {4.5, 2.5, 0.5});
  map.insertOccupied({2, 0, 0});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = false;

  const auto result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {2, 0, 0}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_FALSE(result.path.empty());
  EXPECT_FALSE(map.isOccupied(result.path.back()));
  EXPECT_LE(nav3d::common::distance(result.path.back(), {2, 0, 0}), 1.0);
}

TEST(Jps3D, ResolvesOccupiedStartToAdjacentFreeCell)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, -2, 0}, {4.5, 2.5, 0.5});
  map.insertOccupied({0, 0, 0});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = false;

  const auto result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {3, 0, 0}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_FALSE(result.path.empty());
  EXPECT_FALSE(map.isOccupied(result.path.front()));
  EXPECT_LE(nav3d::common::distance(result.path.front(), {0, 0, 0}), 1.0);
  EXPECT_EQ(result.path.back(), nav3d::common::Point3D(3.5, 0.5, 0.5));
}

TEST(Jps3D, LocksZIn2DMode)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 1}, {4.5, 4.5, 1.5});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = true;

  const auto result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 1}, {4, 4, 3}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  for (const auto& point : result.path) {
    EXPECT_DOUBLE_EQ(point.z, 1.5);
  }
  EXPECT_EQ(result.path.back(), nav3d::common::Point3D(4.5, 4.5, 1.5));
}

TEST(Jps3D, RejectsDiagonalCornerCutThroughBlockedCells)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {1.5, 1.5, 0.5});
  map.insertOccupied({1, 0, 0});
  map.insertOccupied({0, 1, 0});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = true;

  const auto result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {1, 1, 0}, options);

  EXPECT_EQ(result.status, nav3d::planner::SearchStatus::NoPath);
  EXPECT_TRUE(result.path.empty());
}

TEST(Jps3D, RejectsEdgeWhoseContinuousSegmentCrossesObstacle)
{
  MidSegmentObstacleMap map;

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = false;

  const auto result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {1, 0, 0}, options);

  EXPECT_EQ(result.status, nav3d::planner::SearchStatus::NoPath);
  EXPECT_TRUE(result.path.empty());
}

TEST(Jps3D, ReportsSamePathCostAsAStarOnBoundedObstacleMap)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {4.5, 4.5, 0.5});
  map.insertOccupied({1, 1, 0});
  map.insertOccupied({2, 2, 0});
  map.insertOccupied({3, 1, 0});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = true;

  const auto astar_result =
    nav3d::planner::AStar3D{}.search(map, {0, 0, 0}, {4, 4, 0}, options);
  const auto jps_result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {4, 4, 0}, options);

  ASSERT_EQ(astar_result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_EQ(jps_result.status, nav3d::planner::SearchStatus::Success);
  EXPECT_DOUBLE_EQ(astar_result.path_cost, pathCost(astar_result.path));
  EXPECT_DOUBLE_EQ(jps_result.path_cost, pathCost(jps_result.path));
  EXPECT_NEAR(jps_result.path_cost, astar_result.path_cost, 1e-9);
}

TEST(Jps3D, Rejects3DDiagonalThroughBlockedFaceNeighbors)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {1.5, 1.5, 1.5});
  map.insertOccupied({1, 1, 0});
  map.insertOccupied({1, 0, 1});
  map.insertOccupied({0, 1, 1});

  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode3D;
  options.allow_diagonal = true;

  const auto result =
    nav3d::planner::Jps3D{}.search(map, {0, 0, 0}, {1, 1, 1}, options);

  EXPECT_EQ(result.status, nav3d::planner::SearchStatus::NoPath);
  EXPECT_TRUE(result.path.empty());
}
