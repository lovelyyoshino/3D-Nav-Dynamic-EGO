#include <gtest/gtest.h>

#include "nav3d/map/i_map.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/path_searching/astar_3d.h"

namespace {

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

TEST(AStar3D, Finds3DPathAroundBlockedCell)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {4.5, 4.5, 2.5});
  map.insertOccupied({1, 0, 0});

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode3D;
  options.allow_diagonal = false;

  const auto result = astar.search(map, {0, 0, 0}, {2, 0, 0}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_GE(result.path.size(), 5u);
  EXPECT_EQ(result.path.front(), nav3d::common::Point3D(0.5, 0.5, 0.5));
  EXPECT_EQ(result.path.back(), nav3d::common::Point3D(2.5, 0.5, 0.5));
  for (const auto& p : result.path) {
    EXPECT_FALSE(map.isOccupied(p));
  }
}

TEST(AStar3D, ResolvesOccupiedGoalToAdjacentFreeCell)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, -2, 0}, {4.5, 2.5, 0.5});
  map.insertOccupied({2, 0, 0});

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = false;

  const auto result = astar.search(map, {0, 0, 0}, {2, 0, 0}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_FALSE(result.path.empty());
  EXPECT_FALSE(map.isOccupied(result.path.back()));
  EXPECT_LE(nav3d::common::distance(result.path.back(), {2, 0, 0}), 1.0);
}

TEST(AStar3D, ResolvesOccupiedStartToAdjacentFreeCell)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, -2, 0}, {4.5, 2.5, 0.5});
  map.insertOccupied({0, 0, 0});

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = false;

  const auto result = astar.search(map, {0, 0, 0}, {3, 0, 0}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_FALSE(result.path.empty());
  EXPECT_FALSE(map.isOccupied(result.path.front()));
  EXPECT_LE(nav3d::common::distance(result.path.front(), {0, 0, 0}), 1.0);
  EXPECT_EQ(result.path.back(), nav3d::common::Point3D(3.5, 0.5, 0.5));
}

TEST(AStar3D, LocksZIn2DMode)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {3.5, 3.5, 3.5});

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = true;

  const auto result = astar.search(map, {0, 0, 1}, {3, 3, 3}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  for (const auto& p : result.path) {
    EXPECT_DOUBLE_EQ(p.z, 1.5);
  }
  EXPECT_EQ(result.path.back(), nav3d::common::Point3D(3.5, 3.5, 1.5));
}

TEST(AStar3D, RejectsDiagonalCornerCutThroughBlockedCells)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {1.5, 1.5, 0.5});
  map.insertOccupied({1, 0, 0});
  map.insertOccupied({0, 1, 0});

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = true;

  const auto result = astar.search(map, {0, 0, 0}, {1, 1, 0}, options);

  EXPECT_EQ(result.status, nav3d::planner::SearchStatus::NoPath);
  EXPECT_TRUE(result.path.empty());
}

TEST(AStar3D, RejectsEdgeWhoseContinuousSegmentCrossesObstacle)
{
  MidSegmentObstacleMap map;

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = false;

  const auto result = astar.search(map, {0, 0, 0}, {1, 0, 0}, options);

  EXPECT_EQ(result.status, nav3d::planner::SearchStatus::NoPath);
  EXPECT_TRUE(result.path.empty());
}

TEST(AStar3D, Rejects3DDiagonalThroughBlockedFaceNeighbors)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0, 0, 0}, {1.5, 1.5, 1.5});
  map.insertOccupied({1, 1, 0});
  map.insertOccupied({1, 0, 1});
  map.insertOccupied({0, 1, 1});

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode3D;
  options.allow_diagonal = true;

  const auto result = astar.search(map, {0, 0, 0}, {1, 1, 1}, options);

  EXPECT_EQ(result.status, nav3d::planner::SearchStatus::NoPath);
  EXPECT_TRUE(result.path.empty());
}
