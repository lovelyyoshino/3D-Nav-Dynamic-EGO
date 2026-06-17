#include <gtest/gtest.h>

#include "nav3d/map/local_grid.h"
#include "nav3d/map/map_composite.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/path_searching/astar_3d.h"

TEST(MapComposite, FallsBackToGlobalMapWhenLocalCellIsUnknown)
{
  nav3d::map::VoxelGridMap global(1.0);
  global.setExplicitBounds({0.0, 0.0, 0.0}, {4.0, 4.0, 1.0});
  global.insertOccupied({2.0, 0.0, 0.0});
  nav3d::map::LocalGrid local(1.0, {0.0, 0.0, 0.0}, {4.0, 4.0, 1.0});
  nav3d::map::MapComposite composite(global, local);

  EXPECT_TRUE(composite.isOccupied({2.0, 0.0, 0.0}));
  EXPECT_FALSE(composite.isFree({2.0, 0.0, 0.0}));
}

TEST(MapComposite, LocalOccupiedObservationOverridesGlobalFreeSpace)
{
  nav3d::map::VoxelGridMap global(1.0);
  global.setExplicitBounds({0.0, 0.0, 0.0}, {4.0, 4.0, 1.0});
  nav3d::map::LocalGrid local(1.0, {0.0, 0.0, 0.0}, {4.0, 4.0, 1.0});
  local.markOccupied({1.0, 0.0, 0.0});
  nav3d::map::MapComposite composite(global, local);

  EXPECT_TRUE(composite.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_FALSE(composite.isFree({1.0, 0.0, 0.0}));
}

TEST(MapComposite, LocalFreeObservationOverridesStaleGlobalObstacleForAStar)
{
  nav3d::map::VoxelGridMap global(1.0);
  global.setExplicitBounds({0.0, 0.0, 0.0}, {3.0, 2.0, 1.0});
  global.insertOccupied({1.0, 0.0, 0.0});

  nav3d::map::LocalGrid local(1.0, {0.0, 0.0, 0.0}, {3.0, 2.0, 1.0});
  local.markFree({1.0, 0.0, 0.0});
  nav3d::map::MapComposite composite(global, local);

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = false;

  const auto result = astar.search(composite, {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_EQ(result.path.size(), 3u);
  EXPECT_EQ(global.worldToGrid(result.path[1]), global.worldToGrid({1.0, 0.0, 0.0}));
  EXPECT_TRUE(composite.isFree(result.path[1]));
  EXPECT_FALSE(composite.isOccupied(result.path[1]));
}

TEST(MapComposite, LocalObstacleCanForceAStarAroundNewObstacle)
{
  nav3d::map::VoxelGridMap global(1.0);
  global.setExplicitBounds({0.0, 0.0, 0.0}, {3.0, 2.0, 1.0});

  nav3d::map::LocalGrid local(1.0, {0.0, 0.0, 0.0}, {3.0, 2.0, 1.0});
  local.markOccupied({1.0, 0.0, 0.0});
  nav3d::map::MapComposite composite(global, local);

  nav3d::planner::AStar3D astar;
  nav3d::planner::SearchOptions options;
  options.mode = nav3d::planner::PlanningMode::Mode2D;
  options.allow_diagonal = false;

  const auto result = astar.search(composite, {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, options);

  ASSERT_EQ(result.status, nav3d::planner::SearchStatus::Success);
  ASSERT_GT(result.path.size(), 3u);
  for (const auto& point : result.path) {
    EXPECT_FALSE(composite.isOccupied(point));
  }
}

TEST(MapComposite, DistanceStillSeesGlobalObstaclesWhenLocalObstacleExistsElsewhere)
{
  nav3d::map::VoxelGridMap global(1.0);
  global.setExplicitBounds({0.0, 0.0, 0.0}, {5.0, 1.0, 1.0});
  global.insertOccupied({1.0, 0.0, 0.0});

  nav3d::map::LocalGrid local(1.0, {0.0, 0.0, 0.0}, {5.0, 1.0, 1.0});
  local.markOccupied({4.0, 0.0, 0.0});
  nav3d::map::MapComposite composite(global, local);

  EXPECT_DOUBLE_EQ(composite.getDistance({1.0, 0.0, 0.0}), 0.0);
}

TEST(MapComposite, DistanceUsesLocalFreeObservationToClearStaleGlobalObstacle)
{
  nav3d::map::VoxelGridMap global(1.0);
  global.setExplicitBounds({0.0, 0.0, 0.0}, {5.0, 1.0, 1.0});
  global.insertOccupied({1.0, 0.0, 0.0});

  nav3d::map::LocalGrid local(1.0, {0.0, 0.0, 0.0}, {5.0, 1.0, 1.0});
  local.markFree({1.0, 0.0, 0.0});
  local.markOccupied({4.0, 0.0, 0.0});
  nav3d::map::MapComposite composite(global, local);

  EXPECT_DOUBLE_EQ(composite.getDistance({1.0, 0.0, 0.0}), 3.0);
}
