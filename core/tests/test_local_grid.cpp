#include <gtest/gtest.h>

#include "nav3d/map/local_grid.h"
#include "nav3d/map/map_composite.h"
#include "nav3d/map/voxel_grid_map.h"

TEST(LocalGrid, TracksFreeOccupiedAndUnknownCells)
{
  nav3d::map::LocalGrid grid(0.5, {-1.0, -1.0, 0.0}, {2.0, 2.0, 1.0});

  grid.markFree({0.1, 0.1, 0.1});
  grid.markOccupied({1.0, 0.0, 0.0});

  EXPECT_TRUE(grid.hasObservation({0.0, 0.0, 0.0}));
  EXPECT_TRUE(grid.isFree({0.1, 0.1, 0.1}));
  EXPECT_FALSE(grid.isOccupied({0.1, 0.1, 0.1}));
  EXPECT_TRUE(grid.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_FALSE(grid.isFree({1.0, 0.0, 0.0}));
  EXPECT_FALSE(grid.hasObservation({1.5, 1.5, 0.0}));
  EXPECT_FALSE(grid.isFree({1.5, 1.5, 0.0}));
}

TEST(LocalGrid, ClearsOldObservations)
{
  nav3d::map::LocalGrid grid(1.0, {0.0, 0.0, 0.0}, {2.0, 2.0, 2.0});
  grid.markOccupied({1.0, 1.0, 1.0});

  ASSERT_TRUE(grid.isOccupied({1.0, 1.0, 1.0}));
  grid.clear();

  EXPECT_FALSE(grid.hasObservation({1.0, 1.0, 1.0}));
  EXPECT_FALSE(grid.isOccupied({1.0, 1.0, 1.0}));
}

TEST(LocalGrid, RayClearingMarksFreeCellsAndOccupiedEndpoint)
{
  nav3d::map::LocalGrid grid(1.0, {0.0, 0.0, 0.0}, {5.0, 1.0, 1.0});

  grid.markRayFreeAndOccupied({0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

  EXPECT_TRUE(grid.isFree({0.0, 0.0, 0.0}));
  EXPECT_TRUE(grid.isFree({1.0, 0.0, 0.0}));
  EXPECT_TRUE(grid.isFree({2.0, 0.0, 0.0}));
  EXPECT_TRUE(grid.isOccupied({3.0, 0.0, 0.0}));
  EXPECT_FALSE(grid.isFree({3.0, 0.0, 0.0}));
}

TEST(LocalGrid, RayClearingCanOverrideStaleGlobalObstacle)
{
  nav3d::map::VoxelGridMap global(1.0);
  global.setExplicitBounds({0.0, 0.0, 0.0}, {5.0, 1.0, 1.0});
  global.insertOccupied({1.0, 0.0, 0.0});

  nav3d::map::LocalGrid local(1.0, {0.0, 0.0, 0.0}, {5.0, 1.0, 1.0});
  local.markRayFreeAndOccupied({0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
  nav3d::map::MapComposite composite(global, local);

  EXPECT_TRUE(composite.isFree({1.0, 0.0, 0.0}));
  EXPECT_FALSE(composite.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_TRUE(composite.isOccupied({3.0, 0.0, 0.0}));
}
