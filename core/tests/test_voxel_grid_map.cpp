#include <gtest/gtest.h>

#include "nav3d/map/voxel_grid_map.h"

TEST(VoxelGridMap, BuildsOccupancyFromPointCloud)
{
  nav3d::map::PointCloud cloud;
  cloud.points = {{0.1, 0.2, 0.3}, {1.1, 0.0, 0.0}, {1.2, 0.1, 0.0}};

  const auto map = nav3d::map::VoxelGridMap::fromPointCloud(cloud, 0.5);

  EXPECT_TRUE(map.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(map.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_EQ(map.occupiedCells().size(), 2u);
}

TEST(VoxelGridMap, StoresOccupiedCellsAtConfiguredResolution)
{
  nav3d::map::VoxelGridMap map(0.5);
  map.insertOccupied({1.1, 2.0, 0.1});

  EXPECT_TRUE(map.isOccupied({1.24, 2.24, 0.24}));
  EXPECT_FALSE(map.isOccupied({1.51, 2.0, 0.1}));
  EXPECT_TRUE(map.isFree({1.51, 2.0, 0.1}));
  EXPECT_DOUBLE_EQ(map.getResolution(), 0.5);
}

TEST(VoxelGridMap, ComputesApproximateDistanceToNearestOccupiedVoxel)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {5.0, 5.0, 5.0});
  map.insertOccupied({0.0, 0.0, 0.0});
  map.insertOccupied({5.0, 0.0, 0.0});

  EXPECT_NEAR(map.getDistance({2.0, 0.0, 0.0}), 2.0, 1e-9);
  EXPECT_TRUE(map.isInBounds({0.0, 0.0, 0.0}));
  EXPECT_FALSE(map.isInBounds({100.0, 0.0, 0.0}));
}
