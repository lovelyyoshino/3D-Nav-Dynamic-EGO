#include <gtest/gtest.h>

#include "nav3d/collision/inflation_layer.h"
#include "nav3d/map/voxel_grid_map.h"

TEST(InflationLayer, TreatsNearObstacleAsOccupied)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {6.0, 1.0, 1.0});
  map.insertOccupied({3.0, 0.0, 0.0});
  const nav3d::collision::InflationLayer inflated(map, 1.25);

  EXPECT_TRUE(inflated.isOccupied({2.0, 0.0, 0.0}));
  EXPECT_FALSE(inflated.isFree({2.0, 0.0, 0.0}));
  EXPECT_FALSE(inflated.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(inflated.isFree({0.0, 0.0, 0.0}));
}

TEST(InflationLayer, PreservesBoundsResolutionAndDistance)
{
  nav3d::map::VoxelGridMap map(0.5);
  map.setExplicitBounds({-1.0, -2.0, 0.0}, {3.0, 4.0, 2.0});
  map.insertOccupied({1.0, 0.0, 0.0});
  const nav3d::collision::InflationLayer inflated(map, 0.75);

  EXPECT_DOUBLE_EQ(inflated.getResolution(), 0.5);
  EXPECT_EQ(inflated.getBounds().min, map.getBounds().min);
  EXPECT_EQ(inflated.getBounds().max, map.getBounds().max);
  EXPECT_DOUBLE_EQ(inflated.getDistance({2.0, 0.0, 0.0}), 0.25);
  EXPECT_FALSE(inflated.isInBounds({10.0, 0.0, 0.0}));
}

TEST(InflationLayer, RejectsInvalidRadius)
{
  nav3d::map::VoxelGridMap map(1.0);

  EXPECT_THROW(nav3d::collision::InflationLayer(map, -0.1), std::invalid_argument);
}
