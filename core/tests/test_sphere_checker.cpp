#include <gtest/gtest.h>

#include "nav3d/collision/sphere_checker.h"
#include "nav3d/map/voxel_grid_map.h"

TEST(SphereChecker, ReportsCollisionWhenCenterIsInsideObstacle)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {4.0, 4.0, 4.0});
  map.insertOccupied({1.0, 1.0, 1.0});
  const nav3d::collision::SphereChecker checker(0.25);

  const auto result = checker.check(map, {1.0, 1.0, 1.0});

  EXPECT_TRUE(result.in_collision);
  EXPECT_EQ(result.nearest_obstacle_distance, 0.0);
  EXPECT_DOUBLE_EQ(result.clearance, -0.25);
}

TEST(SphereChecker, UsesRadiusAndNearestObstacleDistance)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.setExplicitBounds({0.0, 0.0, 0.0}, {6.0, 1.0, 1.0});
  map.insertOccupied({3.0, 0.0, 0.0});
  const nav3d::collision::SphereChecker checker(1.25);

  const auto near = checker.check(map, {2.0, 0.0, 0.0});
  const auto far = checker.check(map, {0.0, 0.0, 0.0});

  EXPECT_TRUE(near.in_collision);
  EXPECT_DOUBLE_EQ(near.nearest_obstacle_distance, 1.0);
  EXPECT_DOUBLE_EQ(near.clearance, -0.25);
  EXPECT_FALSE(far.in_collision);
  EXPECT_DOUBLE_EQ(far.nearest_obstacle_distance, 3.0);
  EXPECT_DOUBLE_EQ(far.clearance, 1.75);
}

TEST(SphereChecker, RejectsInvalidRadius)
{
  EXPECT_THROW(nav3d::collision::SphereChecker(-0.1), std::invalid_argument);
}
