#include <gtest/gtest.h>

#include "nav3d/map/map_2d_projection.h"

TEST(Map2DProjection, ProjectsOccupiedVoxelsIntoRowMajor2DGrid)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.insertOccupied({0.1, 0.1, 0.1});
  map.insertOccupied({2.1, 1.1, 3.0});

  const auto projected = nav3d::map::Map2DProjection::projectOccupiedVoxels(map, 100);

  ASSERT_TRUE(projected.has_value());
  EXPECT_DOUBLE_EQ(projected->resolution, 1.0);
  EXPECT_DOUBLE_EQ(projected->origin.x, 0.0);
  EXPECT_DOUBLE_EQ(projected->origin.y, 0.0);
  EXPECT_DOUBLE_EQ(projected->origin.z, 0.0);
  EXPECT_EQ(projected->width, 3);
  EXPECT_EQ(projected->height, 2);
  EXPECT_EQ(projected->occupied_count, 2u);
  ASSERT_EQ(projected->data.size(), 6u);
  EXPECT_EQ(projected->data[0], 100);
  EXPECT_EQ(projected->data[1], 0);
  EXPECT_EQ(projected->data[2], 0);
  EXPECT_EQ(projected->data[3], 0);
  EXPECT_EQ(projected->data[4], 0);
  EXPECT_EQ(projected->data[5], 100);
}

TEST(Map2DProjection, FiltersOccupiedVoxelsByConfiguredHeightRange)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.insertOccupied({0.1, 0.1, 0.1});
  map.insertOccupied({1.1, 0.1, 2.1});
  map.insertOccupied({2.1, 0.1, 5.1});

  nav3d::map::Map2DProjectionOptions options;
  options.max_cells = 100;
  options.min_z = 0.0;
  options.max_z = 2.0;

  const auto projected = nav3d::map::Map2DProjection::projectOccupiedVoxels(map, options);

  ASSERT_TRUE(projected.has_value());
  ASSERT_EQ(projected->data.size(), 3u);
  EXPECT_EQ(projected->occupied_count, 1u);
  EXPECT_EQ(projected->data[0], 100);
  EXPECT_EQ(projected->data[1], 0);
  EXPECT_EQ(projected->data[2], 0);
}

TEST(Map2DProjection, CanKeepUnoccupiedCellsUnknownWhenRequested)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.insertOccupied({0.1, 0.1, 0.1});
  map.insertOccupied({2.1, 0.1, 0.1});

  nav3d::map::Map2DProjectionOptions options;
  options.max_cells = 100;
  options.unknown_by_default = true;

  const auto projected = nav3d::map::Map2DProjection::projectOccupiedVoxels(map, options);

  ASSERT_TRUE(projected.has_value());
  ASSERT_EQ(projected->data.size(), 3u);
  EXPECT_EQ(projected->occupied_count, 2u);
  EXPECT_EQ(projected->data[0], 100);
  EXPECT_EQ(projected->data[1], -1);
  EXPECT_EQ(projected->data[2], 100);
}

TEST(Map2DProjection, MarksOnlyPredicateApprovedCellsAsFree)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.insertOccupied({0.1, 0.1, 0.1});
  map.insertOccupied({3.1, 0.1, 0.1});

  nav3d::map::Map2DProjectionOptions options;
  options.max_cells = 100;
  options.free_cell_z = 0.5;
  options.is_free_cell = [](const nav3d::common::Point3D& point) {
    return point.x > 1.0 && point.x < 2.0 && point.z == 0.5;
  };

  const auto projected = nav3d::map::Map2DProjection::projectOccupiedVoxels(map, options);

  ASSERT_TRUE(projected.has_value());
  ASSERT_EQ(projected->data.size(), 4u);
  EXPECT_EQ(projected->occupied_count, 2u);
  EXPECT_EQ(projected->data[0], 100);
  EXPECT_EQ(projected->data[1], 0);
  EXPECT_EQ(projected->data[2], -1);
  EXPECT_EQ(projected->data[3], 100);
}

TEST(Map2DProjection, ReturnsNulloptWhenProjectionExceedsCellLimit)
{
  nav3d::map::VoxelGridMap map(1.0);
  map.insertOccupied({0.0, 0.0, 0.0});
  map.insertOccupied({3.0, 2.0, 0.0});

  const auto projected = nav3d::map::Map2DProjection::projectOccupiedVoxels(map, 11);

  EXPECT_FALSE(projected.has_value());
}

TEST(Map2DProjection, ReturnsNulloptForEmptyMap)
{
  const nav3d::map::VoxelGridMap map(1.0);

  const auto projected = nav3d::map::Map2DProjection::projectOccupiedVoxels(map, 100);

  EXPECT_FALSE(projected.has_value());
}
