#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "nav3d/map/map_builder.h"

namespace {

std::string writeBuilderAsciiPcd()
{
  const std::string path = "test_builder_map.pcd";
  std::ofstream out(path);
  out << "# .PCD v0.7\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH 5\n"
      << "HEIGHT 1\n"
      << "POINTS 5\n"
      << "DATA ascii\n"
      << "0.1 0.1 0.1\n"
      << "0.2 0.2 0.2\n"
      << "1.1 0.1 0.1\n"
      << "1.2 0.2 0.2\n"
      << "5.0 0.0 0.0\n";
  return path;
}

}  // namespace

TEST(MapBuilder, BuildsFilteredVoxelMapFromPcd)
{
  nav3d::map::MapBuildConfig config;
  config.pcd_path = writeBuilderAsciiPcd();
  config.preprocessor.resolution = 1.0;
  config.preprocessor.min_points_per_voxel = 2;
  config.preprocessor.min_cluster_voxels = 2;

  const auto result = nav3d::map::MapBuilder::buildVoxelMap(config);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().raw_point_count, 5u);
  EXPECT_EQ(result.value().filtered_point_count, 2u);
  EXPECT_EQ(result.value().occupied_voxel_count, 2u);
  EXPECT_TRUE(result.value().map.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(result.value().map.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_FALSE(result.value().map.isOccupied({5.0, 0.0, 0.0}));
}

TEST(MapBuilder, BuildsFilteredVoxelMapFromAlreadyLoadedPointCloud)
{
  nav3d::map::MapBuildConfig config;
  config.preprocessor.resolution = 1.0;
  config.preprocessor.min_points_per_voxel = 2;
  config.preprocessor.min_cluster_voxels = 2;
  nav3d::map::PointCloud cloud;
  cloud.points = {
    {0.1, 0.1, 0.1},
    {0.2, 0.2, 0.2},
    {1.1, 0.1, 0.1},
    {1.2, 0.2, 0.2},
    {5.0, 0.0, 0.0},
  };

  const auto result = nav3d::map::MapBuilder::buildVoxelMapFromPointCloud(cloud, config);

  EXPECT_EQ(result.raw_point_count, 5u);
  EXPECT_EQ(result.filtered_point_count, 2u);
  EXPECT_EQ(result.occupied_voxel_count, 2u);
  EXPECT_TRUE(result.map.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(result.map.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_FALSE(result.map.isOccupied({5.0, 0.0, 0.0}));
}

TEST(MapBuilder, ReportsPcdLoadFailure)
{
  nav3d::map::MapBuildConfig config;
  config.pcd_path = "missing-builder-map.pcd";

  const auto result = nav3d::map::MapBuilder::buildVoxelMap(config);

  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().find("failed to open"), std::string::npos);
}
