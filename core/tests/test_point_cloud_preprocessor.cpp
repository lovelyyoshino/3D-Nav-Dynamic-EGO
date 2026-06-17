#include <gtest/gtest.h>

#include <initializer_list>
#include <stdexcept>

#include "nav3d/map/point_cloud_preprocessor.h"
#include "nav3d/map/voxel_grid_map.h"

namespace {

nav3d::map::PointCloud makeCloud(std::initializer_list<nav3d::common::Point3D> points)
{
  nav3d::map::PointCloud cloud;
  cloud.points.assign(points.begin(), points.end());
  return cloud;
}

nav3d::map::PointCloudPreprocessorConfig defaultConfig()
{
  nav3d::map::PointCloudPreprocessorConfig config;
  config.resolution = 1.0;
  config.min_points_per_voxel = 2;
  config.min_cluster_voxels = 1;
  return config;
}

}  // namespace

TEST(PointCloudPreprocessor, RemovesVoxelsBelowMinimumPointCount)
{
  const auto cloud = makeCloud({
    {0.1, 0.1, 0.1},
    {0.2, 0.2, 0.2},
    {2.1, 0.0, 0.0},
  });

  const auto cleaned = nav3d::map::PointCloudPreprocessor::filter(cloud, defaultConfig());
  const auto map = nav3d::map::VoxelGridMap::fromPointCloud(cleaned, 1.0);

  EXPECT_EQ(cleaned.points.size(), 1u);
  EXPECT_TRUE(map.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_FALSE(map.isOccupied({2.0, 0.0, 0.0}));
}

TEST(PointCloudPreprocessor, RemovesSmallIsolatedClusters)
{
  auto config = defaultConfig();
  config.min_cluster_voxels = 3;

  const auto cloud = makeCloud({
    {0.1, 0.1, 0.1}, {0.2, 0.2, 0.2},
    {1.1, 0.1, 0.1}, {1.2, 0.2, 0.2},
    {2.1, 0.1, 0.1}, {2.2, 0.2, 0.2},
    {10.1, 0.1, 0.1}, {10.2, 0.2, 0.2},
  });

  const auto cleaned = nav3d::map::PointCloudPreprocessor::filter(cloud, config);
  const auto map = nav3d::map::VoxelGridMap::fromPointCloud(cleaned, config.resolution);

  EXPECT_EQ(cleaned.points.size(), 3u);
  EXPECT_TRUE(map.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(map.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_TRUE(map.isOccupied({2.0, 0.0, 0.0}));
  EXPECT_FALSE(map.isOccupied({10.0, 0.0, 0.0}));
}

TEST(PointCloudPreprocessor, TreatsDiagonalVoxelsAsConnected)
{
  auto config = defaultConfig();
  config.min_cluster_voxels = 3;

  const auto cloud = makeCloud({
    {0.1, 0.1, 0.1}, {0.2, 0.2, 0.2},
    {1.1, 1.1, 1.1}, {1.2, 1.2, 1.2},
    {2.1, 2.1, 2.1}, {2.2, 2.2, 2.2},
  });

  const auto cleaned = nav3d::map::PointCloudPreprocessor::filter(cloud, config);
  const auto map = nav3d::map::VoxelGridMap::fromPointCloud(cleaned, config.resolution);

  EXPECT_EQ(cleaned.points.size(), 3u);
  EXPECT_TRUE(map.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(map.isOccupied({1.0, 1.0, 1.0}));
  EXPECT_TRUE(map.isOccupied({2.0, 2.0, 2.0}));
}

TEST(PointCloudPreprocessor, RejectsInvalidConfiguration)
{
  auto config = defaultConfig();
  config.resolution = 0.0;
  EXPECT_THROW(nav3d::map::PointCloudPreprocessor::filter({}, config), std::invalid_argument);

  config = defaultConfig();
  config.min_points_per_voxel = 0;
  EXPECT_THROW(nav3d::map::PointCloudPreprocessor::filter({}, config), std::invalid_argument);

  config = defaultConfig();
  config.min_cluster_voxels = 0;
  EXPECT_THROW(nav3d::map::PointCloudPreprocessor::filter({}, config), std::invalid_argument);
}
