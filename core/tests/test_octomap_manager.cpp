#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "nav3d/map/octomap_manager.h"

namespace {

std::string writeOctomapAsciiPcd()
{
  const std::string path = "test_octomap_manager_map.pcd";
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

std::string writeOctomapRayAsciiPcd()
{
  const std::string path = "test_octomap_manager_ray_map.pcd";
  std::ofstream out(path);
  out << "# .PCD v0.7\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH 1\n"
      << "HEIGHT 1\n"
      << "POINTS 1\n"
      << "DATA ascii\n"
      << "3.1 0.0 0.0\n";
  return path;
}

}  // namespace

TEST(OctomapManager, BuildsFilteredOctomapFromPcd)
{
  nav3d::map::MapBuildConfig config;
  config.pcd_path = writeOctomapAsciiPcd();
  config.preprocessor.resolution = 1.0;
  config.preprocessor.min_points_per_voxel = 2;
  config.preprocessor.min_cluster_voxels = 2;

  const auto result = nav3d::map::OctomapManager::buildFromPcd(config);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().raw_point_count, 5u);
  EXPECT_EQ(result.value().filtered_point_count, 2u);
  EXPECT_EQ(result.value().occupied_leaf_count, 2u);
  EXPECT_TRUE(result.value().map.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(result.value().map.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_FALSE(result.value().map.isOccupied({5.0, 0.0, 0.0}));
  EXPECT_TRUE(result.value().map.isFree({5.0, 0.0, 0.0}));
  EXPECT_FALSE(result.value().map.isFree({100.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(result.value().map.getResolution(), 1.0);
  EXPECT_TRUE(result.value().map.getBounds().valid);
  EXPECT_LE(result.value().map.getBounds().min.x, -0.9);
  EXPECT_GE(result.value().map.getBounds().max.x, 6.0);
}

TEST(OctomapManager, BuildsFilteredOctomapFromAlreadyLoadedPointCloud)
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

  const auto result = nav3d::map::OctomapManager::buildFromPointCloud(cloud, config);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().raw_point_count, 5u);
  EXPECT_EQ(result.value().filtered_point_count, 2u);
  EXPECT_EQ(result.value().occupied_leaf_count, 2u);
  EXPECT_TRUE(result.value().map.isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(result.value().map.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_FALSE(result.value().map.isOccupied({5.0, 0.0, 0.0}));
  EXPECT_TRUE(result.value().map.isFree({5.0, 0.0, 0.0}));
}

TEST(OctomapManager, BuildsKnownFreeSpaceFromMultiplePointCloudFrames)
{
  nav3d::map::MapBuildConfig config;
  config.preprocessor.resolution = 1.0;
  config.preprocessor.min_points_per_voxel = 1;
  config.preprocessor.min_cluster_voxels = 1;
  config.insert_free_space_rays = true;

  std::vector<nav3d::map::PointCloudFrame> frames;
  frames.push_back({
    nav3d::map::PointCloud{{{3.1, 0.0, 0.0}}},
    nav3d::common::Point3D{0.0, 0.0, 0.0},
  });
  frames.push_back({
    nav3d::map::PointCloud{{{0.0, 3.1, 0.0}}},
    nav3d::common::Point3D{0.0, 0.0, 0.0},
  });

  const auto result = nav3d::map::OctomapManager::buildFromPointCloudFrames(frames, config);

  ASSERT_TRUE(result.ok()) << result.error();
  EXPECT_EQ(result.value().raw_point_count, 2u);
  EXPECT_EQ(result.value().filtered_point_count, 2u);
  EXPECT_TRUE(result.value().map.isOccupied({3.0, 0.0, 0.0}));
  EXPECT_TRUE(result.value().map.isOccupied({0.0, 3.0, 0.0}));

  const auto* x_ray_free = result.value().map.tree().search(1.0, 0.0, 0.0);
  ASSERT_NE(x_ray_free, nullptr);
  EXPECT_FALSE(result.value().map.tree().isNodeOccupied(x_ray_free));

  const auto* y_ray_free = result.value().map.tree().search(0.0, 1.0, 0.0);
  ASSERT_NE(y_ray_free, nullptr);
  EXPECT_FALSE(result.value().map.tree().isNodeOccupied(y_ray_free));
}

TEST(OctomapManager, RejectsPointCloudFramesWhenRayInsertionIsDisabled)
{
  nav3d::map::MapBuildConfig config;
  config.preprocessor.resolution = 1.0;
  std::vector<nav3d::map::PointCloudFrame> frames;
  frames.push_back({
    nav3d::map::PointCloud{{{1.0, 0.0, 0.0}}},
    nav3d::common::Point3D{0.0, 0.0, 0.0},
  });

  const auto result = nav3d::map::OctomapManager::buildFromPointCloudFrames(frames, config);

  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().find("insert_free_space_rays"), std::string::npos);
}

TEST(OctomapManager, SavesAndLoadsBinaryTree)
{
  nav3d::map::MapBuildConfig config;
  config.pcd_path = writeOctomapAsciiPcd();
  config.preprocessor.resolution = 1.0;
  config.preprocessor.min_points_per_voxel = 2;
  config.preprocessor.min_cluster_voxels = 2;

  const auto result = nav3d::map::OctomapManager::buildFromPcd(config);
  ASSERT_TRUE(result.ok()) << result.error();

  const std::string path = "test_octomap_manager_map.bt";
  ASSERT_TRUE(result.value().map.saveBinary(path).ok());

  const auto loaded = nav3d::map::OctomapManager::loadBinary(path);

  ASSERT_TRUE(loaded.ok()) << loaded.error();
  EXPECT_EQ(loaded.value().occupiedLeafCount(), 2u);
  EXPECT_TRUE(loaded.value().isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(loaded.value().isOccupied({1.0, 0.0, 0.0}));
  EXPECT_FALSE(loaded.value().isOccupied({5.0, 0.0, 0.0}));
  EXPECT_TRUE(loaded.value().getBounds().valid);
  EXPECT_DOUBLE_EQ(loaded.value().getResolution(), 1.0);
}

TEST(OctomapManager, SerializesAndLoadsBinaryPayloadInMemory)
{
  nav3d::map::OctomapManager map(0.5);
  map.insertOccupied({0.0, 0.0, 0.0});
  map.insertOccupied({1.0, 0.0, 0.0});

  const auto payload = map.serializeBinary();

  ASSERT_TRUE(payload.ok()) << payload.error();
  ASSERT_FALSE(payload.value().empty());

  const auto loaded = nav3d::map::OctomapManager::deserializeBinary(payload.value());

  ASSERT_TRUE(loaded.ok()) << loaded.error();
  EXPECT_DOUBLE_EQ(loaded.value().getResolution(), 0.5);
  EXPECT_EQ(loaded.value().occupiedLeafCount(), 2u);
  EXPECT_TRUE(loaded.value().isOccupied({0.0, 0.0, 0.0}));
  EXPECT_TRUE(loaded.value().isOccupied({1.0, 0.0, 0.0}));
  EXPECT_TRUE(loaded.value().getBounds().valid);
}

TEST(OctomapManager, RejectsInvalidBinaryPayload)
{
  const std::vector<std::uint8_t> payload = {'n', 'o', 't', '-', 'b', 't'};

  const auto loaded = nav3d::map::OctomapManager::deserializeBinary(payload);

  ASSERT_FALSE(loaded.ok());
  EXPECT_NE(loaded.error().find("failed to deserialize"), std::string::npos);
}

TEST(OctomapManager, ExposesReadOnlyOcTreeForBridgeSerialization)
{
  nav3d::map::OctomapManager map(0.5);
  map.insertOccupied({1.0, 0.0, 0.0});

  const octomap::OcTree& tree = map.tree();

  EXPECT_DOUBLE_EQ(tree.getResolution(), 0.5);
  const auto* node = tree.search(1.0, 0.0, 0.0);
  ASSERT_NE(node, nullptr);
  EXPECT_TRUE(tree.isNodeOccupied(node));
}

TEST(OctomapManager, BuildsKnownFreeSpaceFromPcdWhenSensorOriginIsConfigured)
{
  nav3d::map::MapBuildConfig config;
  config.pcd_path = writeOctomapRayAsciiPcd();
  config.preprocessor.resolution = 1.0;
  config.preprocessor.min_points_per_voxel = 1;
  config.preprocessor.min_cluster_voxels = 1;
  config.insert_free_space_rays = true;
  config.sensor_origin = nav3d::common::Point3D{0.0, 0.0, 0.0};

  const auto result = nav3d::map::OctomapManager::buildFromPcd(config);

  ASSERT_TRUE(result.ok()) << result.error();
  const auto* known_free = result.value().map.tree().search(1.0, 0.0, 0.0);
  ASSERT_NE(known_free, nullptr);
  EXPECT_FALSE(result.value().map.tree().isNodeOccupied(known_free));
  EXPECT_TRUE(result.value().map.isOccupied({3.0, 0.0, 0.0}));
}

TEST(OctomapManager, RejectsPcdRayInsertionWithoutSensorOrigin)
{
  nav3d::map::MapBuildConfig config;
  config.pcd_path = writeOctomapRayAsciiPcd();
  config.insert_free_space_rays = true;

  const auto result = nav3d::map::OctomapManager::buildFromPcd(config);

  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().find("sensor_origin"), std::string::npos);
}

TEST(OctomapManager, ReportsBinaryLoadFailures)
{
  const auto missing = nav3d::map::OctomapManager::loadBinary("missing-octomap-manager-map.bt");

  ASSERT_FALSE(missing.ok());
  EXPECT_NE(missing.error().find("failed to load"), std::string::npos);

  const std::string corrupt_path = "test_octomap_manager_corrupt.bt";
  {
    std::ofstream out(corrupt_path);
    out << "not an octomap";
  }

  const auto corrupt = nav3d::map::OctomapManager::loadBinary(corrupt_path);

  ASSERT_FALSE(corrupt.ok());
  EXPECT_NE(corrupt.error().find("failed to load"), std::string::npos);
}

TEST(OctomapManager, ReportsBinarySaveFailure)
{
  nav3d::map::OctomapManager map(1.0);
  map.insertOccupied({0.0, 0.0, 0.0});

  const auto saved = map.saveBinary("missing-directory/test_octomap_manager_map.bt");

  ASSERT_FALSE(saved.ok());
  EXPECT_NE(saved.error().find("failed to save"), std::string::npos);
}

TEST(OctomapManager, RayUpdateMarksFreeCellsAndOccupiedEndpoint)
{
  nav3d::map::OctomapManager map(1.0);
  map.setExplicitBounds({{0.0, 0.0, 0.0}, {4.0, 1.0, 1.0}, true});

  map.markRayFreeAndOccupied({0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

  EXPECT_TRUE(map.isFree({0.0, 0.0, 0.0}));
  EXPECT_TRUE(map.isFree({1.0, 0.0, 0.0}));
  EXPECT_TRUE(map.isFree({2.0, 0.0, 0.0}));
  EXPECT_TRUE(map.isOccupied({3.0, 0.0, 0.0}));
  EXPECT_FALSE(map.isFree({3.0, 0.0, 0.0}));
}

TEST(OctomapManager, RayUpdateClearsStaleOccupiedVoxelWithRepeatedMisses)
{
  nav3d::map::OctomapManager map(1.0);
  map.setExplicitBounds({{0.0, 0.0, 0.0}, {5.0, 1.0, 1.0}, true});
  map.insertOccupied({1.0, 0.0, 0.0});

  ASSERT_TRUE(map.isOccupied({1.0, 0.0, 0.0}));
  for (int i = 0; i < 3; ++i) {
    map.markRayFreeAndOccupied({0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
  }

  EXPECT_TRUE(map.isFree({1.0, 0.0, 0.0}));
  EXPECT_FALSE(map.isOccupied({1.0, 0.0, 0.0}));
  EXPECT_TRUE(map.isOccupied({3.0, 0.0, 0.0}));
  EXPECT_EQ(map.occupiedLeafCount(), 1u);
}

TEST(OctomapManager, ReportsPcdLoadFailure)
{
  nav3d::map::MapBuildConfig config;
  config.pcd_path = "missing-octomap-manager-map.pcd";

  const auto result = nav3d::map::OctomapManager::buildFromPcd(config);

  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().find("failed to open"), std::string::npos);
}
