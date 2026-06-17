#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "pcl_pcd_loader.h"

namespace {

std::string writePclAsciiPcd()
{
  const std::string path = "test_pcl_ascii_map.pcd";
  std::ofstream out(path);
  out << "# .PCD v0.7\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\n"
      << "TYPE F F F F\n"
      << "COUNT 1 1 1 1\n"
      << "WIDTH 3\n"
      << "HEIGHT 1\n"
      << "POINTS 3\n"
      << "DATA ascii\n"
      << "0 0 0 1\n"
      << "1.0 2.0 3.0 9\n"
      << "-1.5 0.5 2.25 4\n";
  return path;
}

}  // namespace

TEST(PclPcdLoader, LoadsAsciiXyzThroughPcl)
{
  const auto result = nav3d::tools::PclPcdLoader::load(writePclAsciiPcd());

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().points.size(), 3u);
  EXPECT_DOUBLE_EQ(result.value().points[1].x, 1.0);
  EXPECT_DOUBLE_EQ(result.value().points[1].y, 2.0);
  EXPECT_DOUBLE_EQ(result.value().points[1].z, 3.0);
  EXPECT_DOUBLE_EQ(result.value().points[2].x, -1.5);
  EXPECT_DOUBLE_EQ(result.value().points[2].y, 0.5);
  EXPECT_DOUBLE_EQ(result.value().points[2].z, 2.25);
}

TEST(PclPcdLoader, ReportsMissingFile)
{
  const auto result = nav3d::tools::PclPcdLoader::load("missing-pcl-loader-map.pcd");

  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().find("failed to load PCD with PCL"), std::string::npos);
}
