#include <gtest/gtest.h>

#include <fstream>
#include <ios>
#include <string>

#include "nav3d/map/pcd_loader.h"

namespace {

std::string writeAsciiPcd()
{
  const std::string path = "test_ascii_map.pcd";
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

std::string writeBinaryPcd()
{
  const std::string path = "test_binary_map.pcd";
  std::ofstream out(path, std::ios::binary);
  out << "# .PCD v0.7\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z _\n"
      << "SIZE 4 4 4 1\n"
      << "TYPE F F F U\n"
      << "COUNT 1 1 1 4\n"
      << "WIDTH 2\n"
      << "HEIGHT 1\n"
      << "POINTS 2\n"
      << "DATA binary\n";

  const auto write_point = [&](float x, float y, float z) {
    const unsigned char padding[4] = {0, 0, 0, 0};
    out.write(reinterpret_cast<const char*>(&x), sizeof(float));
    out.write(reinterpret_cast<const char*>(&y), sizeof(float));
    out.write(reinterpret_cast<const char*>(&z), sizeof(float));
    out.write(reinterpret_cast<const char*>(padding), sizeof(padding));
  };
  write_point(1.25F, -2.5F, 0.75F);
  write_point(3.5F, 4.25F, -1.0F);
  return path;
}

}  // namespace

TEST(PcdLoader, LoadsAsciiXyzAndIgnoresExtraFields)
{
  const auto result = nav3d::map::PcdLoader::load(writeAsciiPcd());

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().points.size(), 3u);
  EXPECT_DOUBLE_EQ(result.value().points[1].x, 1.0);
  EXPECT_DOUBLE_EQ(result.value().points[1].y, 2.0);
  EXPECT_DOUBLE_EQ(result.value().points[1].z, 3.0);
  EXPECT_DOUBLE_EQ(result.value().points[2].x, -1.5);
  EXPECT_DOUBLE_EQ(result.value().points[2].y, 0.5);
  EXPECT_DOUBLE_EQ(result.value().points[2].z, 2.25);
}

TEST(PcdLoader, ReportsMissingFile)
{
  const auto result = nav3d::map::PcdLoader::load("missing-file-does-not-exist.pcd");

  ASSERT_FALSE(result.ok());
  EXPECT_NE(result.error().find("failed to open"), std::string::npos);
}

TEST(PcdLoader, LoadsBinaryXyzWithReferenceLikePaddingField)
{
  const auto result = nav3d::map::PcdLoader::load(writeBinaryPcd());

  ASSERT_TRUE(result.ok()) << result.error();
  ASSERT_EQ(result.value().points.size(), 2u);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().points[0].x), 1.25F);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().points[0].y), -2.5F);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().points[0].z), 0.75F);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().points[1].x), 3.5F);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().points[1].y), 4.25F);
  EXPECT_FLOAT_EQ(static_cast<float>(result.value().points[1].z), -1.0F);
}
