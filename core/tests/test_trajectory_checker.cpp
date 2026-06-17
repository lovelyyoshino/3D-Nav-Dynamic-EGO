#include <gtest/gtest.h>

#include "nav3d/collision/trajectory_checker.h"
#include "nav3d/map/i_map.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/bspline/uniform_bspline.h"

namespace {

class NonTraversableGapMap final : public nav3d::map::IMap {
public:
  bool isOccupied(const nav3d::common::Point3D&) const override { return false; }

  bool isFree(const nav3d::common::Point3D& p) const override
  {
    return isInBounds(p) && !(p.x > 0.9 && p.x < 1.1);
  }

  double getDistance(const nav3d::common::Point3D&) const override { return 0.0; }

  bool isInBounds(const nav3d::common::Point3D& p) const override
  {
    return p.x >= 0.0 && p.x <= 2.0 && p.y == 0.0 && p.z == 0.0;
  }

  double getResolution() const override { return 0.1; }

  nav3d::common::BoundingBox getBounds() const override
  {
    return {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, true};
  }
};

}  // namespace

TEST(TrajectoryChecker, FindsCollisionOnSampledBspline)
{
  nav3d::map::VoxelGridMap map(0.5);
  map.insertOccupied({1.0, 0.0, 0.0});
  const auto spline = nav3d::planner::UniformBspline::fitThroughWaypoints(
    {{0, 0, 0}, {0.5, 0, 0}, {1.0, 0, 0}, {1.5, 0, 0}}, 0.1);

  nav3d::collision::TrajectoryChecker checker(0.05);

  const auto result = checker.check(map, spline);

  EXPECT_TRUE(result.in_collision);
  EXPECT_TRUE(result.first_collision_time.has_value());
}

TEST(TrajectoryChecker, RejectsTrajectoryThroughNonFreeGapEvenWhenNotOccupied)
{
  NonTraversableGapMap map;
  const auto spline = nav3d::planner::UniformBspline::fitThroughWaypoints(
    {{0, 0, 0}, {0.5, 0, 0}, {1.0, 0, 0}, {1.5, 0, 0}, {2.0, 0, 0}}, 0.1);

  nav3d::collision::TrajectoryChecker checker(0.02);

  const auto result = checker.check(map, spline);

  EXPECT_TRUE(result.in_collision);
  ASSERT_TRUE(result.first_collision_point.has_value());
  EXPECT_NEAR(result.first_collision_point->x, 1.0, 0.15);
}

TEST(TrajectoryChecker, RejectsCollisionBetweenSparseTimeSamples)
{
  class ThinMiddleWallMap final : public nav3d::map::IMap {
  public:
    bool isOccupied(const nav3d::common::Point3D& p) const override
    {
      return p.x > 0.45 && p.x < 0.55;
    }

    bool isFree(const nav3d::common::Point3D& p) const override
    {
      return isInBounds(p) && !isOccupied(p);
    }

    double getDistance(const nav3d::common::Point3D&) const override { return 0.0; }

    bool isInBounds(const nav3d::common::Point3D& p) const override
    {
      return p.x >= 0.0 && p.x <= 1.0 && p.y == 0.0 && p.z == 0.0;
    }

    double getResolution() const override { return 0.1; }

    nav3d::common::BoundingBox getBounds() const override
    {
      return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, true};
    }
  };

  ThinMiddleWallMap map;
  const nav3d::planner::UniformBspline spline(
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
    1,
    1.0);

  nav3d::collision::TrajectoryChecker checker(1.0);

  const auto result = checker.check(map, spline);

  EXPECT_TRUE(result.in_collision);
  ASSERT_TRUE(result.first_collision_point.has_value());
  EXPECT_NEAR(result.first_collision_point->x, 0.5, 0.1);
}
