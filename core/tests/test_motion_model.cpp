#include <gtest/gtest.h>

#include "nav3d/controller/motion_model.h"

TEST(MotionModel, DifferentialDriveRemovesLateralAndVerticalVelocity)
{
  nav3d::controller::MotionModelConfig config;
  config.type = nav3d::controller::MotionModelType::DifferentialDrive;
  config.max_linear_speed = 1.0;
  config.max_yaw_rate = 0.8;
  const nav3d::controller::MotionModel model(config);

  nav3d::controller::TrajectoryCommand desired;
  desired.linear = {0.8, 0.4, 0.3};
  desired.angular_z = 2.0;

  const auto constrained = model.constrain(desired, {}, 0.1);

  EXPECT_NEAR(constrained.linear.x, 0.8, 1e-9);
  EXPECT_NEAR(constrained.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(constrained.linear.z, 0.0, 1e-9);
  EXPECT_NEAR(constrained.angular_z, 0.8, 1e-9);
}

TEST(MotionModel, OmniBaseAllowsLateralButNotVerticalVelocity)
{
  nav3d::controller::MotionModelConfig config;
  config.type = nav3d::controller::MotionModelType::Omni;
  config.max_linear_speed = 1.0;
  config.max_yaw_rate = 1.0;
  const nav3d::controller::MotionModel model(config);

  nav3d::controller::TrajectoryCommand desired;
  desired.linear = {0.6, 0.8, 0.5};
  desired.angular_z = 0.5;

  const auto constrained = model.constrain(desired, {}, 0.1);

  EXPECT_NEAR(constrained.linear.x, 0.6, 1e-9);
  EXPECT_NEAR(constrained.linear.y, 0.8, 1e-9);
  EXPECT_NEAR(constrained.linear.z, 0.0, 1e-9);
  EXPECT_NEAR(constrained.angular_z, 0.5, 1e-9);
}

TEST(MotionModel, UavAllowsVerticalVelocityAndLimitsVectorMagnitude)
{
  nav3d::controller::MotionModelConfig config;
  config.type = nav3d::controller::MotionModelType::Uav;
  config.max_linear_speed = 1.0;
  config.max_yaw_rate = 2.0;
  const nav3d::controller::MotionModel model(config);

  nav3d::controller::TrajectoryCommand desired;
  desired.linear = {2.0, 0.0, 0.0};
  desired.angular_z = -3.0;

  const auto constrained = model.constrain(desired, {}, 0.1);

  EXPECT_NEAR(constrained.linear.x, 1.0, 1e-9);
  EXPECT_NEAR(constrained.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(constrained.linear.z, 0.0, 1e-9);
  EXPECT_NEAR(constrained.angular_z, -2.0, 1e-9);
}

TEST(MotionModel, AckermannRemovesLateralAndVerticalVelocityAndLimitsYawByTurningRadius)
{
  nav3d::controller::MotionModelConfig config;
  config.type = nav3d::controller::MotionModelType::Ackermann;
  config.max_linear_speed = 2.0;
  config.max_yaw_rate = 10.0;
  config.min_turning_radius = 2.0;
  const nav3d::controller::MotionModel model(config);

  nav3d::controller::TrajectoryCommand desired;
  desired.linear = {1.0, 0.5, 0.4};
  desired.angular_z = 2.0;

  const auto constrained = model.constrain(desired, {}, 0.1);

  EXPECT_NEAR(constrained.linear.x, 1.0, 1e-9);
  EXPECT_NEAR(constrained.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(constrained.linear.z, 0.0, 1e-9);
  EXPECT_NEAR(constrained.angular_z, 0.5, 1e-9);
}

TEST(MotionModel, AckermannLimitsReverseYawMagnitudeByTurningRadius)
{
  nav3d::controller::MotionModelConfig config;
  config.type = nav3d::controller::MotionModelType::Ackermann;
  config.max_linear_speed = 2.0;
  config.max_yaw_rate = 10.0;
  config.min_turning_radius = 1.0;
  const nav3d::controller::MotionModel model(config);

  nav3d::controller::TrajectoryCommand desired;
  desired.linear = {-1.0, 0.2, 0.3};
  desired.angular_z = 5.0;

  const auto constrained = model.constrain(desired, {}, 0.1);

  EXPECT_NEAR(constrained.linear.x, -1.0, 1e-9);
  EXPECT_NEAR(constrained.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(constrained.linear.z, 0.0, 1e-9);
  EXPECT_NEAR(constrained.angular_z, 1.0, 1e-9);
}

TEST(MotionModel, AppliesAccelerationLimitsFromPreviousCommand)
{
  nav3d::controller::MotionModelConfig config;
  config.type = nav3d::controller::MotionModelType::Omni;
  config.max_linear_speed = 10.0;
  config.max_linear_acceleration = 2.0;
  config.max_yaw_rate = 10.0;
  config.max_yaw_acceleration = 1.0;
  const nav3d::controller::MotionModel model(config);

  nav3d::controller::TrajectoryCommand previous;
  previous.linear = {0.0, 0.0, 0.0};
  previous.angular_z = 0.0;
  nav3d::controller::TrajectoryCommand desired;
  desired.linear = {1.0, 0.0, 0.0};
  desired.angular_z = 1.0;

  const auto constrained = model.constrain(desired, previous, 0.1);

  EXPECT_NEAR(constrained.linear.x, 0.2, 1e-9);
  EXPECT_NEAR(constrained.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(constrained.linear.z, 0.0, 1e-9);
  EXPECT_NEAR(constrained.angular_z, 0.1, 1e-9);
}

TEST(MotionModel, RejectsInvalidConfiguration)
{
  nav3d::controller::MotionModelConfig config;
  config.max_linear_speed = 0.0;

  EXPECT_THROW(nav3d::controller::MotionModel model(config), std::invalid_argument);
}

TEST(MotionModel, RejectsInvalidAckermannTurningRadius)
{
  nav3d::controller::MotionModelConfig config;
  config.type = nav3d::controller::MotionModelType::Ackermann;
  config.min_turning_radius = 0.0;

  EXPECT_THROW(nav3d::controller::MotionModel model(config), std::invalid_argument);
}
