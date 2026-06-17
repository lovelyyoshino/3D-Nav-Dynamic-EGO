#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


class CmdVelPoseSim(Node):
    def __init__(self) -> None:
        super().__init__("nav3d_cmd_vel_pose_sim")
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.command_frame = self.declare_parameter("command_frame", "map").value
        self.x = float(self.declare_parameter("start_x", 0.0).value)
        self.y = float(self.declare_parameter("start_y", 0.0).value)
        self.z = float(self.declare_parameter("start_z", 0.0).value)
        self.yaw = float(self.declare_parameter("start_yaw", 0.0).value)
        self.update_rate = float(self.declare_parameter("update_rate", 20.0).value)
        self.max_step = float(self.declare_parameter("max_step", 0.2).value)
        self.cmd_timeout = float(self.declare_parameter("cmd_timeout", 0.5).value)
        if self.update_rate <= 0.0:
            raise ValueError("update_rate must be positive")
        if self.max_step <= 0.0:
            raise ValueError("max_step must be positive")
        if self.cmd_timeout < 0.0:
            raise ValueError("cmd_timeout must be non-negative")

        self.cmd: Optional[Twist] = None
        self.last_cmd_time = self.get_clock().now()
        self.pose_pub = self.create_publisher(PoseStamped, "/nav3d/current_pose", 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.timer = self.create_timer(1.0 / self.update_rate, self.on_timer)

    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self) -> None:
        dt = 1.0 / self.update_rate
        now = self.get_clock().now()
        cmd = self.cmd
        if cmd is not None and self.cmd_timeout > 0.0:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                cmd = None

        if cmd is not None:
            vx = float(cmd.linear.x)
            vy = float(cmd.linear.y)
            vz = float(cmd.linear.z)
            wz = float(cmd.angular.z)
            step_norm = math.sqrt(vx * vx + vy * vy + vz * vz) * dt
            if step_norm > self.max_step:
                scale = self.max_step / step_norm
                vx *= scale
                vy *= scale
                vz *= scale
            if self.command_frame == "body":
                cos_yaw = math.cos(self.yaw)
                sin_yaw = math.sin(self.yaw)
                map_vx = vx * cos_yaw - vy * sin_yaw
                map_vy = vx * sin_yaw + vy * cos_yaw
            else:
                map_vx = vx
                map_vy = vy
            self.x += map_vx * dt
            self.y += map_vy * dt
            self.z += vz * dt
            self.yaw += wz * dt

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        quat = yaw_to_quaternion(self.yaw)
        pose.pose.orientation.x = quat["x"]
        pose.pose.orientation.y = quat["y"]
        pose.pose.orientation.z = quat["z"]
        pose.pose.orientation.w = quat["w"]
        self.pose_pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = CmdVelPoseSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
