#!/usr/bin/env python3
import argparse
import math
import os
import random
import signal
import struct
import sys
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

# Make the sibling lib importable when run via `ros2 run` (which doesn't add
# the script's directory to sys.path).
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from nav3d_local_replan_smoke_lib import (
    advance_along_path_from_current,
    drop_passed_anchors,
    filter_anchors_in_radius,
    initial_travel_anchor_spawn_at,
    latest_snap_z_failure_status,
    nearest_path_arc_length,
    path_endpoint_reaches_goal,
    remaining_distance_along_path,
    sample_anchors_along_path,
    should_retry_snap_z,
    should_publish_anchor_cloud,
    visible_anchors_signature,
    dense_world_anchored_cloud_points,
    empty_trajectory_follow_action,
)


Point = Tuple[float, float, float]


# ---------------------------------------------------------------------------
# Watchdog / lifecycle helpers
#
# The original smoke could "run for a while and then hang" because every
# wait_for_status / wait_for loop blocked indefinitely with no SIGINT escape
# hatch and no heartbeat. The helpers below give us:
#   1. A single source of truth for "should we keep going?" — `_aborted`,
#      flipped by SIGINT/SIGTERM and by hard timeouts;
#   2. Heartbeat printing every `--heartbeat-interval` so an operator watching
#      stdout can tell if a phase is making progress or genuinely stuck;
#   3. retry_call(...) — wraps a publish/spin call in N attempts with a
#      short cooldown, used for local-cloud publishes that occasionally race
#      with bridge readiness;
#   4. bridge-watchdog: every spin cycle we re-check publishers/subscribers
#      so a bridge crash fails fast instead of timing out 4 minutes later.
# ---------------------------------------------------------------------------

_aborted = False


def _request_abort(reason: str) -> None:
    global _aborted
    if not _aborted:
        print(f"nav3d_local_replan_smoke abort_requested reason={reason}", file=sys.stderr)
    _aborted = True


def _signal_handler(signum, _frame) -> None:  # pragma: no cover — runtime only
    _request_abort(f"signal_{signum}")


def _install_signal_handlers() -> None:
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)


def retry_call(
    fn: Callable[[], None],
    *,
    attempts: int = 3,
    cooldown: float = 0.2,
    label: str = "publish",
) -> None:
    last_error: Optional[BaseException] = None
    for attempt in range(1, attempts + 1):
        try:
            fn()
            return
        except Exception as error:  # noqa: BLE001 — we genuinely want to swallow + retry
            last_error = error
            print(
                f"nav3d_local_replan_smoke retry label={label} attempt={attempt}/{attempts} error={error}",
                file=sys.stderr,
            )
            time.sleep(cooldown)
    raise RuntimeError(f"{label} failed after {attempts} attempts: {last_error}")


def parse_point(values: Sequence[str], name: str) -> Point:
    if len(values) != 3:
        raise argparse.ArgumentTypeError(f"{name} requires exactly three values: x y z")
    try:
        return (float(values[0]), float(values[1]), float(values[2]))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{name} values must be numeric") from error


def distance(left: Point, right: Point) -> float:
    return math.sqrt(
        (left[0] - right[0]) ** 2 +
        (left[1] - right[1]) ** 2 +
        (left[2] - right[2]) ** 2
    )


def max_trajectory_delta(left: Sequence[Point], right: Sequence[Point]) -> float:
    count = min(len(left), len(right))
    if count == 0:
        return 0.0
    return max(distance(left[index], right[index]) for index in range(count))


def make_pose(frame_id: str, point: Point, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]
    pose.pose.position.z = point[2]
    pose.pose.orientation.w = 1.0
    return pose


def make_pointcloud(frame_id: str, points: Sequence[Point], stamp) -> PointCloud2:
    data = bytearray()
    for point in points:
        data.extend(struct.pack("<fff", point[0], point[1], point[2]))

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = len(data)
    msg.data = list(data)
    msg.is_dense = True
    return msg


def make_obstacle_cluster(center: Point, radius: float) -> List[Point]:
    offsets = [
        (0.0, 0.0, 0.0),
        (radius, 0.0, 0.0),
        (-radius, 0.0, 0.0),
        (0.0, radius, 0.0),
        (0.0, -radius, 0.0),
        (0.0, 0.0, radius),
        (0.0, 0.0, -radius),
    ]
    return [
        (center[0] + dx, center[1] + dy, center[2] + dz)
        for dx, dy, dz in offsets
    ]


@dataclass
class SmokeObservation:
    statuses: List[str]
    trajectories: List[List[Point]]
    last_trajectory: List[Point]
    local_marker_voxels: int = 0
    max_local_marker_voxels: int = 0
    marker_messages: int = 0
    trajectory_messages: int = 0


class LocalReplanSmoke(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("nav3d_local_replan_smoke")
        self.args = args
        self.observation = SmokeObservation(statuses=[], trajectories=[], last_trajectory=[])
        latched_qos = QoSProfile(depth=10)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.start_pub = self.create_publisher(PoseStamped, "/nav3d/start", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/nav3d/goal", 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, "/nav3d/current_pose", 10)
        self.local_cloud_pub = self.create_publisher(PointCloud2, "/nav3d/local_pointcloud", 1)
        self.create_subscription(String, "/nav3d/status", self.on_status, 50)
        self.create_subscription(Path, "/nav3d/trajectory", self.on_trajectory, latched_qos)
        # Marker subscription is optional and very expensive when bridge is in
        # a dense map (e.g. building2_9 = 62k voxels per MarkerArray) — every
        # callback deserialises the full payload, throttling smoke main-loop
        # to ~5Hz on busy bridges. Skip it unless the operator explicitly
        # requests marker tracking via --track-markers.
        if getattr(args, 'track_markers', False):
            self.create_subscription(
                MarkerArray,
                "/nav3d/planning_occupied_markers",
                self.on_markers,
                latched_qos,
            )

    def on_status(self, msg: String) -> None:
        self.observation.statuses.append(msg.data)

    def on_trajectory(self, msg: Path) -> None:
        points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            for pose in msg.poses
        ]
        self.observation.last_trajectory = points
        self.observation.trajectory_messages += 1
        if points:
            self.observation.trajectories.append(points)

    def on_markers(self, msg: MarkerArray) -> None:
        local_count = 0
        for marker in msg.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            if marker.type == Marker.CUBE_LIST and "local" in marker.ns.lower():
                local_count += len(marker.points)
        self.observation.local_marker_voxels = local_count
        self.observation.max_local_marker_voxels = max(
            self.observation.max_local_marker_voxels,
            local_count,
        )
        self.observation.marker_messages += 1

    def publish_pose_pair(self, start: Point, goal: Point) -> None:
        stamp = self.get_clock().now().to_msg()
        retry_call(
            lambda: self.start_pub.publish(make_pose(self.args.frame_id, start, stamp)),
            label="start_pub",
        )
        time.sleep(self.args.publish_delay)
        retry_call(
            lambda: self.goal_pub.publish(make_pose(self.args.frame_id, goal, stamp)),
            label="goal_pub",
        )

    def publish_current_pose(self, current: Point) -> None:
        retry_call(
            lambda: self.current_pose_pub.publish(
                make_pose(self.args.frame_id, current, self.get_clock().now().to_msg())
            ),
            label="current_pose_pub",
        )

    def publish_local_cloud(self, points: Sequence[Point]) -> None:
        retry_call(
            lambda: self.local_cloud_pub.publish(
                make_pointcloud(self.args.frame_id, points, self.get_clock().now().to_msg())
            ),
            label="local_cloud_pub",
        )

    def publish_local_obstacles(self, current: Point, obstacle_points: Sequence[Point]) -> None:
        for _ in range(3):
            self.publish_current_pose(current)
            time.sleep(self.args.publish_delay)
        self.publish_local_cloud(obstacle_points)
        time.sleep(self.args.publish_delay)
        self.publish_current_pose(current)

    def clear_local_obstacles(self, current: Point) -> None:
        self.publish_current_pose(current)
        time.sleep(self.args.publish_delay)
        self.publish_local_cloud([])
        time.sleep(self.args.publish_delay)
        self.publish_current_pose(current)


def wait_for(
    node: LocalReplanSmoke,
    predicate: Callable[[], bool],
    timeout_sec: float,
    label: str,
) -> None:
    deadline = time.monotonic() + timeout_sec
    next_heartbeat = time.monotonic() + node.args.heartbeat_interval
    started = time.monotonic()
    while time.monotonic() < deadline:
        if _aborted:
            raise RuntimeError(f"aborted while waiting: {label}")
        if (
            node.args.bridge_watchdog
            and node.count_publishers("/nav3d/trajectory") == 0
        ):
            raise RuntimeError(
                f"bridge_disappeared (no publisher on /nav3d/trajectory) while waiting: {label}"
            )
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
        now = time.monotonic()
        if now >= next_heartbeat:
            elapsed = now - started
            last_status = node.observation.statuses[-1] if node.observation.statuses else "<none>"
            print(
                f"nav3d_local_replan_smoke heartbeat phase={label} elapsed={elapsed:.1f}s "
                f"trajectories={len(node.observation.trajectories)} "
                f"local_voxels={node.observation.local_marker_voxels} "
                f"last_status={last_status}",
                flush=True,
            )
            next_heartbeat = now + node.args.heartbeat_interval
    raise TimeoutError(label)


def first_status_since(
    node: LocalReplanSmoke,
    offset: int,
    tokens: Sequence[str],
) -> Optional[str]:
    for status in node.observation.statuses[offset:]:
        if any(token in status for token in tokens):
            return status
    return None


def wait_for_status(
    node: LocalReplanSmoke,
    offset: int,
    tokens: Sequence[str],
    timeout_sec: float,
    label: str,
) -> str:
    deadline = time.monotonic() + timeout_sec
    next_heartbeat = time.monotonic() + node.args.heartbeat_interval
    started = time.monotonic()
    while time.monotonic() < deadline:
        if _aborted:
            raise RuntimeError(f"aborted while waiting: {label}")
        if (
            node.args.bridge_watchdog
            and node.count_publishers("/nav3d/trajectory") == 0
        ):
            raise RuntimeError(
                f"bridge_disappeared (no publisher on /nav3d/trajectory) while waiting: {label}"
            )
        rclpy.spin_once(node, timeout_sec=0.05)
        status = first_status_since(node, offset, tokens)
        if status is not None:
            return status
        now = time.monotonic()
        if now >= next_heartbeat:
            elapsed = now - started
            last_status = node.observation.statuses[-1] if node.observation.statuses else "<none>"
            print(
                f"nav3d_local_replan_smoke heartbeat phase={label} elapsed={elapsed:.1f}s "
                f"awaiting_tokens={list(tokens)} last_status={last_status}",
                flush=True,
            )
            next_heartbeat = now + node.args.heartbeat_interval
    raise TimeoutError(label)


def wait_for_initial_plan(
    node: LocalReplanSmoke,
    status_offset: int,
    trajectory_offset: int,
    min_trajectory_poses: int,
    timeout_sec: float,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        has_plan_status = any(
            status.startswith("plan_success ") or status.startswith("clicked_plan_success ")
            for status in node.observation.statuses[status_offset:]
        )
        has_trajectory = (
            len(node.observation.trajectories) > trajectory_offset and
            len(node.observation.trajectories[-1]) >= min_trajectory_poses
        )
        if has_plan_status and has_trajectory:
            return
    raise TimeoutError("timed out waiting for initial plan_success trajectory")


def restore_initial_plan(
    node: LocalReplanSmoke,
    start: Point,
    goal: Point,
    current: Point,
    min_trajectory_poses: int,
    timeout_sec: float,
) -> None:
    node.clear_local_obstacles(current)
    status_offset = len(node.observation.statuses)
    trajectory_offset = len(node.observation.trajectories)
    node.publish_pose_pair(start, goal)
    wait_for_initial_plan(
        node,
        status_offset,
        trajectory_offset,
        min_trajectory_poses,
        timeout_sec,
    )


def recover_follow_plan(
    node: LocalReplanSmoke,
    args: argparse.Namespace,
    current: Point,
    goal: Point,
    reason: str,
) -> None:
    try:
        node.clear_local_obstacles(current)
    except RuntimeError:
        pass
    status_offset = len(node.observation.statuses)
    trajectory_offset = len(node.observation.trajectories)
    node.publish_pose_pair(current, goal)
    wait_for_initial_plan(
        node,
        status_offset,
        trajectory_offset,
        args.min_trajectory_poses,
        args.timeout,
    )
    print(
        "nav3d_local_replan_smoke recovery_replan_success "
        f"reason={reason} poses={len(node.observation.trajectories[-1])} "
        f"current=({current[0]:.2f},{current[1]:.2f},{current[2]:.2f})"
    )


def clamp_index(index: int, size: int) -> int:
    return min(max(0, index), max(0, size - 1))


def index_at_distance(
    trajectory: Sequence[Point],
    start_index: int,
    fallback_index: int,
    lookahead_distance: float,
) -> int:
    if lookahead_distance <= 0.0 or start_index >= len(trajectory) - 1:
        return clamp_index(fallback_index, len(trajectory))

    total = 0.0
    previous = trajectory[start_index]
    for index in range(start_index + 1, len(trajectory)):
        total += distance(previous, trajectory[index])
        if total >= lookahead_distance:
            return index
        previous = trajectory[index]
    return len(trajectory) - 1


def make_attempt_obstacles(
    trajectory: Sequence[Point],
    current_index: int,
    obstacle_index: int,
    args: argparse.Namespace,
    rng: random.Random,
) -> Tuple[List[Point], List[Point]]:
    cluster_count = max(1, args.random_obstacles)
    centers = [trajectory[obstacle_index]]
    if args.random_obstacles > 0:
        min_offset = max(1, args.random_min_index_offset)
        max_offset = max(min_offset, args.random_max_index_offset)
        max_offset = min(max_offset, max(1, len(trajectory) - current_index - 1))
        for _ in range(cluster_count - 1):
            index = clamp_index(current_index + rng.randint(min_offset, max_offset), len(trajectory))
            base = trajectory[index]
            jitter = args.random_jitter
            centers.append((
                base[0] + rng.uniform(-jitter, jitter),
                base[1] + rng.uniform(-jitter, jitter),
                base[2] + rng.uniform(-jitter * 0.5, jitter * 0.5),
            ))

    points: List[Point] = []
    for center in centers:
        points.extend(make_obstacle_cluster(center, args.obstacle_radius))
    return points, centers


# ---------------------------------------------------------------------------
# World-anchored sticky obstacles (D5)
#
# Operator mental model: every `spawn_interval` seconds the simulator drops a
# fixed cluster of obstacles around the robot — front and both sides relative
# to the robot's current heading along the active trajectory. Each cluster is
# *world-anchored* (stamped at its absolute (x, y, z) on creation) and stays
# there until the robot moves more than `drop_distance` meters past it, at
# which point we let it fade out of the local pointcloud.
#
# This differs from `make_follow_obstacle_cloud`, which respawns a fresh set
# every update — handy for stress-testing replan, but not what the operator
# asked for here. The sticky model is closer to "real" sensor occupancy: the
# obstacle exists in the world, the robot just stops seeing it once it's
# behind by enough margin.
# ---------------------------------------------------------------------------


def spawn_world_anchored_centers(
    current: Point,
    trajectory: Sequence[Point],
    args: argparse.Namespace,
    rng: random.Random,
) -> List[Point]:
    """Generate a fresh batch of obstacle cluster centers around `current`.

    The returned points sit (a) ahead of the robot along the path tangent at
    `--world-anchored-front-distance`, and (b) on the left/right perpendicular
    of that tangent at `--world-anchored-side-offset`. We jitter each by
    `--random-jitter` so consecutive spawns do not perfectly co-locate.
    """
    if args.world_anchored_obstacles <= 0 or len(trajectory) < 2:
        return []

    # Pick a path tangent close to the current pose. We use the projection
    # along the active trajectory: find the nearest waypoint to `current` and
    # use the local (k, k+1) segment direction as forward.
    nearest_idx = 0
    nearest_dist_sq = float("inf")
    for i, point in enumerate(trajectory):
        dx = point[0] - current[0]
        dy = point[1] - current[1]
        dz = point[2] - current[2]
        d2 = dx * dx + dy * dy + dz * dz
        if d2 < nearest_dist_sq:
            nearest_dist_sq = d2
            nearest_idx = i
    fwd_idx = min(nearest_idx + 1, len(trajectory) - 1)
    if fwd_idx == nearest_idx and nearest_idx > 0:
        nearest_idx -= 1
    a = trajectory[nearest_idx]
    b = trajectory[fwd_idx]
    fx = b[0] - a[0]
    fy = b[1] - a[1]
    norm = math.hypot(fx, fy)
    if norm <= 1e-9:
        fx, fy = 1.0, 0.0
    else:
        fx /= norm
        fy /= norm
    # Perpendicular in XY: (-fy, fx) is the left side; (fy, -fx) is the right.
    lx, ly = -fy, fx
    rx, ry = fy, -fx

    front = (
        current[0] + fx * args.world_anchored_front_distance,
        current[1] + fy * args.world_anchored_front_distance,
        current[2],
    )
    left = (
        current[0] + fx * args.world_anchored_front_distance * 0.5
        + lx * args.world_anchored_side_offset,
        current[1] + fy * args.world_anchored_front_distance * 0.5
        + ly * args.world_anchored_side_offset,
        current[2],
    )
    right = (
        current[0] + fx * args.world_anchored_front_distance * 0.5
        + rx * args.world_anchored_side_offset,
        current[1] + fy * args.world_anchored_front_distance * 0.5
        + ry * args.world_anchored_side_offset,
        current[2],
    )

    # Pick `world_anchored_obstacles` of these slots (front always, then add
    # left/right rotating). Jitter each so successive spawns vary slightly.
    template = [front, left, right]
    chosen: List[Point] = []
    j = args.random_jitter
    for i in range(args.world_anchored_obstacles):
        base = template[i % 3]
        chosen.append(
            (
                base[0] + rng.uniform(-j, j),
                base[1] + rng.uniform(-j, j),
                base[2] + rng.uniform(-j * 0.5, j * 0.5),
            )
        )
    return chosen


def prune_world_anchored_obstacles(
    centers: List[Point],
    current: Point,
    drop_distance: float,
) -> Tuple[List[Point], int]:
    """Drop sticky obstacles whose center is more than `drop_distance` from `current`."""
    kept: List[Point] = []
    dropped = 0
    threshold_sq = drop_distance * drop_distance
    for center in centers:
        dx = center[0] - current[0]
        dy = center[1] - current[1]
        dz = center[2] - current[2]
        if dx * dx + dy * dy + dz * dz <= threshold_sq:
            kept.append(center)
        else:
            dropped += 1
    return kept, dropped


def world_anchored_cloud_points(centers: Sequence[Point], radius: float) -> List[Point]:
    """Expand each sticky center into the small 7-point cluster used elsewhere."""
    points: List[Point] = []
    for center in centers:
        points.extend(make_obstacle_cluster(center, radius))
    return points


def path_length(points: Sequence[Point]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(distance(points[index - 1], points[index]) for index in range(1, len(points)))


def point_along_path(points: Sequence[Point], target_distance: float) -> Point:
    if not points:
        raise ValueError("path is empty")
    if target_distance <= 0.0 or len(points) == 1:
        return points[0]

    traveled = 0.0
    for index in range(1, len(points)):
        segment_start = points[index - 1]
        segment_end = points[index]
        segment_length = distance(segment_start, segment_end)
        if segment_length <= 1e-9:
            continue
        if traveled + segment_length >= target_distance:
            ratio = (target_distance - traveled) / segment_length
            return (
                segment_start[0] + (segment_end[0] - segment_start[0]) * ratio,
                segment_start[1] + (segment_end[1] - segment_start[1]) * ratio,
                segment_start[2] + (segment_end[2] - segment_start[2]) * ratio,
            )
        traveled += segment_length
    return points[-1]


def path_xy_direction(points: Sequence[Point], target_distance: float) -> Tuple[float, float]:
    if len(points) < 2:
        return (1.0, 0.0)

    traveled = 0.0
    for index in range(1, len(points)):
        segment_start = points[index - 1]
        segment_end = points[index]
        segment_length = distance(segment_start, segment_end)
        if segment_length <= 1e-9:
            continue
        if traveled + segment_length >= target_distance:
            dx = segment_end[0] - segment_start[0]
            dy = segment_end[1] - segment_start[1]
            norm = math.hypot(dx, dy)
            if norm > 1e-9:
                return (dx / norm, dy / norm)
        traveled += segment_length

    dx = points[-1][0] - points[-2][0]
    dy = points[-1][1] - points[-2][1]
    norm = math.hypot(dx, dy)
    if norm <= 1e-9:
        return (1.0, 0.0)
    return (dx / norm, dy / norm)


def make_follow_obstacle_cloud(
    trajectory: Sequence[Point],
    args: argparse.Namespace,
    rng: random.Random,
) -> Tuple[List[Point], List[Point]]:
    if args.follow_local_obstacle_count <= 0 or len(trajectory) < 2:
        return [], []

    remaining_length = path_length(trajectory)
    if remaining_length <= args.follow_goal_tolerance:
        return [], []

    min_distance = max(0.0, args.follow_local_min_distance)
    max_distance = max(min_distance, args.follow_local_max_distance)
    max_distance = min(max_distance, max(min_distance, remaining_length - args.follow_goal_tolerance))

    centers: List[Point] = []
    for obstacle_index in range(args.follow_local_obstacle_count):
        target_distance = rng.uniform(min_distance, max_distance)
        base = point_along_path(trajectory, target_distance)
        if args.follow_block_path:
            side_x = 0.0
            side_y = 0.0
        else:
            direction_x, direction_y = path_xy_direction(trajectory, target_distance)
            side = args.follow_local_side_offset
            if args.follow_random_side:
                if rng.random() < 0.5:
                    side = -side
            elif obstacle_index % 2 == 1:
                side = -side
            side_x = direction_y * side
            side_y = -direction_x * side
        jitter = args.random_jitter
        centers.append((
            base[0] + side_x + rng.uniform(-jitter, jitter),
            base[1] + side_y + rng.uniform(-jitter, jitter),
            base[2] + rng.uniform(-jitter * 0.5, jitter * 0.5),
        ))

    points: List[Point] = []
    for center in centers:
        points.extend(make_obstacle_cluster(center, args.obstacle_radius))
    return points, centers


def outcome_name(status: str) -> str:
    if "safety_replan_success" in status:
        return "safety_replan_success"
    if "safety_replan_failed" in status:
        return "safety_replan_failed"
    if "safety_replan_emergency_stop" in status:
        return "safety_replan_emergency_stop"
    if "safety_emergency_stop" in status:
        return "safety_emergency_stop"
    return status.split(" ", 1)[0]


def spin_for(node: LocalReplanSmoke, seconds: float) -> None:
    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def clear_follow_local_cloud(node: LocalReplanSmoke, args: argparse.Namespace) -> None:
    if args.keep_local_at_end:
        return
    status_offset = len(node.observation.statuses)
    marker_offset = node.observation.marker_messages
    node.publish_local_cloud([])
    wait_for_status(
        node,
        status_offset,
        ("local_pointcloud_updated occupied=0",),
        args.timeout,
        "timed out waiting for empty local_pointcloud update",
    )
    if not getattr(args, 'track_markers', False):
        return
    wait_for(
        node,
        lambda: (
            node.observation.marker_messages > marker_offset and
            node.observation.local_marker_voxels == 0
        ),
        args.timeout,
        "timed out waiting for local marker clear",
    )


def run_follow_trajectory(
    node: LocalReplanSmoke,
    args: argparse.Namespace,
    initial: Sequence[Point],
    goal: Point,
) -> int:
    rng = random.Random(args.random_seed)
    current = initial[0]
    node.publish_current_pose(current)
    spin_for(node, args.publish_delay)

    start_time = time.monotonic()
    deadline = start_time + args.follow_timeout
    next_local_update = start_time
    # Delay the first sticky-obstacle spawn so the robot has time to move away
    # from its starting position before clusters are placed.  getattr keeps
    # backward compatibility while W2 adds the argparse flag.
    # In path-anchored mode anchors are static along the global path and not
    # fed by a spawn timer, so warmup is unnecessary — short-circuit to 0 unless
    # the operator overrode it.
    _warmup_seconds = getattr(args, 'world_anchored_warmup_seconds', 1.0)
    if getattr(args, 'anchor_mode', 'rolling') in ('path', 'travel'):
        _warmup_seconds = 0.0
    next_world_anchored_spawn = start_time + _warmup_seconds
    world_anchored_warmup_active = True  # True until first spawn fires
    # World-anchored sticky obstacles: list of cluster centers in absolute
    # world coordinates. Pruned every loop by distance from `current`; the
    # bridge sees the union expanded into a 7-point cluster per center via
    # `publish_local_cloud(world_anchored_cloud_points(...))`.
    world_anchored_centers: List[Point] = []
    world_anchored_spawned_total = 0
    world_anchored_dropped_total = 0
    # Path-anchored sticky anchors (W6): sample once from the first global
    # trajectory, frozen in world coordinates. Each frame we publish only the
    # anchors within `--anchor-publish-radius-m` of the robot, and drop ones
    # the robot has traveled `--anchor-drop-past-m` past along the path.
    path_anchors: List[Point] = []
    path_anchors_dropped_total = 0
    path_anchors_resampled_total = 0
    # Track which trajectory the path_anchors were sampled from so we can
    # detect a real replan (new trajectory length differs > 1m or pose count
    # changes >= 8) and resample anchors against the fresh global plan.
    path_anchor_source_length = 0.0
    path_anchor_source_pose_count = 0
    # Visible-anchor signature dedup: only republish the local pointcloud when
    # the quantised visible set actually changes. Cuts ~10Hz redundant work
    # at follow_rate=20Hz while the robot drifts inside one anchor's cell.
    last_visible_signature: Optional[Tuple] = None
    # Heartbeat republish is independent of signature dedup. The bridge keeps
    # only the latest local frame, so every heartbeat refreshes visible anchors
    # and also clears anchors that fell outside the 10 m publish radius or were
    # dropped after the robot passed them.
    last_publish_time = -1.0  # force first publish on the very first applicable step
    # v3.8 travel mode state.
    # travel_total_m: cumulative true robot motion along the world (advance_distance summed).
    # travel_next_spawn_at_m: next travel_total at which we spawn an anchor.
    #   Starts at anchor_travel_step_m so "every 5m" means 5/10/15m, not an
    #   extra 0m spawn before the robot has traveled.
    # path_anchor_spawn_travels: parallel list to path_anchors holding the
    #   travel_total_m at which each anchor was spawned. Used by drop logic
    #   instead of the path-shape-dependent _nearest_waypoint_index because
    #   anchors that fall off-path (3D stair turns) made the old logic never
    #   classify them as "passed".
    travel_total_m = 0.0
    travel_next_spawn_at_m = (
        initial_travel_anchor_spawn_at(args.anchor_travel_step_m)
        if getattr(args, 'anchor_mode', 'rolling') == 'travel'
        else float('inf')
    )
    travel_spawned_total = 0
    path_anchor_spawn_travels: List[float] = []
    if getattr(args, 'anchor_mode', 'rolling') == 'path' and len(initial) >= 2:
        # v3.8: --anchor-block-path forces side_offset=0 so anchors land
        # directly on the trajectory; the planner must produce a real detour
        # and replanFromCurrentPose's raw-A* fast path is actually exercised.
        anchor_side = 0.0 if getattr(args, 'anchor_block_path', False) else args.world_anchored_side_offset
        path_anchors = sample_anchors_along_path(
            initial,
            interval_m=args.anchor_spacing_m,
            side_offset=anchor_side,
            jitter=args.random_jitter,
            rng=rng,
        )
        path_anchor_source_length = path_length(initial)
        path_anchor_source_pose_count = len(initial)
        print(
            "nav3d_local_replan_smoke path_anchors_seeded "
            f"count={len(path_anchors)} spacing={args.anchor_spacing_m}m "
            f"radius={args.anchor_publish_radius_m}m drop_past={args.anchor_drop_past_m}m "
            f"side_offset={anchor_side}m block_path={getattr(args,'anchor_block_path',False)} "
            f"source_length={path_anchor_source_length:.2f}m source_poses={path_anchor_source_pose_count}"
        )
    elif getattr(args, 'anchor_mode', 'rolling') == 'travel':
        # No anchors at start — anchors are spawned as the robot travels.
        path_anchor_source_length = path_length(initial) if len(initial) >= 2 else 0.0
        path_anchor_source_pose_count = len(initial)
        print(
            "nav3d_local_replan_smoke travel_anchors_armed "
            f"step={args.anchor_travel_step_m}m spawn_ahead={args.anchor_travel_spawn_distance_m}m "
            f"radius={args.anchor_publish_radius_m}m drop_past={args.anchor_drop_past_m}m "
            f"initial_path_length={path_anchor_source_length:.2f}m initial_poses={path_anchor_source_pose_count}"
        )
    next_progress_print = start_time
    next_heartbeat = start_time + args.heartbeat_interval
    step_period = 1.0 / args.follow_rate
    # Wall-clock dt advance: nominal step_period assumes the loop sleeps
    # exactly 1/follow_rate, but Python+rclpy callbacks add ~50–100% overhead,
    # so a 20Hz follow_rate often only delivers ~9 steps/sec → effective speed
    # = follow_speed × 0.5 ≈ 0.5×target. Tracking the real wall delta makes
    # the publish travel = follow_speed × wall_seconds_since_last_pose, so
    # 2 m/s is honoured regardless of loop overhead. We cap each step at
    # 4× nominal to reject jitter spikes (debugger pause, GC stall).
    last_pose_time = start_time
    max_per_step_dt = step_period * 4.0
    steps = 0
    local_updates = 0
    status_offset = len(node.observation.statuses)
    trajectory_message_offset = node.observation.trajectory_messages
    # Idle watchdog: track when trajectories or local-marker counts last changed.
    # If neither moves for `--idle-timeout` seconds we abort instead of letting
    # the operator stare at a hung process.
    last_change_at = start_time
    last_traj_count = len(node.observation.trajectories)
    last_local_voxels = node.observation.local_marker_voxels
    consecutive_emergency_stops = 0

    while time.monotonic() < deadline:
        if _aborted:
            raise RuntimeError(f"aborted during follow steps={steps} local_updates={local_updates}")
        if (
            args.bridge_watchdog
            and node.count_publishers("/nav3d/trajectory") == 0
        ):
            raise RuntimeError(
                f"bridge_disappeared during follow steps={steps} local_updates={local_updates}"
            )

        # ---- idle detection ----
        now_check = time.monotonic()
        traj_count = len(node.observation.trajectories)
        local_voxels = node.observation.local_marker_voxels
        if traj_count != last_traj_count or local_voxels != last_local_voxels:
            last_change_at = now_check
            last_traj_count = traj_count
            last_local_voxels = local_voxels
        elif now_check - last_change_at > args.idle_timeout:
            raise RuntimeError(
                f"idle_timeout: no trajectory or local-marker change for {args.idle_timeout:.1f}s "
                f"(steps={steps} local_updates={local_updates})"
            )

        empty_action = empty_trajectory_follow_action(
            has_new_trajectory_message=(
                node.observation.trajectory_messages > trajectory_message_offset
            ),
            trajectory_pose_count=len(node.observation.last_trajectory),
            distance_to_goal=distance(current, goal),
            tolerance=args.follow_goal_tolerance,
        )
        if empty_action != "ignore":
            # Bridge sent an empty trajectory. The original logic treated this
            # as unambiguous follow_complete (bridge published empty Path =
            # tracking_goal_reached on the controller side). With sticky-world
            # obstacle mode that accumulates dozens of clusters near the
            # robot, bridge can transiently emit empty Paths during severe
            # replan churn — which used to spoof "complete" while the robot
            # was still meters from goal. Validate against true goal proximity
            # before accepting the completion signal.
            distance_to_goal = distance(current, goal)
            if empty_action == "recover":
                # Spurious empty trajectory; clear local cloud, sleep briefly,
                # and actively request a fresh current->goal plan. The bridge
                # clears active_goal_ on safety_replan_failed, so publishing only
                # current_pose would leave it with no target to replan toward.
                print(
                    "nav3d_local_replan_smoke spurious_empty_trajectory "
                    f"step={steps} distance_to_goal={distance_to_goal:.3f} "
                    f"tolerance={args.follow_goal_tolerance}",
                    file=sys.stderr,
                    flush=True,
                )
                # Reset rolling sticky state so it doesn't keep blocking the fresh
                # plan. Path-anchored anchors are world-fixed and must NOT be
                # cleared (the operator's layout is intentional).
                if getattr(args, 'anchor_mode', 'rolling') == 'rolling':
                    world_anchored_centers.clear()
                    next_world_anchored_spawn = time.monotonic() + getattr(args, 'world_anchored_warmup_seconds', 1.0)
                    world_anchored_warmup_active = True
                last_visible_signature = None
                last_publish_time = -1.0
                recover_follow_plan(node, args, current, goal, "spurious_empty_trajectory")
                trajectory_message_offset = node.observation.trajectory_messages
                status_offset = len(node.observation.statuses)
                last_change_at = time.monotonic()
                last_traj_count = len(node.observation.trajectories)
                last_local_voxels = node.observation.local_marker_voxels
                last_pose_time = last_change_at
                consecutive_emergency_stops = 0
                continue
            clear_follow_local_cloud(node, args)
            if (
                getattr(args, 'track_markers', False) and
                args.follow_local_obstacle_count > 0 and
                node.observation.max_local_marker_voxels == 0
            ):
                raise RuntimeError("follow completed but local obstacle markers never became visible")
            print(
                "nav3d_local_replan_smoke follow_complete "
                f"steps={steps} local_updates={local_updates} "
                f"remaining=0.000 distance_to_goal={distance_to_goal:.3f} "
                f"local_marker_voxels={node.observation.local_marker_voxels} "
                f"max_local_marker_voxels={node.observation.max_local_marker_voxels}"
            )
            return 0

        active = node.observation.last_trajectory
        if len(active) < 2:
            break

        # v3.8: when smoke is within follow_goal_tolerance of the requested
        # goal, exit immediately. Earlier `remaining` is path_length(active)
        # which can stay stale at the original value when the bridge stops
        # publishing fresh trajectories (e.g. it has decided the goal is
        # reached and cleared its active trajectory). Without this check the
        # smoke loop would idle at the goal until idle_timeout fired.
        distance_to_goal_now = distance(current, goal)
        if distance_to_goal_now <= args.follow_goal_tolerance:
            node.publish_current_pose(current)
            spin_for(node, args.publish_delay)
            # Best-effort cleanup of the local pointcloud so the bridge does not
            # carry sticky obstacles into the next run; do not abort the run if
            # the bridge happens to skip the empty-update confirmation.
            try:
                clear_follow_local_cloud(node, args)
            except Exception as cleanup_error:  # noqa: BLE001
                print(
                    "nav3d_local_replan_smoke follow_complete_cleanup_skipped "
                    f"reason={cleanup_error}"
                )
            print(
                "nav3d_local_replan_smoke follow_complete_by_proximity "
                f"steps={steps} local_updates={local_updates} "
                f"distance_to_goal={distance_to_goal_now:.3f} "
                f"path_anchors_dropped_total={path_anchors_dropped_total}"
            )
            return 0

        # Remaining distance must be measured from the smoke's current projected
        # position on the latest published trajectory. Using full path_length()
        # made local replans look stale because the bridge publishes a fresh
        # segment while the smoke is already part-way through it.
        remaining = remaining_distance_along_path(active, current)
        if remaining <= args.follow_goal_tolerance:
            distance_to_goal_now = distance(current, goal)
            if distance_to_goal_now > args.follow_goal_tolerance:
                endpoint_distance_to_goal = distance(active[-1], goal)
                if path_endpoint_reaches_goal(active, goal, args.follow_goal_tolerance):
                    current = active[-1]
                    node.publish_current_pose(current)
                    spin_for(node, args.publish_delay)
                    clear_follow_local_cloud(node, args)
                    print(
                        "nav3d_local_replan_smoke follow_complete_by_endpoint "
                        f"steps={steps} local_updates={local_updates} "
                        f"remaining={remaining:.3f} "
                        f"distance_to_goal_before_endpoint={distance_to_goal_now:.3f} "
                        f"endpoint_distance_to_goal={endpoint_distance_to_goal:.3f} "
                        f"local_marker_voxels={node.observation.local_marker_voxels} "
                        f"max_local_marker_voxels={node.observation.max_local_marker_voxels}"
                    )
                    return 0
                print(
                    "nav3d_local_replan_smoke active_segment_exhausted_before_goal "
                    f"step={steps} remaining={remaining:.3f} "
                    f"distance_to_goal={distance_to_goal_now:.3f} "
                    f"endpoint_distance_to_goal={endpoint_distance_to_goal:.3f} "
                    f"tolerance={args.follow_goal_tolerance}",
                    file=sys.stderr,
                    flush=True,
                )
                current = active[-1]
                node.publish_current_pose(current)
                spin_for(node, args.publish_delay)
                continue
            current = active[-1]
            node.publish_current_pose(current)
            spin_for(node, args.publish_delay)
            clear_follow_local_cloud(node, args)
            if (
                getattr(args, 'track_markers', False) and
                args.follow_local_obstacle_count > 0 and
                node.observation.max_local_marker_voxels == 0
            ):
                raise RuntimeError("follow completed but local obstacle markers never became visible")
            print(
                "nav3d_local_replan_smoke follow_complete "
                f"steps={steps} local_updates={local_updates} "
                f"remaining={remaining:.3f} local_marker_voxels={node.observation.local_marker_voxels} "
                f"max_local_marker_voxels={node.observation.max_local_marker_voxels}"
            )
            return 0

        safety_stop = first_status_since(
            node,
            status_offset,
            ("safety_replan_emergency_stop", "safety_emergency_stop"),
        )
        if safety_stop is not None:
            # Soft-tolerate up to N emergency stops: bridge sometimes brakes for
            # a single unsolvable obstacle frame and recovers when it clears.
            consecutive_emergency_stops += 1
            print(
                f"nav3d_local_replan_smoke emergency_stop_observed count={consecutive_emergency_stops}/"
                f"{args.emergency_stop_tolerance} status={safety_stop}",
                file=sys.stderr,
                flush=True,
            )
            if consecutive_emergency_stops >= args.emergency_stop_tolerance:
                raise RuntimeError(f"follow stopped by safety response: {safety_stop}")
            # Clear local cloud so next plan has a chance, push pose, sleep briefly.
            # Reset rolling sticky state so the recovered plan isn't immediately
            # blocked. In path-anchored mode the anchors are world-fixed and must
            # NOT be cleared on recovery — the planner needs them to find a
            # stable replan that respects the operator's obstacle layout.
            if getattr(args, 'anchor_mode', 'rolling') == 'rolling':
                world_anchored_centers.clear()
                next_world_anchored_spawn = time.monotonic() + getattr(args, 'world_anchored_warmup_seconds', 1.0)
                world_anchored_warmup_active = True
            last_visible_signature = None
            last_publish_time = -1.0
            recover_follow_plan(node, args, current, goal, "safety_emergency_stop")
            trajectory_message_offset = node.observation.trajectory_messages
            status_offset = len(node.observation.statuses)
            last_change_at = time.monotonic()
            last_traj_count = len(node.observation.trajectories)
            last_local_voxels = node.observation.local_marker_voxels
            last_pose_time = last_change_at
            continue
        # No emergency this loop — reset counter.
        consecutive_emergency_stops = 0

        # Wall-clock dt advance: see step_period comment above. dt_step is
        # the real seconds since the last current_pose publish, capped to
        # max_per_step_dt to absorb jitter spikes without teleporting the
        # robot through obstacles.
        now_advance = time.monotonic()
        dt_step = min(now_advance - last_pose_time, max_per_step_dt)
        if dt_step <= 0.0:
            dt_step = step_period  # first iteration / clock skew fallback
        advance_distance = min(args.follow_speed * dt_step, max(remaining, 0.0))
        # World-frame integration: project the smoke's current position onto the
        # latest active polyline, then advance along arc length. Segment
        # projection avoids jumping backward/sideways on freshly-replanned
        # trajectories where nearest-waypoint selection is ambiguous.
        current = advance_along_path_from_current(active, current, advance_distance)
        last_pose_time = now_advance
        node.publish_current_pose(current)
        steps += 1

        now_time = time.monotonic()
        if getattr(args, 'anchor_mode', 'rolling') == 'travel':
            # v3.8 travel mode (operator's spec).
            # 1) Accumulate true robot travel.
            # 2) Every --anchor-travel-step-m, spawn ONE anchor placed
            #    --anchor-travel-spawn-distance-m AHEAD of the robot ALONG the
            #    active trajectory (point_along_path arc length, not raw XY
            #    tangent — the path-tangent variant placed anchors off-floor
            #    on 3D stair turns, which the operator caught visually).
            # 3) Drop anchors whose travel_total has advanced > drop_past
            #    past their spawn travel — independent of trajectory shape.
            travel_total_m += advance_distance
            while travel_total_m >= travel_next_spawn_at_m and len(active) >= 2:
                # Project current onto active to get the robot's arc-length on
                # the published trajectory; spawn ahead by spawn_distance along
                # that path so the obstacle lands ON the path, not in midair.
                proj_arc = nearest_path_arc_length(active, current)
                target_arc = proj_arc + args.anchor_travel_spawn_distance_m
                spawn = point_along_path(active, target_arc)
                jit = args.random_jitter
                spawn = (
                    spawn[0] + rng.uniform(-jit, jit),
                    spawn[1] + rng.uniform(-jit, jit),
                    spawn[2] + rng.uniform(-jit * 0.5, jit * 0.5),
                )
                path_anchors.append(spawn)
                path_anchor_spawn_travels.append(travel_total_m)
                travel_spawned_total += 1
                travel_next_spawn_at_m += args.anchor_travel_step_m
                last_visible_signature = None  # force a republish after spawn
                print(
                    "nav3d_local_replan_smoke travel_anchor_spawned "
                    f"step={steps} travel_total={travel_total_m:.2f}m "
                    f"spawn=({spawn[0]:.2f},{spawn[1]:.2f},{spawn[2]:.2f}) "
                    f"current=({current[0]:.2f},{current[1]:.2f},{current[2]:.2f}) "
                    f"total_anchors={len(path_anchors)} spawned_total={travel_spawned_total}"
                )
            # Travel-based drop: anchor is "passed" when robot's cumulative
            # travel exceeds spawn_travel + spawn_distance + drop_past_m. The
            # +spawn_distance term accounts for the gap the anchor was placed
            # ahead at spawn time (we want robot to actually reach + cross it).
            if path_anchors:
                kept_anchors: List[Point] = []
                kept_travels: List[float] = []
                drop_threshold = args.anchor_drop_past_m + args.anchor_travel_spawn_distance_m
                for anchor, spawn_travel in zip(path_anchors, path_anchor_spawn_travels):
                    if (travel_total_m - spawn_travel) > drop_threshold:
                        path_anchors_dropped_total += 1
                    else:
                        kept_anchors.append(anchor)
                        kept_travels.append(spawn_travel)
                if len(kept_anchors) != len(path_anchors):
                    last_visible_signature = None  # force republish on drop, including empty clouds
                path_anchors = kept_anchors
                path_anchor_spawn_travels = kept_travels
            # Publish only those within radius.
            visible_anchors = filter_anchors_in_radius(
                path_anchors,
                current,
                args.anchor_publish_radius_m,
            )
            visible_signature = visible_anchors_signature(visible_anchors)
            heartbeat_due = (now_time - last_publish_time) >= args.anchor_publish_interval_s
            if should_publish_anchor_cloud(
                visible_signature=visible_signature,
                last_visible_signature=last_visible_signature,
                heartbeat_due=heartbeat_due,
                visible_count=len(visible_anchors),
            ):
                # Travel mode publishes a compact dense voxel cube per anchor
                # so the planner has to detour without growing the local grid
                # into the old 250/500+ voxel range. Heartbeat republish keeps
                # the latest-frame bridge grid synchronized with the visible
                # anchor set, including empty clouds after drops.
                cloud = dense_world_anchored_cloud_points(
                    visible_anchors,
                    args.anchor_travel_half_extent_m,
                    args.anchor_travel_voxel_resolution_m,
                )
                try:
                    node.publish_local_cloud(cloud)
                except RuntimeError as error:
                    print(
                        f"nav3d_local_replan_smoke local_cloud_publish_failed step={steps} error={error}",
                        file=sys.stderr,
                        flush=True,
                    )
                else:
                    local_updates += 1
                    last_visible_signature = visible_signature
                    last_publish_time = now_time
        elif getattr(args, 'anchor_mode', 'rolling') == 'path':
            # v3.8 (--anchor-static-after-seed=true, default): once anchors are
            # seeded from the initial global plan they are world-fixed and never
            # resampled, even if the bridge republishes a fresh trajectory. This
            # matches the operator's spec exactly: "全局路径生成后每隔5m才会生成
            # 一次障碍物" — once at plan time, never again.
            #
            # Legacy resample (--anchor-allow-resample) re-samples whenever the
            # bridge emits a materially different trajectory. Operator perceives
            # this as "still spawning dynamic obstacles", so we keep it off by
            # default.
            current_traj_length = path_length(active)
            traj_changed = (
                not getattr(args, 'anchor_static_after_seed', True) and
                len(active) >= 2 and (
                    abs(current_traj_length - path_anchor_source_length) > 1.0 or
                    abs(len(active) - path_anchor_source_pose_count) >= 8
                )
            )
            if traj_changed:
                # v3.8: keep block_path semantics on resample — operator's
                # 路径-阻塞 layout must persist across replans.
                anchor_side = 0.0 if getattr(args, 'anchor_block_path', False) else args.world_anchored_side_offset
                fresh_anchors = sample_anchors_along_path(
                    active,
                    interval_m=args.anchor_spacing_m,
                    side_offset=anchor_side,
                    jitter=args.random_jitter,
                    rng=rng,
                )
                if fresh_anchors:
                    path_anchors = fresh_anchors
                    path_anchor_source_length = current_traj_length
                    path_anchor_source_pose_count = len(active)
                    path_anchors_resampled_total += 1
                    last_visible_signature = None  # force a republish next step
                    print(
                        "nav3d_local_replan_smoke path_anchors_resampled "
                        f"step={steps} count={len(path_anchors)} "
                        f"new_length={current_traj_length:.2f}m "
                        f"resamples_total={path_anchors_resampled_total}"
                    )
            if path_anchors:
                path_anchors, dropped_now = drop_passed_anchors(
                    path_anchors,
                    active,  # use the *active* trajectory so dropping reflects current plan
                    current,
                    args.anchor_drop_past_m,
                )
                path_anchors_dropped_total += dropped_now
            visible_anchors = filter_anchors_in_radius(
                path_anchors,
                current,
                args.anchor_publish_radius_m,
            )
            visible_signature = visible_anchors_signature(visible_anchors)
            if visible_signature != last_visible_signature:
                cloud = world_anchored_cloud_points(visible_anchors, args.obstacle_radius)
                try:
                    node.publish_local_cloud(cloud)
                except RuntimeError as error:
                    print(
                        f"nav3d_local_replan_smoke local_cloud_publish_failed step={steps} error={error}",
                        file=sys.stderr,
                        flush=True,
                    )
                else:
                    local_updates += 1
                    last_visible_signature = visible_signature
        elif args.world_anchored_obstacles > 0:
            # Sticky path: spawn fresh clusters every spawn_interval, prune
            # those past drop_distance, publish full union every step so the
            # bridge sees a stable world map (max_observation_age stays fresh
            # because we re-stamp on every publish).
            world_anchored_centers, dropped = prune_world_anchored_obstacles(
                world_anchored_centers,
                current,
                args.world_anchored_drop_distance,
            )
            world_anchored_dropped_total += dropped
            if now_time >= next_world_anchored_spawn:
                fresh = spawn_world_anchored_centers(current, active, args, rng)
                world_anchored_centers.extend(fresh)
                world_anchored_spawned_total += len(fresh)
                # FIFO cap eviction so the bridge never has to plan against
                # an unbounded sticky cluster set; the oldest centers go first.
                if (
                    args.world_anchored_cap > 0
                    and len(world_anchored_centers) > args.world_anchored_cap
                ):
                    overflow = len(world_anchored_centers) - args.world_anchored_cap
                    world_anchored_centers = world_anchored_centers[overflow:]
                    world_anchored_dropped_total += overflow
                next_world_anchored_spawn = now_time + args.world_anchored_spawn_interval
                world_anchored_warmup_active = False  # warmup has elapsed; first spawn fired
                if fresh:
                    print(
                        "nav3d_local_replan_smoke world_anchored_spawn "
                        f"step={steps} fresh={len(fresh)} total={len(world_anchored_centers)} "
                        f"spawned_total={world_anchored_spawned_total} "
                        f"dropped_total={world_anchored_dropped_total} "
                        f"current=({current[0]:.2f},{current[1]:.2f},{current[2]:.2f})"
                    )
            cloud = world_anchored_cloud_points(world_anchored_centers, args.obstacle_radius)
            try:
                node.publish_local_cloud(cloud)
            except RuntimeError as error:
                print(
                    f"nav3d_local_replan_smoke local_cloud_publish_failed step={steps} error={error}",
                    file=sys.stderr,
                    flush=True,
                )
            else:
                local_updates += 1
        elif now_time >= next_local_update:
            cloud, centers = make_follow_obstacle_cloud(active, args, rng)
            try:
                node.publish_local_cloud(cloud)
            except RuntimeError as error:
                print(
                    f"nav3d_local_replan_smoke local_cloud_publish_failed step={steps} error={error}",
                    file=sys.stderr,
                    flush=True,
                )
            else:
                local_updates += 1
                if centers:
                    print(
                        "nav3d_local_replan_smoke follow_local_update "
                        f"step={steps} centers={len(centers)} points={len(cloud)} "
                        f"remaining={remaining:.3f}"
                    )
            next_local_update = now_time + args.follow_local_update_interval

        if now_time >= next_progress_print:
            print(
                "nav3d_local_replan_smoke follow_progress "
                f"step={steps} remaining={remaining:.3f} "
                f"current=({current[0]:.2f},{current[1]:.2f},{current[2]:.2f}) "
                f"dt_step={dt_step*1000:.1f}ms advance={advance_distance:.3f}m "
                f"trajectories={len(node.observation.trajectories)} "
                f"local_marker_voxels={node.observation.local_marker_voxels} "
                f"max_local_marker_voxels={node.observation.max_local_marker_voxels} "
                f"active_sticky_centers={len(world_anchored_centers)} "
                f"sticky_spawned_total={world_anchored_spawned_total} "
                f"sticky_dropped_total={world_anchored_dropped_total} "
                f"path_anchors_total={len(path_anchors)} "
                f"path_anchors_visible={len(filter_anchors_in_radius(path_anchors, current, args.anchor_publish_radius_m)) if path_anchors else 0} "
                f"path_anchors_dropped_total={path_anchors_dropped_total} "
                f"path_anchors_resampled_total={path_anchors_resampled_total} "
                f"local_updates={local_updates}"
            )
            next_progress_print = now_time + args.follow_print_interval

        if now_time >= next_heartbeat:
            print(
                "nav3d_local_replan_smoke heartbeat phase=follow "
                f"elapsed={now_time - start_time:.1f}s steps={steps} "
                f"local_updates={local_updates} idle_for={now_time - last_change_at:.1f}s",
                flush=True,
            )
            next_heartbeat = now_time + args.heartbeat_interval

        # Bounded-time spin: drain callbacks for at most step_period total,
        # then return. The earlier `spin_for(node, step_period)` would *loop*
        # spin_once until step_period elapsed, but each spin_once with a
        # 50ms timeout amplifies when bridge publishes status/trajectory/marker
        # bursts (200+ msg/s). The drained-then-sleep variant keeps the
        # follow loop at the requested follow_rate even under callback load.
        drain_budget = max(0.0, step_period * 0.4)
        drain_deadline = time.monotonic() + drain_budget
        while time.monotonic() < drain_deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
        sleep_remaining = step_period - (time.monotonic() - now_advance)
        if sleep_remaining > 0:
            time.sleep(sleep_remaining)

    raise TimeoutError(
        "timed out before follow_complete; "
        f"steps={steps} local_updates={local_updates}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Publish /nav3d/current_pose and /nav3d/local_pointcloud, then verify local safety replan.",
    )
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--start", nargs=3, default=("-12", "3.19", "0.351"), metavar=("X", "Y", "Z"))
    parser.add_argument("--goal", nargs=3, default=("8.16", "0.418", "0.351"), metavar=("X", "Y", "Z"))
    parser.add_argument("--current-index", type=int, default=8)
    parser.add_argument("--obstacle-index", type=int, default=24)
    parser.add_argument("--obstacle-lookahead-distance", type=float, default=0.0)
    parser.add_argument("--obstacle-radius", type=float, default=0.2)
    parser.add_argument(
        "--expected-outcome",
        choices=("safety_response", "replan_success", "emergency_stop"),
        default="safety_response",
        help=(
            "safety_response: pass on the first safety_replan_success; if all "
            "attempts only produce emergency_stop, exit 2 (partial_pass) so a "
            "genuine 'no alternate path' is distinguishable from a healthy replan. "
            "replan_success requires a changed trajectory; emergency_stop requires "
            "a stop response."
        ),
    )
    parser.add_argument("--attempts", type=int, default=3)
    parser.add_argument(
        "--random-obstacles",
        type=int,
        default=0,
        help="Add this many random obstacle clusters on the active trajectory; 0 uses one deterministic cluster.",
    )
    parser.add_argument("--random-seed", type=int, default=7)
    parser.add_argument("--random-min-index-offset", type=int, default=16)
    parser.add_argument("--random-max-index-offset", type=int, default=96)
    parser.add_argument("--random-jitter", type=float, default=0.0)
    parser.add_argument("--min-trajectory-poses", type=int, default=50)
    parser.add_argument("--min-trajectory-delta", type=float, default=0.02)
    parser.add_argument("--timeout", type=float, default=50.0)
    parser.add_argument("--publish-delay", type=float, default=0.2)
    parser.add_argument(
        "--follow-trajectory",
        action="store_true",
        default=True,
        help=(
            "Continuously publish /nav3d/current_pose along the latest trajectory "
            "and refresh local obstacles. Default is True so a bare `ros2 run "
            "nav3d_ros2_bridge nav3d_local_replan_smoke.py` walks the operator's "
            "start/goal end-to-end with sticky-world obstacle injection. Pair "
            "with --no-follow-trajectory for the legacy single-attempt replan check."
        ),
    )
    parser.add_argument(
        "--no-follow-trajectory",
        dest="follow_trajectory",
        action="store_false",
        help="Disable follow-trajectory (legacy attempt-mode replan check).",
    )
    parser.add_argument("--follow-speed", type=float, default=0.5)
    parser.add_argument("--follow-rate", type=float, default=20.0)
    parser.add_argument("--follow-timeout", type=float, default=1200.0)
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=360.0,
        help=(
            "Abort the follow phase if neither trajectories nor local marker counts "
            "change for this many seconds. Guards against bridge silently wedging. "
            "Default 360s tolerates building2_9 long-corridor replan churn where "
            "smoke-side anchor recomputes and bridge-side B-spline fits can stall "
            "trajectory updates for ~90s while the planner refines around freshly "
            "published sticky obstacles."
        ),
    )
    parser.add_argument("--follow-goal-tolerance", type=float, default=0.25)
    parser.add_argument("--follow-local-update-interval", type=float, default=1.0)
    parser.add_argument("--follow-local-obstacle-count", type=int, default=4)
    parser.add_argument("--follow-local-min-distance", type=float, default=1.2)
    parser.add_argument("--follow-local-max-distance", type=float, default=3.0)
    parser.add_argument("--follow-local-side-offset", type=float, default=0.3)
    parser.add_argument("--follow-random-side", action="store_true")
    parser.add_argument(
        "--follow-block-path",
        action="store_true",
        help="Place follow-mode local obstacles directly on the remaining trajectory instead of beside it.",
    )
    parser.add_argument("--follow-print-interval", type=float, default=2.0)
    parser.add_argument("--keep-local-at-end", action="store_true")
    parser.add_argument(
        "--heartbeat-interval",
        type=float,
        default=5.0,
        help="Seconds between phase heartbeat lines so a hung phase is visible.",
    )
    parser.add_argument(
        "--bridge-watchdog",
        action="store_true",
        help=(
            "When set, every blocking wait checks count_publishers('/nav3d/trajectory'); "
            "if the bridge disappears we fail fast instead of timing out."
        ),
    )
    parser.add_argument(
        "--emergency-stop-tolerance",
        type=int,
        default=3,
        help=(
            "Number of consecutive safety_*_emergency_stop statuses tolerated "
            "before the smoke gives up. Each tolerated stop sleeps briefly and "
            "lets the bridge attempt a recovery replan."
        ),
    )
    parser.add_argument(
        "--world-anchored-obstacles",
        type=int,
        default=3,
        help=(
            "Sticky-world obstacle mode (operator-requested). When >0, every "
            "--world-anchored-spawn-interval seconds spawn this many clusters "
            "ahead of and beside the robot at absolute world coordinates; "
            "centers persist until they are >--world-anchored-drop-distance "
            "behind the robot. Replaces the rolling --follow-local-obstacle "
            "behavior in the follow-trajectory loop. Default 3 enables sticky "
            "mode out-of-the-box for `ros2 run nav3d_local_replan_smoke.py`."
        ),
    )
    parser.add_argument("--world-anchored-spawn-interval", type=float, default=8.0)
    parser.add_argument("--world-anchored-front-distance", type=float, default=2.5)
    parser.add_argument("--world-anchored-side-offset", type=float, default=0.6)
    parser.add_argument("--world-anchored-drop-distance", type=float, default=4.0)
    parser.add_argument(
        "--world-anchored-cap",
        type=int,
        default=6,
        help=(
            "Hard upper bound on the number of sticky obstacle centers held "
            "live at once. When the cap is reached, the oldest centers are "
            "evicted FIFO. Prevents bridge from being swamped during severe "
            "replan churn that keeps the robot near the same spot. Default 6 "
            "(≈ 2 spawn cycles) keeps front-view obstacle density reasonable "
            "without trapping the planner in a fully boxed corridor."
        ),
    )
    parser.add_argument(
        "--snap-z-fallback",
        type=float,
        default=1.0,
        help=(
            "If the initial plan_failed because the requested start/goal z is "
            "unreachable on the loaded PCD, retry once with start.z = goal.z = "
            "this fallback value. 0 disables the retry — pass e.g. "
            "1.0 when running against building2_9.pcd whose corridor sits at "
            "z≈1.0 and the operator gave a ground-level z by mistake."
        ),
    )
    parser.add_argument(
        "--world-anchored-warmup-seconds",
        type=float,
        default=1.0,
        help=(
            "Delay first sticky-obstacle spawn by this many seconds after start "
            "or after a recovery (spurious empty trajectory / safety_emergency_stop). "
            "Prevents the planner from being trapped by clusters seeded before "
            "the robot has any forward motion. Set to 0 to disable warmup."
        ),
    )
    # ---- W6 / v3.8: path / travel / rolling anchor modes -----------------
    parser.add_argument(
        "--anchor-mode",
        choices=("path", "travel", "rolling"),
        default="travel",
        help=(
            "travel = (operator's v3.8 spec, default) every --anchor-travel-step-m "
            "meters the ROBOT travels, spawn one world-fixed obstacle "
            "--anchor-travel-spawn-distance-m ahead of its current pose. "
            "Anchors stay world-fixed; only those within --anchor-publish-radius-m "
            "of the robot are published; once the robot is --anchor-drop-past-m "
            "past an anchor along the path, it is dropped permanently. "
            "path = legacy v3.5 — sample sticky anchors ONCE at plan time, every "
            "--anchor-spacing-m meters along the initial trajectory. "
            "rolling = oldest behaviour — time-based --world-anchored-* spawning."
        ),
    )
    parser.add_argument(
        "--anchor-travel-step-m",
        type=float,
        default=5.0,
        help="travel mode: spawn a fresh anchor every N meters of robot travel.",
    )
    parser.add_argument(
        "--anchor-travel-spawn-distance-m",
        type=float,
        default=3.0,
        help=(
            "travel mode: spawn the new anchor this many meters ahead of the "
            "robot's current pose along the active path."
        ),
    )
    parser.add_argument(
        "--anchor-travel-half-extent-m",
        type=float,
        default=0.25,
        help=(
            "travel mode: half-edge of the dense obstacle voxel cluster spawned "
            "at each anchor. Default 0.25m keeps the local obstacle compact while "
            "still filling neighboring map-resolution voxels."
        ),
    )
    parser.add_argument(
        "--anchor-travel-voxel-resolution-m",
        type=float,
        default=0.2,
        help=(
            "travel mode: voxel grid spacing for the dense obstacle cluster. "
            "Should match map.resolution from nav3d_bridge.yaml so each emitted "
            "point lands on a planning-map voxel center."
        ),
    )
    parser.add_argument(
        "--anchor-publish-interval-s",
        type=float,
        default=1.0,
        help=(
            "travel/path mode: republish the local pointcloud at minimum this "
            "often as a watchdog. Bridge yaml uses latest-frame local-grid "
            "semantics, so republishing also clears dropped anchors quickly. "
            "Default 1s keeps obstacle loading responsive without flooding "
            "the bridge."
        ),
    )
    parser.add_argument(
        "--anchor-spacing-m",
        type=float,
        default=5.0,
        help="Path-anchored mode: spacing between sticky anchors along the global trajectory.",
    )
    parser.add_argument(
        "--anchor-publish-radius-m",
        type=float,
        default=10.0,
        help=(
            "Path-anchored mode: only publish anchors within this radius of the "
            "robot. Anchors outside the radius stay dormant; they re-appear when "
            "the robot approaches."
        ),
    )
    parser.add_argument(
        "--anchor-drop-past-m",
        type=float,
        default=10.0,
        help=(
            "Path-anchored mode: permanently drop an anchor once the robot has "
            "traveled this many meters past it along the trajectory. Default 10m "
            "matches the operator's v3.8 spec ('经过了障碍物10M'); v3.5 used 5m which "
            "kept obstacles in published_set too long for the spec."
        ),
    )
    parser.add_argument(
        "--anchor-block-path",
        action="store_true",
        default=False,
        help=(
            "Path-anchored mode: place anchors ON the trajectory (side_offset=0) "
            "instead of perpendicular to it. Forces the planner to actually "
            "detour, exercising the safety_replan + raw-A* fast path inside "
            "replanFromCurrentPose. Default False keeps v3.7 e2e contract "
            "(lateral anchors) so existing tests pass."
        ),
    )
    parser.add_argument(
        "--anchor-static-after-seed",
        action="store_true",
        default=True,
        help=(
            "Path-anchored mode: once seeded, NEVER resample anchors when the "
            "bridge republishes a different trajectory. Matches operator spec "
            "'全局路径生成后每隔5m才会生成一次障碍物 ... 不会再随机器人移动生成动态障碍'. "
            "When False (legacy v3.7 behaviour) anchors resample whenever the "
            "bridge emits a materially different trajectory, which the operator "
            "perceives as 'still spawning dynamic obstacles' even though anchors "
            "themselves are world-fixed between resamples."
        ),
    )
    parser.add_argument(
        "--anchor-allow-resample",
        dest="anchor_static_after_seed",
        action="store_false",
        help="Inverse of --anchor-static-after-seed: legacy resample-on-replan behaviour.",
    )
    parser.add_argument(
        "--track-markers",
        action="store_true",
        default=False,
        help=(
            "Subscribe to /nav3d/planning_occupied_markers. The MarkerArray "
            "payload is huge on dense maps (62k+ voxels per message); "
            "deserialising every message throttles the smoke loop to ~5Hz, "
            "which makes follow_speed look slow even at high follow_rate. "
            "Default False — most validation goals do not need marker counts."
        ),
    )
    args = parser.parse_args()
    if args.attempts < 1:
        parser.error("--attempts must be >= 1")
    if args.random_obstacles < 0:
        parser.error("--random-obstacles must be >= 0")
    if args.follow_speed <= 0.0:
        parser.error("--follow-speed must be > 0")
    if args.follow_rate <= 0.0:
        parser.error("--follow-rate must be > 0")
    if args.follow_timeout <= 0.0:
        parser.error("--follow-timeout must be > 0")
    if args.anchor_travel_step_m <= 0.0:
        parser.error("--anchor-travel-step-m must be > 0")
    if args.follow_local_update_interval <= 0.0:
        parser.error("--follow-local-update-interval must be > 0")
    if args.follow_local_obstacle_count < 0:
        parser.error("--follow-local-obstacle-count must be >= 0")

    start = parse_point(args.start, "--start")
    goal = parse_point(args.goal, "--goal")

    _install_signal_handlers()
    rclpy.init()
    node = LocalReplanSmoke(args)
    try:
        end_wait = time.monotonic() + max(1.0, args.publish_delay)
        while time.monotonic() < end_wait:
            rclpy.spin_once(node, timeout_sec=0.05)

        status_offset = len(node.observation.statuses)
        trajectory_offset = len(node.observation.trajectories)
        node.publish_pose_pair(start, goal)
        try:
            wait_for_initial_plan(
                node,
                status_offset,
                trajectory_offset,
                args.min_trajectory_poses,
                args.timeout,
            )
        except TimeoutError:
            # Initial plan failed — most commonly the operator gave a z that
            # the loaded PCD's planning grid does not have a free cell for.
            # Retry once at --snap-z-fallback if set.
            last_status = latest_snap_z_failure_status(
                node.observation.statuses[status_offset:]
            )
            if (
                args.snap_z_fallback != 0.0
                and should_retry_snap_z(last_status)
            ):
                snapped_start = (start[0], start[1], args.snap_z_fallback)
                snapped_goal = (goal[0], goal[1], args.snap_z_fallback)
                print(
                    "nav3d_local_replan_smoke snap_z_retry "
                    f"original_start_z={start[2]} original_goal_z={goal[2]} "
                    f"retry_z={args.snap_z_fallback}",
                    file=sys.stderr,
                    flush=True,
                )
                status_offset = len(node.observation.statuses)
                trajectory_offset = len(node.observation.trajectories)
                node.publish_pose_pair(snapped_start, snapped_goal)
                wait_for_initial_plan(
                    node,
                    status_offset,
                    trajectory_offset,
                    args.min_trajectory_poses,
                    args.timeout,
                )
                start = snapped_start
                goal = snapped_goal
            else:
                raise

        initial = node.observation.trajectories[-1]
        if args.follow_trajectory:
            return run_follow_trajectory(node, args, initial, goal)

        current_index = clamp_index(args.current_index, len(initial))
        rng = random.Random(args.random_seed)
        attempt_summaries: List[str] = []
        last_error = "no attempts ran"

        for attempt in range(1, args.attempts + 1):
            active = node.observation.trajectories[-1]
            current_index = clamp_index(current_index, len(active))
            obstacle_index = index_at_distance(
                active,
                current_index,
                args.obstacle_index + (attempt - 1) * args.random_min_index_offset,
                args.obstacle_lookahead_distance,
            )
            obstacle_index = min(max(current_index + 1, obstacle_index), len(active) - 1)
            current = active[current_index]
            obstacle_points, obstacle_centers = make_attempt_obstacles(
                active,
                current_index,
                obstacle_index,
                args,
                rng,
            )
            status_offset = len(node.observation.statuses)
            trajectory_offset = len(node.observation.trajectories)
            marker_offset = node.observation.marker_messages
            node.publish_local_obstacles(current, obstacle_points)

            wait_for_status(
                node,
                status_offset,
                ("local_pointcloud_updated",),
                args.timeout,
                "timed out waiting for local_pointcloud_updated",
            )
            wait_for(
                node,
                lambda marker_offset=marker_offset: (
                    node.observation.marker_messages > marker_offset and
                    node.observation.local_marker_voxels > 0
                ),
                args.timeout,
                "timed out waiting for nav3d_local_occupied_voxels marker",
            )
            wait_for_status(
                node,
                status_offset,
                ("safety_replan_needed", "safety_emergency_stop"),
                args.timeout,
                "timed out waiting for safety_replan_needed or safety_emergency_stop",
            )
            response_status = wait_for_status(
                node,
                status_offset,
                (
                    "safety_replan_success",
                    "safety_replan_emergency_stop",
                    "safety_emergency_stop",
                ),
                args.timeout,
                "timed out waiting for safety replan response",
            )
            outcome = outcome_name(response_status)
            replanned = node.observation.trajectories[-1]
            trajectory_delta = max_trajectory_delta(active, replanned)
            changed_trajectory = (
                len(node.observation.trajectories) > trajectory_offset and
                trajectory_delta >= args.min_trajectory_delta
            )
            attempt_summary = (
                f"attempt={attempt} outcome={outcome} "
                f"local_marker_voxels={node.observation.local_marker_voxels} "
                f"trajectory_delta={trajectory_delta:.3f} current={current} "
                f"obstacle_centers={obstacle_centers}"
            )
            attempt_summaries.append(attempt_summary)

            if args.expected_outcome == "safety_response":
                # safety_response means "the bridge took *some* safe action".
                # The original implementation accepted the first attempt
                # unconditionally — including a bare safety_emergency_stop —
                # which let an obvious obstacle-avoidance regression silently
                # pass with status `outcome=safety_emergency_stop`. Operators
                # rightly read this as "the robot just slammed the brakes and
                # called it success". The new policy:
                #   1. safety_replan_success on any attempt   → pass (return 0)
                #   2. emergency_stop → restore + retry until --attempts run out
                #   3. all attempts emergency_stop, never replan_success
                #      → partial pass (return 2) so CI can distinguish a
                #         genuine "no alternate path exists" from a healthy
                #         replan, but the smoke does not falsely claim full pass
                if outcome == "safety_replan_success" and changed_trajectory:
                    print(
                        "nav3d_local_replan_smoke passed outcome=safety_replan_success "
                        f"initial_poses={len(initial)} active_poses={len(active)} "
                        f"replanned_poses={len(replanned)} {attempt_summary}"
                    )
                    return 0
                if outcome in ("safety_emergency_stop", "safety_replan_emergency_stop"):
                    last_error = (
                        f"emergency_stop on attempt {attempt}/{args.attempts}; "
                        "trying restore + retry to give the bridge a chance "
                        "to find a real replan"
                    )
                    if attempt < args.attempts:
                        restore_initial_plan(
                            node,
                            start,
                            goal,
                            current,
                            args.min_trajectory_poses,
                            args.timeout,
                        )
                        continue
                    # Last attempt also emergency_stop — exit the attempt loop
                    # and let the post-loop branch report partial pass below.
                    print(
                        "nav3d_local_replan_smoke partial_pass outcome=safety_emergency_stop "
                        f"reason=no_alternate_path_after_{args.attempts}_attempts "
                        f"initial_poses={len(initial)} active_poses={len(active)} "
                        f"replanned_poses={len(replanned)} {attempt_summary}",
                        file=sys.stderr,
                    )
                    return 2
                # Any other unexpected outcome — restore + retry.
                last_error = f"unexpected safety_response outcome={outcome}"
                if attempt < args.attempts:
                    restore_initial_plan(
                        node,
                        start,
                        goal,
                        current,
                        args.min_trajectory_poses,
                        args.timeout,
                    )
                    continue

            if args.expected_outcome == "replan_success":
                if outcome == "safety_replan_success" and changed_trajectory:
                    print(
                        "nav3d_local_replan_smoke passed "
                        f"initial_poses={len(initial)} active_poses={len(active)} "
                        f"replanned_poses={len(replanned)} {attempt_summary}"
                    )
                    return 0
                last_error = (
                    "strict replan_success was not reached; "
                    f"last outcome={outcome} changed_trajectory={changed_trajectory}"
                )
                restore_initial_plan(
                    node,
                    start,
                    goal,
                    current,
                    args.min_trajectory_poses,
                    args.timeout,
                )
                continue

            if args.expected_outcome == "emergency_stop":
                if outcome in ("safety_emergency_stop", "safety_replan_emergency_stop"):
                    print(
                        "nav3d_local_replan_smoke passed "
                        f"initial_poses={len(initial)} active_poses={len(active)} "
                        f"{attempt_summary}"
                    )
                    return 0
                last_error = f"emergency_stop was not reached; last outcome={outcome}"
                restore_initial_plan(
                    node,
                    start,
                    goal,
                    current,
                    args.min_trajectory_poses,
                    args.timeout,
                )

        raise RuntimeError(
            f"{last_error}; attempts: " + " | ".join(attempt_summaries)
        )
    except Exception as error:
        print(f"nav3d_local_replan_smoke failed: {error}", file=sys.stderr)
        print("recent statuses:", file=sys.stderr)
        for status in node.observation.statuses[-12:]:
            print(f"  {status}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
