"""Pure-function helpers extracted from nav3d_local_replan_smoke.py.

No rclpy or ROS imports — all functions are deterministic given their inputs
and safe to unit-test without a running ROS environment.
"""
import math
import random
from typing import List, Sequence, Tuple

# ---------------------------------------------------------------------------
# Type alias
# ---------------------------------------------------------------------------

Point = Tuple[float, float, float]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def distance(left: Point, right: Point) -> float:
    """Euclidean distance between two 3-D points."""
    return math.sqrt(
        (left[0] - right[0]) ** 2 +
        (left[1] - right[1]) ** 2 +
        (left[2] - right[2]) ** 2
    )


def max_trajectory_delta(left: Sequence[Point], right: Sequence[Point]) -> float:
    """Maximum per-index distance between two trajectories (matched by index)."""
    count = min(len(left), len(right))
    if count == 0:
        return 0.0
    return max(distance(left[index], right[index]) for index in range(count))


def make_obstacle_cluster(center: Point, radius: float) -> List[Point]:
    """Return a 7-point cluster: center + one point on each +-axis at `radius`."""
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


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------


def path_length(points: Sequence[Point]) -> float:
    """Total arc length of a polyline through `points`."""
    if len(points) < 2:
        return 0.0
    return sum(distance(points[index - 1], points[index]) for index in range(1, len(points)))


def point_along_path(points: Sequence[Point], target_distance: float) -> Point:
    """Interpolated point on the polyline at `target_distance` from the start.

    Returns points[0] when target_distance <= 0 or the path has one point.
    Returns points[-1] when target_distance exceeds the total path length.
    """
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


def nearest_path_arc_length(points: Sequence[Point], current: Point) -> float:
    """Arc length of the closest projection of `current` onto a polyline.

    This uses segment projection instead of nearest waypoint selection. The
    follow smoke needs this when the bridge republishes a different trajectory:
    a nearest discrete point can jump backward or skip a detour on sparse or
    freshly-replanned paths.
    """
    if not points:
        raise ValueError("path is empty")
    if len(points) == 1:
        return 0.0

    best_arc = 0.0
    best_distance_sq = float("inf")
    traveled = 0.0
    for index in range(1, len(points)):
        start = points[index - 1]
        end = points[index]
        vx = end[0] - start[0]
        vy = end[1] - start[1]
        vz = end[2] - start[2]
        segment_length_sq = vx * vx + vy * vy + vz * vz
        if segment_length_sq <= 1e-18:
            continue
        wx = current[0] - start[0]
        wy = current[1] - start[1]
        wz = current[2] - start[2]
        ratio = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / segment_length_sq))
        px = start[0] + vx * ratio
        py = start[1] + vy * ratio
        pz = start[2] + vz * ratio
        dx = current[0] - px
        dy = current[1] - py
        dz = current[2] - pz
        distance_sq = dx * dx + dy * dy + dz * dz
        segment_length = math.sqrt(segment_length_sq)
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_arc = traveled + segment_length * ratio
        traveled += segment_length
    return best_arc


def remaining_distance_along_path(points: Sequence[Point], current: Point) -> float:
    """Remaining path length from current's projection to the end."""
    if not points:
        return 0.0
    return max(0.0, path_length(points) - nearest_path_arc_length(points, current))


def path_endpoint_reaches_goal(
    points: Sequence[Point],
    goal: Point,
    tolerance: float,
) -> bool:
    """Return True when the path endpoint is close enough to the requested goal."""
    return bool(points) and distance(points[-1], goal) <= tolerance


def advance_along_path_from_current(
    points: Sequence[Point],
    current: Point,
    advance_distance: float,
) -> Point:
    """Advance from current's projected path position by `advance_distance`."""
    if not points:
        raise ValueError("path is empty")
    start_arc = nearest_path_arc_length(points, current)
    return point_along_path(points, start_arc + max(0.0, advance_distance))


def path_xy_direction(points: Sequence[Point], target_distance: float) -> Tuple[float, float]:
    """Normalised XY heading at `target_distance` along the polyline.

    Falls back to (1, 0) when the path has fewer than 2 points or the last
    segment has zero XY length.
    """
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


# ---------------------------------------------------------------------------
# Index helpers
# ---------------------------------------------------------------------------


def clamp_index(index: int, size: int) -> int:
    """Clamp `index` to [0, size-1]."""
    return min(max(0, index), max(0, size - 1))


def index_at_distance(
    trajectory: Sequence[Point],
    start_index: int,
    fallback_index: int,
    lookahead_distance: float,
) -> int:
    """Return the index in `trajectory` that is >= `lookahead_distance` ahead of start_index.

    Falls back to clamp_index(fallback_index, len(trajectory)) when
    lookahead_distance <= 0 or start_index is already at the last point.
    """
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


# ---------------------------------------------------------------------------
# World-anchored sticky obstacle helpers
# ---------------------------------------------------------------------------


def prune_world_anchored_obstacles(
    centers: List[Point],
    current: Point,
    drop_distance: float,
) -> Tuple[List[Point], int]:
    """Remove sticky obstacle centers farther than `drop_distance` from `current`.

    Returns (kept_centers, dropped_count).
    """
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
    """Expand each sticky center into a 7-point obstacle cluster."""
    points: List[Point] = []
    for center in centers:
        points.extend(make_obstacle_cluster(center, radius))
    return points


def make_dense_obstacle_cluster(
    center: Point,
    half_extent: float,
    voxel_resolution: float,
) -> List[Point]:
    """Solid cube of points around `center`, sized to actually block a planner.

    The default 7-point cross from `make_obstacle_cluster` only marks 7 voxels
    in a + pattern. With map.resolution matching the obstacle radius, the
    planner's robot footprint (e.g. ground_robot_radius 0.25 m) finds diagonal
    gaps and the trajectory squeezes through. This generator instead emits a
    dense (2n+1)^3 voxel cube where n = round(half_extent / voxel_resolution),
    producing a continuous occupied region the planner cannot slip through.

    Designed for v3.8 travel mode: each spawned anchor expands into ~125-343
    points covering ~1.0 m^3, large enough to force a real detour but small
    enough to keep the bridge's local_pointcloud occupancy update under a
    millisecond per anchor at follow_rate 20 Hz.
    """
    if half_extent <= 0.0 or voxel_resolution <= 0.0:
        return [center]
    n = max(1, int(round(half_extent / voxel_resolution)))
    points: List[Point] = []
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            for k in range(-n, n + 1):
                points.append((
                    center[0] + i * voxel_resolution,
                    center[1] + j * voxel_resolution,
                    center[2] + k * voxel_resolution,
                ))
    return points


def dense_world_anchored_cloud_points(
    centers: Sequence[Point],
    half_extent: float,
    voxel_resolution: float,
) -> List[Point]:
    """Like `world_anchored_cloud_points` but uses `make_dense_obstacle_cluster`."""
    points: List[Point] = []
    for center in centers:
        points.extend(make_dense_obstacle_cluster(center, half_extent, voxel_resolution))
    return points


def spawn_world_anchored_centers(
    current: Point,
    trajectory: Sequence[Point],
    *,
    front_distance: float,
    side_offset: float,
    jitter: float,
    count: int,
    rng: random.Random,
) -> List[Point]:
    """Generate a fresh batch of obstacle cluster centers around `current`.

    Parameters
    ----------
    current:
        Current robot position in world coordinates.
    trajectory:
        Active path (must have >= 2 points; otherwise returns []).
    front_distance:
        Distance ahead of the robot (along the path tangent) to place the
        front obstacle.
    side_offset:
        Perpendicular offset from the forward tangent for left/right obstacles.
    jitter:
        Max random displacement applied to each chosen point so consecutive
        spawns do not perfectly co-locate.
    count:
        How many cluster centers to return (rotates front/left/right slots).
    rng:
        Seeded random.Random instance for deterministic output.
    """
    if count <= 0 or len(trajectory) < 2:
        return []

    # Find the nearest waypoint to `current` and derive the local XY tangent.
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
    lx, ly = -fy, fx   # left perpendicular in XY
    rx, ry = fy, -fx   # right perpendicular in XY

    front = (
        current[0] + fx * front_distance,
        current[1] + fy * front_distance,
        current[2],
    )
    left = (
        current[0] + fx * front_distance * 0.5 + lx * side_offset,
        current[1] + fy * front_distance * 0.5 + ly * side_offset,
        current[2],
    )
    right = (
        current[0] + fx * front_distance * 0.5 + rx * side_offset,
        current[1] + fy * front_distance * 0.5 + ry * side_offset,
        current[2],
    )

    template = [front, left, right]
    chosen: List[Point] = []
    j = jitter
    for i in range(count):
        base = template[i % 3]
        chosen.append((
            base[0] + rng.uniform(-j, j),
            base[1] + rng.uniform(-j, j),
            base[2] + rng.uniform(-j * 0.5, j * 0.5),
        ))
    return chosen


# ---------------------------------------------------------------------------
# Outcome helpers
# ---------------------------------------------------------------------------


def outcome_name(status: str) -> str:
    """Normalise a raw status string to a short outcome token."""
    if "safety_replan_success" in status:
        return "safety_replan_success"
    if "safety_replan_failed" in status:
        return "safety_replan_failed"
    if "safety_replan_emergency_stop" in status:
        return "safety_replan_emergency_stop"
    if "safety_emergency_stop" in status:
        return "safety_emergency_stop"
    return status.split(" ", 1)[0]


def should_retry_snap_z(status: str) -> bool:
    """Return True when an initial plan failure may be caused by bad z."""
    return (
        "search_status=no_path" in status or
        "search_status=invalid_input" in status
    )


def latest_snap_z_failure_status(statuses: Sequence[str]) -> str:
    """Return the newest plan failure status relevant to snap-z retry."""
    for status in reversed(statuses):
        if "plan_failed" in status and "search_status=" in status:
            return status
    return ""


def empty_trajectory_follow_action(
    *,
    has_new_trajectory_message: bool,
    trajectory_pose_count: int,
    distance_to_goal: float,
    tolerance: float,
) -> str:
    """Classify what the follow loop should do with an empty trajectory message."""
    if not has_new_trajectory_message or trajectory_pose_count > 0:
        return "ignore"
    if distance_to_goal <= tolerance:
        return "complete"
    return "recover"


def initial_travel_anchor_spawn_at(step_m: float) -> float:
    """Return the travel distance threshold for the first travel-mode anchor."""
    if step_m <= 0.0:
        raise ValueError("travel anchor step must be positive")
    return step_m


def should_publish_anchor_cloud(
    *,
    visible_signature: Tuple,
    last_visible_signature: Tuple,
    heartbeat_due: bool,
    visible_count: int,
) -> bool:
    """Return True when the smoke should publish the current visible anchor cloud."""
    if visible_signature != last_visible_signature:
        return True
    return heartbeat_due and visible_count > 0


# ---------------------------------------------------------------------------
# Path-anchored sticky obstacle helpers (W6)
#
# Operator semantics: once a global trajectory is planned, sample fixed sticky
# anchor centers every `interval_m` meters along that path (frozen in world
# coordinates). Each frame, only publish anchors within `radius_m` of the robot
# — anchors outside the radius stay dormant and re-appear when the robot
# approaches. Once the robot has traveled `drop_past_m` meters past an anchor
# along the path, the anchor is permanently dropped.
#
# This differs from `spawn_world_anchored_centers` (rolling/time-based) in
# being a one-shot sample at planning time with no further spawning.
# ---------------------------------------------------------------------------


def sample_anchors_along_path(
    trajectory: Sequence[Point],
    interval_m: float,
    *,
    side_offset: float,
    jitter: float,
    rng: random.Random,
) -> List[Point]:
    """Sample sticky anchor centers every `interval_m` meters along `trajectory`.

    Each anchor sits perpendicular to the local path tangent (alternating
    left/right by sample index), with optional XY jitter (±jitter) and Z
    jitter (±jitter*0.5). Returns absolute world coordinates frozen at the
    moment of sampling — the operator's "global path generation moment".

    The first anchor lands at `interval_m`; the last anchor is at the largest
    multiple of `interval_m` not exceeding the path length.
    Returns [] when `interval_m <= 0`, the path is shorter than `interval_m`,
    or the trajectory has fewer than 2 points.
    """
    if interval_m <= 0.0 or len(trajectory) < 2:
        return []
    total_length = path_length(trajectory)
    if total_length < interval_m:
        return []

    anchors: List[Point] = []
    sample_distance = interval_m
    sample_index = 0
    while sample_distance <= total_length + 1e-9:
        base = point_along_path(trajectory, sample_distance)
        fx, fy = path_xy_direction(trajectory, sample_distance)
        # Perpendicular: alternate left (+) and right (-) by sample index
        side_sign = 1.0 if (sample_index % 2 == 0) else -1.0
        lx, ly = -fy * side_sign, fx * side_sign
        anchors.append((
            base[0] + lx * side_offset + rng.uniform(-jitter, jitter),
            base[1] + ly * side_offset + rng.uniform(-jitter, jitter),
            base[2] + rng.uniform(-jitter * 0.5, jitter * 0.5),
        ))
        sample_index += 1
        sample_distance += interval_m
    return anchors


def filter_anchors_in_radius(
    anchors: Sequence[Point],
    current: Point,
    radius_m: float,
) -> List[Point]:
    """Return anchors with Euclidean distance to `current` <= `radius_m` (inclusive)."""
    if radius_m <= 0.0:
        return []
    threshold_sq = radius_m * radius_m
    visible: List[Point] = []
    for anchor in anchors:
        dx = anchor[0] - current[0]
        dy = anchor[1] - current[1]
        dz = anchor[2] - current[2]
        if dx * dx + dy * dy + dz * dz <= threshold_sq + 1e-9:
            visible.append(anchor)
    return visible


def visible_anchors_signature(visible: Sequence[Point], decimals: int = 2) -> Tuple:
    """Return a hashable, order-independent signature for the visible anchor set.

    Used by the smoke loop to skip republishing the local pointcloud when the
    visible set has not changed at quantised precision. Republishing is what
    triggers bridge-side `updateLocalObservationWindow` + safety eval, so
    skipping equal frames cuts ~10Hz redundant work while the robot drifts
    inside one path-anchor's coverage cell.
    """
    if not visible:
        return ()
    factor = 10 ** max(0, decimals)
    quantised = tuple(
        sorted(
            (
                round(p[0] * factor) / factor,
                round(p[1] * factor) / factor,
                round(p[2] * factor) / factor,
            )
            for p in visible
        )
    )
    return quantised


def _nearest_waypoint_index(trajectory: Sequence[Point], target: Point) -> int:
    """Index of the trajectory waypoint nearest to `target` (Euclidean)."""
    if not trajectory:
        return 0
    best_idx = 0
    best_d2 = float("inf")
    for i, p in enumerate(trajectory):
        dx = p[0] - target[0]
        dy = p[1] - target[1]
        dz = p[2] - target[2]
        d2 = dx * dx + dy * dy + dz * dz
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
    return best_idx


def drop_passed_anchors(
    anchors: Sequence[Point],
    trajectory: Sequence[Point],
    current: Point,
    drop_past_distance_m: float,
) -> Tuple[List[Point], int]:
    """Drop anchors the robot has traveled past along the trajectory.

    For each anchor, find the nearest trajectory waypoint to it; do the same
    for `current`. If the robot's projected trajectory index is *past* the
    anchor's projected index, and the arc-length separation between those two
    indices exceeds `drop_past_distance_m`, the anchor is dropped.

    Returns (kept_anchors, dropped_count).
    """
    if not anchors or len(trajectory) < 2 or drop_past_distance_m <= 0.0:
        return list(anchors), 0

    robot_idx = _nearest_waypoint_index(trajectory, current)
    kept: List[Point] = []
    dropped = 0
    for anchor in anchors:
        anchor_idx = _nearest_waypoint_index(trajectory, anchor)
        if robot_idx <= anchor_idx:
            kept.append(anchor)
            continue
        # Arc length between anchor_idx and robot_idx
        arc = 0.0
        for i in range(anchor_idx + 1, robot_idx + 1):
            arc += distance(trajectory[i - 1], trajectory[i])
        if arc > drop_past_distance_m:
            dropped += 1
        else:
            kept.append(anchor)
    return kept, dropped
