# Test file for nav3d_local_replan_smoke_lib helpers (W3)
"""Unit tests for nav3d_local_replan_smoke_lib pure helpers.

All tests are deterministic (no rclpy / ROS required).
"""
import math
import random
import sys
import os

import pytest

# Make the scripts package importable when running from the repo root.
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "scripts"),
)

from nav3d_local_replan_smoke_lib import (  # noqa: E402
    clamp_index,
    distance,
    drop_passed_anchors,
    filter_anchors_in_radius,
    index_at_distance,
    initial_travel_anchor_spawn_at,
    latest_snap_z_failure_status,
    make_obstacle_cluster,
    max_trajectory_delta,
    nearest_path_arc_length,
    outcome_name,
    path_length,
    path_endpoint_reaches_goal,
    path_xy_direction,
    point_along_path,
    prune_world_anchored_obstacles,
    remaining_distance_along_path,
    sample_anchors_along_path,
    should_retry_snap_z,
    should_publish_anchor_cloud,
    empty_trajectory_follow_action,
    spawn_world_anchored_centers,
    advance_along_path_from_current,
    visible_anchors_signature,
    world_anchored_cloud_points,
)

RNG = random.Random(7)


# ---------------------------------------------------------------------------
# distance
# ---------------------------------------------------------------------------


class TestDistance:
    def test_zero_same_point(self):
        assert distance((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) == pytest.approx(0.0)

    def test_axis_aligned_x(self):
        assert distance((0.0, 0.0, 0.0), (3.0, 0.0, 0.0)) == pytest.approx(3.0)

    def test_3_4_5_triangle_xy(self):
        # classic 3-4-5 right triangle in XY, z=0
        assert distance((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == pytest.approx(5.0)

    def test_3d_diagonal(self):
        # sqrt(1+1+1) = sqrt(3)
        assert distance((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)) == pytest.approx(math.sqrt(3))

    def test_symmetry(self):
        a = (1.0, 2.0, 3.0)
        b = (4.0, 5.0, 6.0)
        assert distance(a, b) == pytest.approx(distance(b, a))


# ---------------------------------------------------------------------------
# path_length
# ---------------------------------------------------------------------------


class TestPathLength:
    def test_empty(self):
        assert path_length([]) == pytest.approx(0.0)

    def test_single_point(self):
        assert path_length([(1.0, 2.0, 3.0)]) == pytest.approx(0.0)

    def test_two_points(self):
        assert path_length([(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]) == pytest.approx(5.0)

    def test_three_point_right_angle(self):
        # (0,0,0) -> (1,0,0) len 1; (1,0,0) -> (1,1,0) len 1; total 2
        pts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
        assert path_length(pts) == pytest.approx(2.0)

    def test_collinear(self):
        pts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
        assert path_length(pts) == pytest.approx(3.0)


# ---------------------------------------------------------------------------
# point_along_path
# ---------------------------------------------------------------------------


class TestPointAlongPath:
    PATH = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (3.0, 0.0, 0.0)]  # total length 3

    def test_target_zero_returns_first(self):
        result = point_along_path(self.PATH, 0.0)
        assert result == pytest.approx((0.0, 0.0, 0.0))

    def test_target_negative_returns_first(self):
        result = point_along_path(self.PATH, -1.0)
        assert result == pytest.approx((0.0, 0.0, 0.0))

    def test_target_within_first_segment(self):
        # 0.5 into [0,0,0]->[1,0,0] should be (0.5, 0, 0)
        result = point_along_path(self.PATH, 0.5)
        assert result == pytest.approx((0.5, 0.0, 0.0))

    def test_target_equals_total_length(self):
        result = point_along_path(self.PATH, 3.0)
        assert result == pytest.approx((3.0, 0.0, 0.0))

    def test_target_exceeds_total(self):
        result = point_along_path(self.PATH, 100.0)
        assert result == pytest.approx((3.0, 0.0, 0.0))

    def test_single_point_always_returns_it(self):
        result = point_along_path([(5.0, 6.0, 7.0)], 99.0)
        assert result == pytest.approx((5.0, 6.0, 7.0))

    def test_target_in_second_segment(self):
        # total length is 3; 1.5 is 0.5 into the second segment [1,0,0]->[3,0,0]
        result = point_along_path(self.PATH, 1.5)
        assert result == pytest.approx((1.5, 0.0, 0.0))

    def test_empty_raises(self):
        with pytest.raises(ValueError):
            point_along_path([], 1.0)


# ---------------------------------------------------------------------------
# trajectory projection / advancement
# ---------------------------------------------------------------------------


class TestTrajectoryProjection:
    PATH = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0)]

    def test_nearest_path_arc_projects_onto_segment_not_just_waypoints(self):
        arc = nearest_path_arc_length(self.PATH, (4.2, 1.0, 0.0))
        assert arc == pytest.approx(4.2)

    def test_remaining_distance_uses_current_projection(self):
        remaining = remaining_distance_along_path(self.PATH, (4.0, 0.0, 0.0))
        assert remaining == pytest.approx(16.0)

    def test_advance_starts_from_projected_current_point(self):
        point = advance_along_path_from_current(self.PATH, (4.0, 1.0, 0.0), 2.5)
        assert point == pytest.approx((6.5, 0.0, 0.0))

    def test_advance_clamps_to_path_end(self):
        point = advance_along_path_from_current(self.PATH, (9.0, 9.0, 0.0), 99.0)
        assert point == pytest.approx((10.0, 10.0, 0.0))

    def test_empty_path_raises_for_projection(self):
        with pytest.raises(ValueError):
            nearest_path_arc_length([], (0.0, 0.0, 0.0))

    def test_path_endpoint_reaches_goal_when_last_point_is_inside_tolerance(self):
        assert path_endpoint_reaches_goal(
            [(0.0, 0.0, 0.0), (8.1, 0.5, 0.3)],
            (8.16, 0.418, 0.351),
            1.0,
        ) is True

    def test_path_endpoint_does_not_reach_goal_for_short_local_segment(self):
        assert path_endpoint_reaches_goal(
            [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
            (8.0, 0.0, 0.0),
            1.0,
        ) is False


# ---------------------------------------------------------------------------
# path_xy_direction
# ---------------------------------------------------------------------------


class TestPathXyDirection:
    def test_single_point_fallback(self):
        assert path_xy_direction([(0.0, 0.0, 0.0)], 0.0) == pytest.approx((1.0, 0.0))

    def test_empty_fallback(self):
        assert path_xy_direction([], 0.0) == pytest.approx((1.0, 0.0))

    def test_east_direction(self):
        pts = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
        dx, dy = path_xy_direction(pts, 1.0)
        assert (dx, dy) == pytest.approx((1.0, 0.0))

    def test_north_direction(self):
        pts = [(0.0, 0.0, 0.0), (0.0, 4.0, 0.0)]
        dx, dy = path_xy_direction(pts, 2.0)
        assert (dx, dy) == pytest.approx((0.0, 1.0))

    def test_zero_length_last_segment_fallback(self):
        # Both points identical in XY — should fall back to (1, 0).
        pts = [(0.0, 0.0, 0.0), (0.0, 0.0, 1.0)]  # only Z differs
        dx, dy = path_xy_direction(pts, 0.5)
        assert (dx, dy) == pytest.approx((1.0, 0.0))

    def test_diagonal_normalised(self):
        pts = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
        dx, dy = path_xy_direction(pts, 0.5)
        expected = 1.0 / math.sqrt(2)
        assert dx == pytest.approx(expected)
        assert dy == pytest.approx(expected)


# ---------------------------------------------------------------------------
# prune_world_anchored_obstacles
# ---------------------------------------------------------------------------


class TestPruneWorldAnchoredObstacles:
    CURRENT = (0.0, 0.0, 0.0)

    def test_all_within_kept(self):
        centers = [(1.0, 0.0, 0.0), (0.5, 0.5, 0.0)]
        kept, dropped = prune_world_anchored_obstacles(centers, self.CURRENT, 2.0)
        assert len(kept) == 2
        assert dropped == 0

    def test_boundary_equal_kept(self):
        # distance exactly == drop_distance should be kept (<=)
        center = (3.0, 4.0, 0.0)  # distance = 5.0
        kept, dropped = prune_world_anchored_obstacles([center], self.CURRENT, 5.0)
        assert len(kept) == 1
        assert dropped == 0

    def test_just_over_boundary_dropped(self):
        center = (3.0, 4.0, 0.0)  # distance = 5.0
        kept, dropped = prune_world_anchored_obstacles([center], self.CURRENT, 4.999)
        assert len(kept) == 0
        assert dropped == 1

    def test_mixed(self):
        centers = [
            (1.0, 0.0, 0.0),   # dist 1 — kept
            (10.0, 0.0, 0.0),  # dist 10 — dropped
            (2.0, 0.0, 0.0),   # dist 2 — kept
            (20.0, 0.0, 0.0),  # dist 20 — dropped
        ]
        kept, dropped = prune_world_anchored_obstacles(centers, self.CURRENT, 5.0)
        assert len(kept) == 2
        assert dropped == 2

    def test_empty_input(self):
        kept, dropped = prune_world_anchored_obstacles([], self.CURRENT, 5.0)
        assert kept == []
        assert dropped == 0


# ---------------------------------------------------------------------------
# world_anchored_cloud_points
# ---------------------------------------------------------------------------


class TestWorldAnchoredCloudPoints:
    def test_one_center_seven_points(self):
        centers = [(0.0, 0.0, 0.0)]
        pts = world_anchored_cloud_points(centers, 0.2)
        assert len(pts) == 7

    def test_two_centers_fourteen_points(self):
        centers = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        pts = world_anchored_cloud_points(centers, 0.2)
        assert len(pts) == 14

    def test_empty_centers(self):
        assert world_anchored_cloud_points([], 0.2) == []


# ---------------------------------------------------------------------------
# spawn_world_anchored_centers
# ---------------------------------------------------------------------------


class TestSpawnWorldAnchoredCenters:
    TRAJ_SHORT = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]

    def test_empty_trajectory_returns_empty(self):
        result = spawn_world_anchored_centers(
            (0.0, 0.0, 0.0),
            [],
            front_distance=1.5,
            side_offset=0.6,
            jitter=0.0,
            count=3,
            rng=random.Random(7),
        )
        assert result == []

    def test_single_point_trajectory_returns_empty(self):
        result = spawn_world_anchored_centers(
            (0.0, 0.0, 0.0),
            [(0.0, 0.0, 0.0)],
            front_distance=1.5,
            side_offset=0.6,
            jitter=0.0,
            count=3,
            rng=random.Random(7),
        )
        assert result == []

    def test_count_zero_returns_empty(self):
        result = spawn_world_anchored_centers(
            (0.0, 0.0, 0.0),
            self.TRAJ_SHORT,
            front_distance=1.5,
            side_offset=0.6,
            jitter=0.0,
            count=0,
            rng=random.Random(7),
        )
        assert result == []

    def test_front_in_positive_x_direction(self):
        # Trajectory goes +x, robot at origin, jitter=0 -> front at (+front_distance, 0, 0)
        result = spawn_world_anchored_centers(
            (0.0, 0.0, 0.0),
            self.TRAJ_SHORT,
            front_distance=1.5,
            side_offset=0.6,
            jitter=0.0,
            count=1,
            rng=random.Random(7),
        )
        assert len(result) == 1
        fx, fy, fz = result[0]
        assert fx == pytest.approx(1.5, abs=1e-6)
        assert fy == pytest.approx(0.0, abs=1e-6)
        assert fz == pytest.approx(0.0, abs=1e-6)

    def test_left_right_in_y_direction(self):
        # Trajectory +x; left slot should be +y, right slot should be -y
        # front_distance=2, side_offset=1, jitter=0, count=3
        result = spawn_world_anchored_centers(
            (0.0, 0.0, 0.0),
            self.TRAJ_SHORT,
            front_distance=2.0,
            side_offset=1.0,
            jitter=0.0,
            count=3,
            rng=random.Random(7),
        )
        assert len(result) == 3
        front, left, right = result
        # front: x=2, y=0
        assert front[0] == pytest.approx(2.0, abs=1e-6)
        assert front[1] == pytest.approx(0.0, abs=1e-6)
        # left: x=front_distance*0.5=1, y=+side_offset=1
        assert left[0] == pytest.approx(1.0, abs=1e-6)
        assert left[1] == pytest.approx(1.0, abs=1e-6)
        # right: x=1, y=-side_offset=-1
        assert right[0] == pytest.approx(1.0, abs=1e-6)
        assert right[1] == pytest.approx(-1.0, abs=1e-6)

    def test_deterministic_with_seed(self):
        kwargs = dict(
            front_distance=1.5,
            side_offset=0.6,
            jitter=0.15,
            count=3,
        )
        r1 = spawn_world_anchored_centers(
            (0.0, 0.0, 0.0), self.TRAJ_SHORT, rng=random.Random(7), **kwargs
        )
        r2 = spawn_world_anchored_centers(
            (0.0, 0.0, 0.0), self.TRAJ_SHORT, rng=random.Random(7), **kwargs
        )
        assert r1 == r2


# ---------------------------------------------------------------------------
# make_obstacle_cluster
# ---------------------------------------------------------------------------


class TestMakeObstacleCluster:
    def test_seven_points(self):
        pts = make_obstacle_cluster((0.0, 0.0, 0.0), 0.2)
        assert len(pts) == 7

    def test_center_included(self):
        center = (1.0, 2.0, 3.0)
        pts = make_obstacle_cluster(center, 0.5)
        assert center in pts

    def test_offsets_correct(self):
        r = 0.5
        pts = make_obstacle_cluster((0.0, 0.0, 0.0), r)
        # All 6 axis points should be exactly r from center
        for pt in pts[1:]:
            assert distance((0.0, 0.0, 0.0), pt) == pytest.approx(r)

    def test_nonzero_center(self):
        c = (3.0, -1.0, 2.0)
        r = 1.0
        pts = make_obstacle_cluster(c, r)
        assert pts[0] == c
        assert pts[1] == (4.0, -1.0, 2.0)
        assert pts[2] == (2.0, -1.0, 2.0)


# ---------------------------------------------------------------------------
# index_at_distance
# ---------------------------------------------------------------------------


class TestIndexAtDistance:
    TRAJ = [(float(i), 0.0, 0.0) for i in range(10)]  # 9 unit segments

    def test_lookahead_zero_returns_fallback(self):
        idx = index_at_distance(self.TRAJ, 0, 5, 0.0)
        assert idx == 5

    def test_start_at_last_returns_fallback(self):
        # start_index >= len-1  => fallback
        idx = index_at_distance(self.TRAJ, 9, 3, 2.0)
        assert idx == 3

    def test_lookahead_mid_trajectory(self):
        # start at index 2, lookahead 3.0 => should hit index 5
        idx = index_at_distance(self.TRAJ, 2, 0, 3.0)
        assert idx == 5

    def test_lookahead_reaches_end(self):
        idx = index_at_distance(self.TRAJ, 0, 0, 100.0)
        assert idx == len(self.TRAJ) - 1

    def test_fallback_clamped(self):
        # fallback beyond end gets clamped
        idx = index_at_distance(self.TRAJ, 0, 999, 0.0)
        assert idx == len(self.TRAJ) - 1


# ---------------------------------------------------------------------------
# outcome_name
# ---------------------------------------------------------------------------


class TestOutcomeName:
    def test_safety_replan_success(self):
        assert outcome_name("safety_replan_success ts=123") == "safety_replan_success"

    def test_safety_emergency_stop(self):
        assert outcome_name("safety_emergency_stop ts=456") == "safety_emergency_stop"

    def test_safety_replan_emergency_stop(self):
        # Must match before safety_emergency_stop (more specific substring first)
        assert outcome_name("safety_replan_emergency_stop extra") == "safety_replan_emergency_stop"

    def test_safety_replan_failed(self):
        assert outcome_name("safety_replan_failed reason=x") == "safety_replan_failed"

    def test_plain_token(self):
        # No known keyword -> split on first space
        assert outcome_name("plan_success some detail") == "plan_success"

    def test_no_space_plain(self):
        assert outcome_name("idle") == "idle"


class TestShouldRetrySnapZ:
    def test_retries_no_path(self):
        assert should_retry_snap_z("plan_failed search_status=no_path") is True

    def test_retries_invalid_input(self):
        assert should_retry_snap_z("plan_failed search_status=invalid_input") is True

    def test_does_not_retry_other_failures(self):
        assert should_retry_snap_z("plan_failed search_status=iteration_limit") is False

    def test_finds_failure_status_before_later_heartbeat_statuses(self):
        statuses = [
            "plan_failed status=search_failed search_status=invalid_input",
            "occupied_grid_published width=1",
            "planning_occupied_markers_published voxels=10 local_voxels=0",
        ]
        assert latest_snap_z_failure_status(statuses) == statuses[0]

    def test_returns_empty_when_no_plan_failure_seen(self):
        assert latest_snap_z_failure_status(["occupied_grid_published width=1"]) == ""


class TestEmptyTrajectoryFollowAction:
    def test_new_empty_trajectory_far_from_goal_recovers(self):
        assert empty_trajectory_follow_action(
            has_new_trajectory_message=True,
            trajectory_pose_count=0,
            distance_to_goal=16.5,
            tolerance=1.0,
        ) == "recover"

    def test_new_empty_trajectory_at_goal_completes(self):
        assert empty_trajectory_follow_action(
            has_new_trajectory_message=True,
            trajectory_pose_count=0,
            distance_to_goal=0.8,
            tolerance=1.0,
        ) == "complete"

    def test_nonempty_trajectory_is_ignored(self):
        assert empty_trajectory_follow_action(
            has_new_trajectory_message=True,
            trajectory_pose_count=12,
            distance_to_goal=16.5,
            tolerance=1.0,
        ) == "ignore"

    def test_without_new_message_is_ignored(self):
        assert empty_trajectory_follow_action(
            has_new_trajectory_message=False,
            trajectory_pose_count=0,
            distance_to_goal=16.5,
            tolerance=1.0,
        ) == "ignore"


class TestInitialTravelAnchorSpawnAt:
    def test_first_spawn_happens_after_one_step_of_robot_travel(self):
        assert initial_travel_anchor_spawn_at(5.0) == pytest.approx(5.0)

    def test_rejects_nonpositive_step(self):
        with pytest.raises(ValueError):
            initial_travel_anchor_spawn_at(0.0)


class TestShouldPublishAnchorCloud:
    def test_changed_signature_publishes_empty_clear(self):
        assert should_publish_anchor_cloud(
            visible_signature=(),
            last_visible_signature=((1.0, 2.0, 3.0),),
            heartbeat_due=False,
            visible_count=0,
        ) is True

    def test_unchanged_empty_signature_does_not_heartbeat(self):
        assert should_publish_anchor_cloud(
            visible_signature=(),
            last_visible_signature=(),
            heartbeat_due=True,
            visible_count=0,
        ) is False

    def test_unchanged_nonempty_signature_heartbeats(self):
        assert should_publish_anchor_cloud(
            visible_signature=((1.0, 2.0, 3.0),),
            last_visible_signature=((1.0, 2.0, 3.0),),
            heartbeat_due=True,
            visible_count=1,
        ) is True

    def test_unchanged_nonempty_signature_waits_before_heartbeat(self):
        assert should_publish_anchor_cloud(
            visible_signature=((1.0, 2.0, 3.0),),
            last_visible_signature=((1.0, 2.0, 3.0),),
            heartbeat_due=False,
            visible_count=1,
        ) is False


# ---------------------------------------------------------------------------
# Path-anchored sticky helpers (W6)
# ---------------------------------------------------------------------------


class TestSampleAnchorsAlongPath:
    def test_4_anchors_on_20m_path(self):
        rng = random.Random(7)
        anchors = sample_anchors_along_path(
            [(0, 0, 0), (20, 0, 0)],
            5.0,
            side_offset=0.0,
            jitter=0.0,
            rng=rng,
        )
        assert len(anchors) == 4
        xs = [a[0] for a in anchors]
        assert xs == pytest.approx([5.0, 10.0, 15.0, 20.0])

    def test_short_path_returns_empty(self):
        rng = random.Random(7)
        anchors = sample_anchors_along_path(
            [(0, 0, 0), (2, 0, 0)],
            5.0,
            side_offset=0.5,
            jitter=0.0,
            rng=rng,
        )
        assert anchors == []

    def test_alternating_side_offset(self):
        rng = random.Random(7)
        anchors = sample_anchors_along_path(
            [(0, 0, 0), (20, 0, 0)],
            5.0,
            side_offset=1.0,
            jitter=0.0,
            rng=rng,
        )
        # Even-index anchors go left (+y), odd go right (-y)
        assert anchors[0][1] == pytest.approx(1.0)
        assert anchors[1][1] == pytest.approx(-1.0)
        assert anchors[2][1] == pytest.approx(1.0)
        assert anchors[3][1] == pytest.approx(-1.0)


class TestFilterAnchorsInRadius:
    def test_radius_filter_basic(self):
        anchors = [(0, 0, 0), (3, 0, 0), (6, 0, 0), (9, 0, 0), (12, 0, 0)]
        visible = filter_anchors_in_radius(anchors, (5, 0, 0), 3.0)
        # |5-x| <= 3 → x in {3,6,9 wait no — only |5-3|=2,|5-6|=1,|5-9|=4 too far}
        # exact: x=3 (d=2), x=6 (d=1) — both within; x=9 (d=4) > 3 → out
        # So visible {3, 6}? Actually we also include x=0? d=5 > 3 → no
        # x=12: d=7 → no
        assert len(visible) == 2
        xs = sorted(a[0] for a in visible)
        assert xs == [3, 6]

    def test_boundary_inclusive(self):
        anchors = [(3, 0, 0), (5, 0, 0), (8, 0, 0)]
        visible = filter_anchors_in_radius(anchors, (5, 0, 0), 3.0)
        # All three: |5-3|=2, |5-5|=0, |5-8|=3 (==radius → keep)
        assert len(visible) == 3

    def test_zero_radius_returns_empty(self):
        anchors = [(0, 0, 0), (1, 0, 0)]
        assert filter_anchors_in_radius(anchors, (0, 0, 0), 0.0) == []


class TestDropPassedAnchors:
    def test_drop_past_threshold(self):
        # Path 0→10 along x; anchor at 3, robot at 8 → 5m past, drop_past=4 → drop
        path = [(i, 0, 0) for i in range(11)]  # waypoints at integer xs
        kept, dropped = drop_passed_anchors(
            [(3, 0, 0)], path, (8, 0, 0), drop_past_distance_m=4.0,
        )
        assert kept == []
        assert dropped == 1

    def test_keep_within_threshold(self):
        path = [(i, 0, 0) for i in range(11)]
        kept, dropped = drop_passed_anchors(
            [(3, 0, 0)], path, (6, 0, 0), drop_past_distance_m=4.0,
        )
        # 3m past, less than threshold → keep
        assert len(kept) == 1
        assert dropped == 0

    def test_empty_anchors(self):
        path = [(0, 0, 0), (10, 0, 0)]
        kept, dropped = drop_passed_anchors([], path, (5, 0, 0), 4.0)
        assert kept == []
        assert dropped == 0

    def test_anchor_ahead_kept(self):
        # Robot at 3, anchor at 8 (ahead) — never dropped
        path = [(i, 0, 0) for i in range(11)]
        kept, dropped = drop_passed_anchors(
            [(8, 0, 0)], path, (3, 0, 0), drop_past_distance_m=2.0,
        )
        assert len(kept) == 1
        assert dropped == 0


# ---------------------------------------------------------------------------
# visible_anchors_signature (W-B)
# ---------------------------------------------------------------------------


class TestVisibleAnchorsSignature:
    def test_empty_returns_empty_tuple(self):
        assert visible_anchors_signature([]) == ()

    def test_order_independent(self):
        a = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        b = [(4.0, 5.0, 6.0), (1.0, 2.0, 3.0)]
        assert visible_anchors_signature(a) == visible_anchors_signature(b)

    def test_quantises_below_decimals(self):
        # Two anchors that differ only at the 4th decimal place quantise equal at 2dp.
        a = [(1.0, 2.0, 3.0)]
        b = [(1.0001, 2.0001, 3.0001)]
        assert visible_anchors_signature(a) == visible_anchors_signature(b)

    def test_distinguishes_real_change(self):
        a = [(1.0, 2.0, 3.0)]
        b = [(1.5, 2.0, 3.0)]
        assert visible_anchors_signature(a) != visible_anchors_signature(b)

    def test_change_when_anchor_dropped(self):
        a = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        b = [(1.0, 2.0, 3.0)]
        assert visible_anchors_signature(a) != visible_anchors_signature(b)

    def test_signature_is_hashable(self):
        sig = visible_anchors_signature([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
        # Tuple of tuples — must be usable as a dict key.
        store = {sig: True}
        assert sig in store
