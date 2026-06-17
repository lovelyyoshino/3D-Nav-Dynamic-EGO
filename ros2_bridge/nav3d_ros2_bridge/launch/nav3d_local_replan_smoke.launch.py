from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def _smoke_process(context: LaunchContext, *args):
    pkg_share = FindPackageShare("nav3d_ros2_bridge").perform(context)

    pcd_path = LaunchConfiguration("pcd_path").perform(context)
    start_xyz = LaunchConfiguration("start_xyz").perform(context)
    goal_xyz = LaunchConfiguration("goal_xyz").perform(context)
    follow_speed = LaunchConfiguration("follow_speed").perform(context)
    snap_z_fallback = LaunchConfiguration("snap_z_fallback").perform(context)
    anchor_mode = LaunchConfiguration("anchor_mode").perform(context)
    anchor_spacing_m = LaunchConfiguration("anchor_spacing_m").perform(context)
    anchor_publish_radius_m = LaunchConfiguration("anchor_publish_radius_m").perform(context)
    anchor_drop_past_m = LaunchConfiguration("anchor_drop_past_m").perform(context)
    anchor_block_path = LaunchConfiguration("anchor_block_path").perform(context)
    anchor_travel_step_m = LaunchConfiguration("anchor_travel_step_m").perform(context)
    anchor_travel_spawn_distance_m = LaunchConfiguration("anchor_travel_spawn_distance_m").perform(context)
    anchor_travel_half_extent_m = LaunchConfiguration("anchor_travel_half_extent_m").perform(context)
    anchor_travel_voxel_resolution_m = LaunchConfiguration("anchor_travel_voxel_resolution_m").perform(context)
    anchor_publish_interval_s = LaunchConfiguration("anchor_publish_interval_s").perform(context)
    follow_timeout = LaunchConfiguration("follow_timeout").perform(context)
    follow_goal_tolerance = LaunchConfiguration("follow_goal_tolerance").perform(context)
    idle_timeout = LaunchConfiguration("idle_timeout").perform(context)

    start_parts = start_xyz.split()
    goal_parts = goal_xyz.split()

    # v3.8: install(PROGRAMS …) lands the smoke script under lib/<pkg>/, not
    # share/<pkg>/scripts/. The previous launch invented a share/scripts/ path
    # that never existed → "python3: can't open file …/share/.../scripts/…".
    # FindPackageShare gives us share/<pkg>; the executable sibling lives in
    # the parallel lib tree.
    pkg_lib = pkg_share.replace("/share/", "/lib/", 1)
    script_path = pkg_lib + "/nav3d_local_replan_smoke.py"

    cmd = [
        "python3",
        script_path,
        "--follow-trajectory",
        "--bridge-watchdog",
        "--start", start_parts[0], start_parts[1], start_parts[2],
        "--goal", goal_parts[0], goal_parts[1], goal_parts[2],
        "--anchor-mode", anchor_mode,
        "--anchor-spacing-m", anchor_spacing_m,
        "--anchor-publish-radius-m", anchor_publish_radius_m,
        "--anchor-drop-past-m", anchor_drop_past_m,
        "--anchor-travel-step-m", anchor_travel_step_m,
        "--anchor-travel-spawn-distance-m", anchor_travel_spawn_distance_m,
        "--anchor-travel-half-extent-m", anchor_travel_half_extent_m,
        "--anchor-travel-voxel-resolution-m", anchor_travel_voxel_resolution_m,
        "--anchor-publish-interval-s", anchor_publish_interval_s,
        "--follow-speed", follow_speed,
        "--snap-z-fallback", snap_z_fallback,
        "--follow-timeout", follow_timeout,
        "--follow-goal-tolerance", follow_goal_tolerance,
        "--idle-timeout", idle_timeout,
    ]
    if anchor_block_path.lower() in ("true", "1", "yes"):
        cmd.append("--anchor-block-path")

    return [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
        )
    ]


def generate_launch_description():
    pkg_share = FindPackageShare("nav3d_ros2_bridge")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pcd_path",
                default_value="",
                description="Absolute path to the PCD file (required).",
            ),
            DeclareLaunchArgument(
                "start_xyz",
                default_value="-12 3.19 0.351",
                description="Start pose as 'X Y Z' (space-separated).",
            ),
            DeclareLaunchArgument(
                "goal_xyz",
                default_value="8.16 0.418 0.351",
                description="Goal pose as 'X Y Z' (space-separated).",
            ),
            # v3.8: path-anchored mode is the operator's spec — every 5 m
            # along the global plan, only publish anchors within 10 m of the
            # robot, drop them once 10 m past. Replaces the rolling sticky
            # spawn flags from v3.5.
            DeclareLaunchArgument(
                "anchor_mode",
                default_value="travel",
                description="travel = (default, operator's spec) every N m of robot travel spawn one obstacle 3m ahead; path = legacy seed-once-per-plan; rolling = legacy time-based.",
            ),
            DeclareLaunchArgument(
                "anchor_travel_step_m",
                default_value="5.0",
                description="travel mode: spawn a fresh anchor every N meters of robot travel.",
            ),
            DeclareLaunchArgument(
                "anchor_travel_spawn_distance_m",
                default_value="3.0",
                description="travel mode: place new anchor this many meters ahead of the robot's current pose.",
            ),
            DeclareLaunchArgument(
                "anchor_travel_half_extent_m",
                default_value="0.25",
                description="travel mode: half-edge of the compact dense voxel-cube obstacle.",
            ),
            DeclareLaunchArgument(
                "anchor_travel_voxel_resolution_m",
                default_value="0.2",
                description="travel mode: voxel grid spacing for the dense obstacle cube (match map.resolution).",
            ),
            DeclareLaunchArgument(
                "anchor_publish_interval_s",
                default_value="1.0",
                description="travel/path mode: heartbeat republish interval for the local obstacle pointcloud.",
            ),
            DeclareLaunchArgument(
                "anchor_spacing_m",
                default_value="5.0",
                description="Path-anchored: meters between consecutive anchors along the global plan.",
            ),
            DeclareLaunchArgument(
                "anchor_publish_radius_m",
                default_value="10.0",
                description="Path-anchored: only publish anchors within this radius of the robot.",
            ),
            DeclareLaunchArgument(
                "anchor_drop_past_m",
                default_value="10.0",
                description="Path-anchored: drop anchors once the robot has traveled this many meters past them along the path.",
            ),
            DeclareLaunchArgument(
                "anchor_block_path",
                default_value="false",
                description="If true, anchors land ON the trajectory (side_offset=0) so safety_replan + raw-A* fast path is exercised.",
            ),
            DeclareLaunchArgument(
                "follow_speed",
                default_value="0.5",
                description="Simulated robot follow speed in m/s.",
            ),
            DeclareLaunchArgument(
                "follow_timeout",
                default_value="1320.0",
                description="Maximum seconds for the follow phase before TimeoutError.",
            ),
            DeclareLaunchArgument(
                "follow_goal_tolerance",
                default_value="1.0",
                description="Distance to goal (m) at which follow_complete_by_proximity fires.",
            ),
            DeclareLaunchArgument(
                "idle_timeout",
                default_value="400.0",
                description="Abort follow phase if neither trajectories nor local marker counts change for this many seconds.",
            ),
            DeclareLaunchArgument(
                "snap_z_fallback",
                default_value="1.0",
                description="Z fallback when start/goal z has no free cell (building2_9 corridor sits at z≈1.0).",
            ),
            # v3.8: forward all bridge args so the smoke launch composes the
            # exact same launch configuration the operator is testing manually.
            # Defaults match the values the operator uses interactively:
            # pcd_loader=builtin, planning_mode=3d, planning_traversability=ground,
            # planning_use_octomap_map=false, search_algorithm=astar, resolution=0.2,
            # rosbridge=true, rviz=true.
            DeclareLaunchArgument("pcd_loader", default_value="builtin"),
            DeclareLaunchArgument("planning_mode", default_value="3d"),
            DeclareLaunchArgument("planning_traversability", default_value="ground"),
            DeclareLaunchArgument("planning_use_octomap_map", default_value="false"),
            DeclareLaunchArgument("search_algorithm", default_value="astar"),
            DeclareLaunchArgument("resolution", default_value="0.2"),
            DeclareLaunchArgument("rosbridge", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, "launch", "nav3d_bridge.launch.py"])
                ),
                launch_arguments={
                    "pcd_path": LaunchConfiguration("pcd_path"),
                    "pcd_loader": LaunchConfiguration("pcd_loader"),
                    "planning_mode": LaunchConfiguration("planning_mode"),
                    "planning_traversability": LaunchConfiguration("planning_traversability"),
                    "planning_use_octomap_map": LaunchConfiguration("planning_use_octomap_map"),
                    "search_algorithm": LaunchConfiguration("search_algorithm"),
                    "resolution": LaunchConfiguration("resolution"),
                    "rosbridge": LaunchConfiguration("rosbridge"),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
            OpaqueFunction(function=_smoke_process),
        ]
    )
