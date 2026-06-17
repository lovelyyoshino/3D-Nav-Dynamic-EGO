from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_file(*parts):
    return PathJoinSubstitution([FindPackageShare("nav3d_ros2_bridge"), *parts])


def generate_launch_description():
    pcd_path = LaunchConfiguration("pcd_path")
    frame_id = LaunchConfiguration("frame_id")
    planning_mode = LaunchConfiguration("planning_mode")
    planning_traversability = LaunchConfiguration("planning_traversability")
    search_algorithm = LaunchConfiguration("search_algorithm")
    planning_use_octomap_map = LaunchConfiguration("planning_use_octomap_map")
    pcd_loader = LaunchConfiguration("pcd_loader")
    resolution = LaunchConfiguration("resolution")
    occupancy_grid_min_z = LaunchConfiguration("occupancy_grid_min_z")
    occupancy_grid_max_z = LaunchConfiguration("occupancy_grid_max_z")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    config_path = LaunchConfiguration("config_path")
    rviz_config = LaunchConfiguration("rviz_config")

    bridge_overrides = {
        "map.pcd_path": pcd_path,
        "frame_id": frame_id,
        "planning.mode": planning_mode,
        "planning.traversability": planning_traversability,
        "planning.search_algorithm": search_algorithm,
        "planning.use_octomap_map": ParameterValue(
            planning_use_octomap_map,
            value_type=bool,
        ),
        "map.pcd_loader": pcd_loader,
        "map.resolution": ParameterValue(resolution, value_type=float),
        "map.occupancy_grid_min_z": ParameterValue(
            occupancy_grid_min_z,
            value_type=float,
        ),
        "map.occupancy_grid_max_z": ParameterValue(
            occupancy_grid_max_z,
            value_type=float,
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("pcd_path", default_value=""),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument(
                "config_path",
                default_value=package_file("config", "nav3d_bridge.yaml"),
                description="YAML file for stable bridge/controller/safety/local-grid defaults.",
            ),
            DeclareLaunchArgument("planning_mode", default_value="3d"),
            DeclareLaunchArgument(
                "planning_traversability",
                default_value="uav",
                description="uav uses free-space 3D planning; ground requires supported ground cells.",
            ),
            DeclareLaunchArgument("search_algorithm", default_value="astar"),
            DeclareLaunchArgument(
                "planning_use_octomap_map",
                default_value="false",
                description="false uses the dense planning VoxelGridMap; true uses OctoMap as the planner collision map.",
            ),
            DeclareLaunchArgument("pcd_loader", default_value="builtin"),
            DeclareLaunchArgument("resolution", default_value="0.2"),
            DeclareLaunchArgument("occupancy_grid_min_z", default_value="0.2"),
            DeclareLaunchArgument("occupancy_grid_max_z", default_value="0.9"),
            DeclareLaunchArgument("rosbridge", default_value="true"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=package_file("rviz", "nav3d.rviz"),
            ),
            DeclareLaunchArgument("cmd_vel_pose_sim", default_value="false"),
            Node(
                package="nav3d_ros2_bridge",
                executable="nav3d_bridge_node",
                name="nav3d_bridge_node",
                output="screen",
                parameters=[config_path, bridge_overrides],
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="nav3d_rosbridge_websocket",
                output="screen",
                parameters=[
                    {
                        "port": ParameterValue(
                            rosbridge_port,
                            value_type=int,
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("rosbridge")),
            ),
            Node(
                package="nav3d_ros2_bridge",
                executable="nav3d_cmd_vel_pose_sim.py",
                name="nav3d_cmd_vel_pose_sim",
                output="screen",
                parameters=[config_path, {"frame_id": frame_id}],
                condition=IfCondition(LaunchConfiguration("cmd_vel_pose_sim")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="nav3d_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
