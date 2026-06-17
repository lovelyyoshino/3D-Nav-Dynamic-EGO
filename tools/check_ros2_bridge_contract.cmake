set(BRIDGE_DIR "${PROJECT_SOURCE_DIR}/ros2_bridge/nav3d_ros2_bridge")
set(BRIDGE_PACKAGE "${BRIDGE_DIR}/package.xml")
set(BRIDGE_NODE "${BRIDGE_DIR}/src/nav3d_bridge_node.cpp")
set(BRIDGE_LAUNCH "${BRIDGE_DIR}/launch/nav3d_bridge.launch.py")
set(BRIDGE_RVIZ "${BRIDGE_DIR}/rviz/nav3d.rviz")
set(BRIDGE_CONFIG "${BRIDGE_DIR}/config/nav3d_bridge.yaml")
set(BRIDGE_CMAKE "${BRIDGE_DIR}/CMakeLists.txt")
set(WEB_APP "${PROJECT_SOURCE_DIR}/web_ui/src/App.tsx")
set(WEB_CLIENT "${PROJECT_SOURCE_DIR}/web_ui/src/rosbridge/client.ts")

foreach(required_file IN ITEMS
    "${BRIDGE_PACKAGE}"
    "${BRIDGE_NODE}"
    "${BRIDGE_LAUNCH}"
    "${BRIDGE_RVIZ}"
    "${BRIDGE_CONFIG}"
    "${BRIDGE_CMAKE}"
    "${WEB_APP}"
    "${WEB_CLIENT}")
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR "missing bridge contract file: ${required_file}")
  endif()
endforeach()

file(READ "${BRIDGE_PACKAGE}" package_xml)
foreach(required_text IN ITEMS
    "<name>nav3d_ros2_bridge</name>"
    "<exec_depend>rosbridge_server</exec_depend>"
    "<exec_depend>rviz2</exec_depend>"
    "<depend>visualization_msgs</depend>")
  if(NOT package_xml MATCHES "${required_text}")
    message(FATAL_ERROR "package.xml is missing required text: ${required_text}")
  endif()
endforeach()

file(READ "${BRIDGE_NODE}" bridge_node)
foreach(required_text IN ITEMS
    "/nav3d/start"
    "/nav3d/goal"
    "/nav3d/trajectory"
    "/nav3d/trajectory_marker"
    "/nav3d/start_marker"
    "/nav3d/goal_marker"
	    "/nav3d/status"
	    "/nav3d/planning_occupied_markers"
    "/nav3d/occupied_grid"
    "map.occupancy_grid_min_z"
    "map.occupancy_grid_max_z"
    "Map2DProjectionOptions"
    "free_cell_z"
    "is_free_cell"
    "free="
    "/clicked_point"
    "/octomap"
    "planning.mode"
    "PlanningTraversabilityMode"
    "planning.traversability"
    "planning.use_octomap_map"
    "planning.uav_endpoint_snap_radius_cells"
	    "resolveUavEndpoint"
	    "resolveGroundEndpoint"
	    "uav_endpoint_snapped"
	    "ground_endpoint_snapped"
    "groundTraversabilityEnabled"
    "planningMapBackendName"
    "GroundSupportedSearcher"
    "GroundTraversabilityMap"
    "return planning_traversability_mode_ == PlanningTraversabilityMode::Ground"
    "clicked_point_topic"
    "onClickedPoint"
    "planning.ground_strict_direct_support"
    "visualization_msgs::msg::Marker"
    "visualization_msgs::msg::MarkerArray"
    "visualization_msgs::msg::Marker::CUBE_LIST"
    "visualization_msgs::msg::Marker::LINE_STRIP"
	    "publishTrajectoryMarker"
	    "publishPlanningOccupiedMarkers"
	    "publishOctomapBinary")
  if(NOT bridge_node MATCHES "${required_text}")
    message(FATAL_ERROR "bridge node is missing required text: ${required_text}")
  endif()
endforeach()
string(CONCAT removed_octomap_marker_topic "/nav3d/octomap_" "occupied_markers")
string(CONCAT removed_octomap_marker_builder "makeOctomap" "OccupiedMarkerArray")
string(CONCAT removed_octomap_marker_publisher "publishOctomap" "OccupiedMarkers")
string(CONCAT removed_octomap_marker_status "octomap_" "markers")
foreach(forbidden_text IN ITEMS
    "${removed_octomap_marker_topic}"
    "${removed_octomap_marker_builder}"
    "${removed_octomap_marker_publisher}"
    "${removed_octomap_marker_status}")
  if(bridge_node MATCHES "${forbidden_text}")
    message(FATAL_ERROR "bridge node must not contain removed OctoMap occupied marker text: ${forbidden_text}")
  endif()
endforeach()

file(READ "${BRIDGE_LAUNCH}" bridge_launch)
foreach(required_text IN ITEMS
    "config_path"
    "nav3d_bridge.yaml"
    "DeclareLaunchArgument\\(\"rosbridge\", default_value=\"true\"\\)"
    "DeclareLaunchArgument\\(\"rviz\", default_value=\"true\"\\)"
    "planning_mode"
    "planning.mode"
    "planning_traversability"
    "planning.traversability"
    "planning_use_octomap_map"
    "planning.use_octomap_map"
    "DeclareLaunchArgument\\(\"resolution\", default_value=\"0.2\"\\)"
    "DeclareLaunchArgument\\(\"occupancy_grid_min_z\", default_value=\"0.2\"\\)"
    "DeclareLaunchArgument\\(\"occupancy_grid_max_z\", default_value=\"0.9\"\\)"
    "rosbridge_server"
    "rosbridge_websocket"
    "rosbridge_port"
    "9090"
    "rviz2"
    "nav3d.rviz")
  if(NOT bridge_launch MATCHES "${required_text}")
    message(FATAL_ERROR "launch file is missing required text: ${required_text}")
  endif()
endforeach()
foreach(forbidden_text IN ITEMS
    "DeclareLaunchArgument\\(\"dynamic_"
    "DeclareLaunchArgument\\(\"time_allocation_"
    "DeclareLaunchArgument\\(\"planning_ground_"
    "DeclareLaunchArgument\\(\"planning_uav_"
    "DeclareLaunchArgument\\(\"min_points_per_voxel"
    "DeclareLaunchArgument\\(\"min_cluster_voxels"
    "DeclareLaunchArgument\\(\"insert_free_space_rays"
    "DeclareLaunchArgument\\(\"sensor_origin_"
    "DeclareLaunchArgument\\(\"occupancy_grid_max_cells"
    "DeclareLaunchArgument\\(\"local_grid_"
    "DeclareLaunchArgument\\(\"safety_"
    "DeclareLaunchArgument\\(\"controller_"
    "DeclareLaunchArgument\\(\"cmd_vel_pose_sim_start_"
    "DeclareLaunchArgument\\(\"cmd_vel_pose_sim_update_"
    "DeclareLaunchArgument\\(\"cmd_vel_pose_sim_max_step"
    "DeclareLaunchArgument\\(\"cmd_vel_pose_sim_cmd_timeout"
    "DeclareLaunchArgument\\(\"cmd_vel_pose_sim_command_frame")
  if(bridge_launch MATCHES "${forbidden_text}")
    message(FATAL_ERROR "launch file still exposes stable config as launch argument: ${forbidden_text}")
  endif()
endforeach()

file(READ "${BRIDGE_CONFIG}" bridge_config)
foreach(required_text IN ITEMS
    "nav3d_bridge_node:"
    "nav3d_cmd_vel_pose_sim:"
    "ros__parameters:"
    "planning.enable_dynamic_feasibility_check:"
    "planning.ground_snap_radius_cells:"
    "planning.uav_endpoint_snap_radius_cells:"
    "map.min_points_per_voxel:"
    "map.occupancy_grid_min_z: 0.2"
    "map.occupancy_grid_max_z: 0.9"
    "local_grid.enabled:"
    "safety.enabled:"
    "controller.enabled:"
    "controller.max_speed:"
    "cmd_timeout:")
  if(NOT bridge_config MATCHES "${required_text}")
    message(FATAL_ERROR "bridge config is missing required text: ${required_text}")
  endif()
endforeach()

file(READ "${BRIDGE_CMAKE}" bridge_cmake)
if(NOT bridge_cmake MATCHES "install\\(DIRECTORY launch rviz config")
  message(FATAL_ERROR "CMake install rule must install launch, rviz, and config directories")
endif()

file(READ "${BRIDGE_RVIZ}" bridge_rviz)
if(bridge_rviz MATCHES "Marker Topic:")
  message(FATAL_ERROR "RViz MarkerArray displays must use the Humble-compatible 'Topic:' property, not 'Marker Topic:'")
endif()
foreach(required_text IN ITEMS
	    "Class: rviz_default_plugins/MarkerArray"
	    "Name: Nav3D Planning Occupied Voxels"
	    "Value: /nav3d/planning_occupied_markers"
    "Name: Nav3D 3D Trajectory Marker"
    "Value: /nav3d/trajectory_marker"
    "Name: Nav3D Start Marker"
    "Value: /nav3d/start_marker"
    "Name: Nav3D Goal Marker"
    "Value: /nav3d/goal_marker"
    "Class: rviz_default_plugins/PublishPoint"
    "Value: /clicked_point"
    "Class: rviz_default_plugins/Map"
    "Name: Nav3D OccupancyGrid Fallback"
    "Enabled: false")
  if(NOT bridge_rviz MATCHES "${required_text}")
    message(FATAL_ERROR "RViz config is missing required text: ${required_text}")
  endif()
endforeach()
if(bridge_rviz MATCHES "/nav3d/occupied_voxels")
  message(FATAL_ERROR "RViz must not display the removed VoxelGrid debug topic /nav3d/occupied_voxels")
endif()
if(bridge_rviz MATCHES "Alpha: 0\\.32")
  message(FATAL_ERROR "RViz OccupancyGrid display must not use low alpha; free cells should render white")
endif()
if(bridge_rviz MATCHES "Window Geometry:")
  message(FATAL_ERROR "RViz config must not pin Window Geometry; it can hang Qt restore on another display")
endif()
string(CONCAT removed_octomap_marker_display "Nav3D OctoMap " "Occupied Leafs")
string(CONCAT removed_octomap_marker_ns "nav3d_octomap_" "occupied_leafs")
foreach(forbidden_text IN ITEMS
    "${removed_octomap_marker_topic}"
    "${removed_octomap_marker_display}"
    "${removed_octomap_marker_ns}")
  if(bridge_rviz MATCHES "${forbidden_text}")
    message(FATAL_ERROR "RViz must not display the removed OctoMap occupied marker topic: ${forbidden_text}")
  endif()
endforeach()

file(READ "${WEB_APP}" web_app)
if(web_app MATCHES "/api/nav3d/bridge/start")
  message(FATAL_ERROR "web UI must not start rosbridge; it should connect to the default running rosbridge")
endif()
foreach(required_text IN ITEMS
    "ws://localhost:9090"
    "selectVoxelTargetFromHits"
    "检查/重连")
  if(NOT web_app MATCHES "${required_text}")
    message(FATAL_ERROR "web App is missing required text: ${required_text}")
  endif()
endforeach()

file(READ "${WEB_CLIENT}" web_client)
foreach(required_text IN ITEMS
	    "/nav3d/start"
	    "/nav3d/goal"
	    "/nav3d/trajectory"
	    "/nav3d/planning_occupied_markers"
	    "visualization_msgs/MarkerArray")
  if(NOT web_client MATCHES "${required_text}")
    message(FATAL_ERROR "rosbridge client is missing required text: ${required_text}")
  endif()
endforeach()
