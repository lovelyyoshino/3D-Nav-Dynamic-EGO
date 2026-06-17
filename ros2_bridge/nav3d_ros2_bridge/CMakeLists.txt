cmake_minimum_required(VERSION 3.16)
project(nav3d_ros2_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs QUIET)

set(NAV3D_REPO_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../.."
  CACHE PATH "Path to the 3D-Octomap-Ego-Nav repository root")

if(NOT TARGET nav3d_core)
  set(NAV3D_BUILD_TESTS OFF CACHE BOOL "Build nav3d core tests" FORCE)
  add_subdirectory(
    "${NAV3D_REPO_ROOT}/core"
    "${CMAKE_CURRENT_BINARY_DIR}/nav3d_core_build")
endif()

find_package(PCL QUIET COMPONENTS io common)
if(PCL_FOUND AND NOT TARGET nav3d_pcl_tools)
  add_subdirectory(
    "${NAV3D_REPO_ROOT}/tools"
    "${CMAKE_CURRENT_BINARY_DIR}/nav3d_tools_build")
endif()

add_executable(nav3d_bridge_node src/nav3d_bridge_node.cpp)
target_link_libraries(nav3d_bridge_node nav3d_core)
if(PCL_FOUND)
  target_link_libraries(nav3d_bridge_node nav3d_pcl_tools)
  target_compile_definitions(nav3d_bridge_node PRIVATE NAV3D_HAS_PCL=1)
endif()
if(octomap_msgs_FOUND)
  target_compile_definitions(nav3d_bridge_node PRIVATE NAV3D_HAS_OCTOMAP_MSGS=1)
  get_filename_component(NAV3D_OCTOMAP_MSGS_PREFIX
    "${octomap_msgs_DIR}/../../.."
    ABSOLUTE)
  set(NAV3D_OCTOMAP_MSGS_LIBRARY_DIR
    "${NAV3D_OCTOMAP_MSGS_PREFIX}/lib")
  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_FOUND)
    set(NAV3D_OCTOMAP_MSGS_PYTHONPATH
      "${NAV3D_OCTOMAP_MSGS_PREFIX}/local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages")
  else()
    set(NAV3D_OCTOMAP_MSGS_PYTHONPATH
      "${NAV3D_OCTOMAP_MSGS_PREFIX}/local/lib/python3.10/dist-packages")
  endif()
endif()
ament_target_dependencies(
  nav3d_bridge_node
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)
if(octomap_msgs_FOUND)
  ament_target_dependencies(nav3d_bridge_node octomap_msgs)
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/environment/nav3d_octomap_msgs_library_path.sh.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/environment/nav3d_octomap_msgs_library_path.dsv.in")
endif()
set_target_properties(nav3d_bridge_node PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS nav3d_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/nav3d_cmd_vel_pose_sim.py
  scripts/nav3d_local_replan_smoke.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME})

ament_package()
