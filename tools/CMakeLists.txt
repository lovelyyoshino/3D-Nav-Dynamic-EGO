add_executable(nav3d_standalone_demo standalone_demo.cpp)
target_link_libraries(nav3d_standalone_demo PRIVATE nav3d_core)

add_executable(nav3d_octomap_trajectory_demo octomap_trajectory_demo.cpp)
target_link_libraries(nav3d_octomap_trajectory_demo PRIVATE nav3d_core)

find_package(PCL QUIET COMPONENTS io common)
if(PCL_FOUND)
  add_library(nav3d_pcl_tools
    pcl_pcd_loader.cpp
  )
  target_include_directories(nav3d_pcl_tools
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
      ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(nav3d_pcl_tools
    PUBLIC
      nav3d_core
    PRIVATE
      ${PCL_LIBRARIES}
  )
  target_compile_definitions(nav3d_pcl_tools PRIVATE NAV3D_HAS_PCL=1)
  target_link_libraries(nav3d_standalone_demo PRIVATE nav3d_pcl_tools)
  target_compile_definitions(nav3d_standalone_demo PRIVATE NAV3D_HAS_PCL=1)
  target_link_libraries(nav3d_octomap_trajectory_demo PRIVATE nav3d_pcl_tools)
  target_compile_definitions(nav3d_octomap_trajectory_demo PRIVATE NAV3D_HAS_PCL=1)
endif()

if(NAV3D_BUILD_TESTS)
  add_test(
    NAME octomap_trajectory_demo_plans_mock_corridor
    COMMAND nav3d_octomap_trajectory_demo
      --map-backend octomap
      --planning-mode 2d
      --start -2 0 0
      --goal 2 0 0
      --resolution 0.5
      --save-trajectory-plot ${CMAKE_CURRENT_BINARY_DIR}/mock_corridor_trajectory.svg
      ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mock_corridor_map.pcd
  )
  set_tests_properties(octomap_trajectory_demo_plans_mock_corridor
    PROPERTIES
      PASS_REGULAR_EXPRESSION "loaded_map .*map_backend=octomap.*planning_mode=2d.*occupied_leafs=8.*trajectory_result success=true.*partial=false.*planned_goal=2,0,0.*saved_trajectory_plot=.*mock_corridor_trajectory.svg"
  )

  add_test(
    NAME octomap_trajectory_demo_writes_svg_plot
    COMMAND ${CMAKE_COMMAND}
      -DDEMO_EXE=$<TARGET_FILE:nav3d_octomap_trajectory_demo>
      -DTEST_PCD=${CMAKE_CURRENT_SOURCE_DIR}/testdata/mock_corridor_map.pcd
      -DOUTPUT_SVG=${CMAKE_CURRENT_BINARY_DIR}/mock_corridor_trajectory.svg
      -P ${CMAKE_CURRENT_SOURCE_DIR}/check_octomap_trajectory_demo_plot.cmake
  )

  add_test(
    NAME octomap_trajectory_demo_ground_profile_climbs_mock_stairs
    COMMAND ${CMAKE_COMMAND}
      -DDEMO_EXE=$<TARGET_FILE:nav3d_octomap_trajectory_demo>
      -DTEST_PCD=${CMAKE_CURRENT_SOURCE_DIR}/testdata/mock_stairs_map.pcd
      -DOUTPUT_SVG=${CMAKE_CURRENT_BINARY_DIR}/mock_stairs_trajectory.svg
      -P ${CMAKE_CURRENT_SOURCE_DIR}/check_octomap_trajectory_ground_plot.cmake
  )

  set(NAV3D_REFERENCE_BUILDING_PCD
    ${CMAKE_SOURCE_DIR}/reference/OctoPlanner3D-ROS2/octomap/pcd_files/building2_9.pcd)
  if(EXISTS ${NAV3D_REFERENCE_BUILDING_PCD})
    add_test(
      NAME octomap_trajectory_demo_ground_profile_climbs_reference_stairs
      COMMAND ${CMAKE_COMMAND}
        -DDEMO_EXE=$<TARGET_FILE:nav3d_octomap_trajectory_demo>
        -DTEST_PCD=${NAV3D_REFERENCE_BUILDING_PCD}
        -DOUTPUT_SVG=${CMAKE_CURRENT_BINARY_DIR}/reference_ground_trajectory.svg
        -P ${CMAKE_CURRENT_SOURCE_DIR}/check_octomap_trajectory_ground_plot.cmake
    )
    add_test(
      NAME octomap_trajectory_demo_ground_profile_snaps_reference_request
      COMMAND nav3d_octomap_trajectory_demo
        --map-backend octomap
        --planning-mode 3d
        --start -13 8 1
        --goal 13 8 1
        --resolution 0.5
        --save-trajectory-plot ${CMAKE_CURRENT_BINARY_DIR}/reference_request_ground.svg
        ${NAV3D_REFERENCE_BUILDING_PCD}
    )
    set_tests_properties(octomap_trajectory_demo_ground_profile_snaps_reference_request
      PROPERTIES
        PASS_REGULAR_EXPRESSION "trajectory_result success=true.*planning_mode=3d.*partial=true.*planned_start=-10.75,7.25,0.75.*planned_goal=11.75,5.75,1.75.*z_span_m=1.*saved_trajectory_plot=.*reference_request_ground.svg"
    )
  endif()

  add_test(
    NAME standalone_demo_builds_filtered_map
    COMMAND nav3d_standalone_demo
      --load-only
      --resolution 1.0
      --min-points-per-voxel 2
      --min-cluster-voxels 2
      ${CMAKE_CURRENT_SOURCE_DIR}/testdata/demo_filter_map.pcd
  )
  set_tests_properties(standalone_demo_builds_filtered_map
    PROPERTIES
      PASS_REGULAR_EXPRESSION "raw_points=5.*filtered_points=2.*occupied_voxels=2"
  )

  add_test(
    NAME standalone_demo_saves_filtered_octomap
    COMMAND nav3d_standalone_demo
      --load-only
      --resolution 1.0
      --min-points-per-voxel 2
      --min-cluster-voxels 2
      --save-octomap ${CMAKE_CURRENT_BINARY_DIR}/demo_filter_map.bt
      ${CMAKE_CURRENT_SOURCE_DIR}/testdata/demo_filter_map.pcd
  )
  set_tests_properties(standalone_demo_saves_filtered_octomap
    PROPERTIES
      PASS_REGULAR_EXPRESSION "saved_octomap=.*demo_filter_map.bt.*occupied_leafs=2"
  )

  add_test(
    NAME standalone_demo_saves_multiframe_octomap
    COMMAND nav3d_standalone_demo
      --load-only
      --resolution 1.0
      --min-points-per-voxel 1
      --min-cluster-voxels 1
      --pointcloud-frame-manifest ${CMAKE_CURRENT_SOURCE_DIR}/testdata/multiframe_manifest.txt
      --save-octomap ${CMAKE_CURRENT_BINARY_DIR}/multiframe_map.bt
  )
  set_tests_properties(standalone_demo_saves_multiframe_octomap
    PROPERTIES
      PASS_REGULAR_EXPRESSION "loaded_pointcloud_frames=2.*raw_points=2.*filtered_points=2.*saved_octomap=.*multiframe_map.bt.*occupied_leafs=2"
  )

  if(PCL_FOUND)
    add_test(
      NAME standalone_demo_loads_map_with_pcl_loader
      COMMAND nav3d_standalone_demo
        --load-only
        --pcd-loader pcl
        --resolution 1.0
        --min-points-per-voxel 2
        --min-cluster-voxels 2
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/demo_filter_map.pcd
    )
    set_tests_properties(standalone_demo_loads_map_with_pcl_loader
      PROPERTIES
        PASS_REGULAR_EXPRESSION "pcd_loader=pcl.*raw_points=5.*filtered_points=2.*occupied_voxels=2"
    )

    find_package(GTest REQUIRED)
    add_executable(nav3d_pcl_tools_tests
      test_pcl_pcd_loader.cpp
    )
    target_link_libraries(nav3d_pcl_tools_tests
      PRIVATE
        nav3d_pcl_tools
        GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(nav3d_pcl_tools_tests)
  endif()
endif()
