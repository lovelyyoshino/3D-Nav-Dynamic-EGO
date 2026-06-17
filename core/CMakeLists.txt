find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

add_library(nav3d_core
  map_core/src/local_grid.cpp
  map_core/src/map_builder.cpp
  map_core/src/map_2d_projection.cpp
  map_core/src/map_composite.cpp
  map_core/src/octomap_manager.cpp
  map_core/src/pcd_loader.cpp
  map_core/src/point_cloud_preprocessor.cpp
  map_core/src/voxel_grid_map.cpp
  planner_core/src/path_searching/astar_3d.cpp
  planner_core/src/path_searching/jps_3d.cpp
  planner_core/src/ego_planner_core.cpp
  planner_core/src/replan_fsm.cpp
  planner_core/src/bspline/bspline_optimizer.cpp
  planner_core/src/bspline/uniform_bspline.cpp
  collision_engine/src/inflation_layer.cpp
  collision_engine/src/sphere_checker.cpp
  collision_engine/src/trajectory_checker.cpp
  collision_engine/src/trajectory_feasibility_checker.cpp
  controller_core/src/motion_model.cpp
  controller_core/src/safety_monitor.cpp
  controller_core/src/trajectory_tracker.cpp
)

target_include_directories(nav3d_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/map_core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/planner_core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/collision_engine/include
    ${CMAKE_CURRENT_SOURCE_DIR}/controller_core/include
)

target_link_libraries(nav3d_core PUBLIC Eigen3::Eigen)
target_include_directories(nav3d_core PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(nav3d_core PUBLIC ${OCTOMAP_LIBRARIES})

if(NAV3D_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(nav3d_core_tests
    tests/test_local_grid.cpp
    tests/test_map_builder.cpp
    tests/test_map_2d_projection.cpp
    tests/test_map_composite.cpp
    tests/test_octomap_manager.cpp
    tests/test_pcd_loader.cpp
    tests/test_point_cloud_preprocessor.cpp
    tests/test_replan_fsm.cpp
    tests/test_voxel_grid_map.cpp
    tests/test_astar_3d.cpp
    tests/test_jps_3d.cpp
    tests/test_planning_scenario_matrix.cpp
    tests/test_ego_planner_core.cpp
    tests/test_bspline_optimizer.cpp
    tests/test_uniform_bspline.cpp
    tests/test_inflation_layer.cpp
    tests/test_sphere_checker.cpp
    tests/test_trajectory_checker.cpp
    tests/test_trajectory_feasibility_checker.cpp
    tests/test_motion_model.cpp
    tests/test_safety_monitor.cpp
    tests/test_trajectory_tracker.cpp
  )

  target_link_libraries(nav3d_core_tests PRIVATE nav3d_core GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(nav3d_core_tests)
endif()
