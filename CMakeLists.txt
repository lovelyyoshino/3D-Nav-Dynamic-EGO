cmake_minimum_required(VERSION 3.16)
project(nav3d_octomap_ego_nav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(NAV3D_BUILD_TESTS "Build unit tests" ON)

if(NAV3D_BUILD_TESTS)
  enable_testing()
  add_test(
    NAME ros2_bridge_contract
    COMMAND ${CMAKE_COMMAND}
      -DPROJECT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_ros2_bridge_contract.cmake
  )
  add_test(
    NAME plan_status_contract
    COMMAND ${CMAKE_COMMAND}
      -DPROJECT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_plan_status_contract.cmake
  )
endif()

add_subdirectory(core)
add_subdirectory(tools)
