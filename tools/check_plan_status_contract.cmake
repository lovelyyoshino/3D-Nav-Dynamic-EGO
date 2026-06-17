set(BRIDGE_NODE "${PROJECT_SOURCE_DIR}/ros2_bridge/nav3d_ros2_bridge/src/nav3d_bridge_node.cpp")
set(WEB_APP "${PROJECT_SOURCE_DIR}/web_ui/src/App.tsx")
set(DEMO "${PROJECT_SOURCE_DIR}/tools/octomap_trajectory_demo.cpp")

foreach(required_file IN ITEMS "${BRIDGE_NODE}" "${WEB_APP}" "${DEMO}")
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR "missing plan status contract file: ${required_file}")
  endif()
endforeach()

file(READ "${BRIDGE_NODE}" bridge_node)
foreach(required_text IN ITEMS
    "plan_success"
    "plan_failed"
    "planned_goal="
    "requested_goal="
    "partial="
    "appendPlanEndpointStatus")
  if(NOT bridge_node MATCHES "${required_text}")
    message(FATAL_ERROR "bridge node is missing plan status text: ${required_text}")
  endif()
endforeach()

file(READ "${WEB_APP}" web_app)
foreach(required_text IN ITEMS
    "parsePlanStatusKind"
    "partial=true"
    "plan_failed"
    "shortened fallback")
  if(NOT web_app MATCHES "${required_text}")
    message(FATAL_ERROR "web App is missing plan status text: ${required_text}")
  endif()
endforeach()

file(READ "${DEMO}" demo_cpp)
foreach(required_text IN ITEMS
    "planning_mode="
    "GroundSupportedSearcher"
    "planned_start="
    "planned_goal="
    "z_span_m="
    "saved_trajectory_plot=")
  if(NOT demo_cpp MATCHES "${required_text}")
    message(FATAL_ERROR "octomap trajectory demo is missing status text: ${required_text}")
  endif()
endforeach()
