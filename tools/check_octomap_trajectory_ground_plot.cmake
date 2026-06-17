if(NOT DEFINED DEMO_EXE)
  message(FATAL_ERROR "DEMO_EXE is required")
endif()
if(NOT DEFINED TEST_PCD)
  message(FATAL_ERROR "TEST_PCD is required")
endif()
if(NOT DEFINED OUTPUT_SVG)
  message(FATAL_ERROR "OUTPUT_SVG is required")
endif()

set(OUTPUT_CSV "${OUTPUT_SVG}.csv")
file(REMOVE "${OUTPUT_SVG}" "${OUTPUT_CSV}")

execute_process(
  COMMAND "${DEMO_EXE}"
    --map-backend octomap
    --planning-mode 3d
    --start 0 0 0.5
    --goal 2 0 2.5
    --resolution 0.5
    --save-trajectory "${OUTPUT_CSV}"
    --save-trajectory-plot "${OUTPUT_SVG}"
    "${TEST_PCD}"
  RESULT_VARIABLE demo_result
  OUTPUT_VARIABLE demo_stdout
  ERROR_VARIABLE demo_stderr
)

if(NOT demo_result EQUAL 0)
  message(FATAL_ERROR
    "nav3d_octomap_trajectory_demo failed with ${demo_result}\nstdout:\n${demo_stdout}\nstderr:\n${demo_stderr}")
endif()

if(NOT demo_stdout MATCHES "trajectory_result success=true")
  message(FATAL_ERROR "demo did not report success:\n${demo_stdout}")
endif()
if(NOT demo_stdout MATCHES "planning_mode=3d")
  message(FATAL_ERROR "demo did not use 3D planning mode:\n${demo_stdout}")
endif()
if(NOT demo_stdout MATCHES "partial=false")
  message(FATAL_ERROR "demo only produced a partial path:\n${demo_stdout}")
endif()

if(NOT EXISTS "${OUTPUT_SVG}")
  message(FATAL_ERROR "expected SVG plot was not written: ${OUTPUT_SVG}")
endif()
file(READ "${OUTPUT_SVG}" svg_content)
if(NOT svg_content MATCHES "<svg id=\"nav3d-trajectory-plot\"")
  message(FATAL_ERROR "SVG plot does not look like the Nav3D trajectory plot")
endif()
if(NOT svg_content MATCHES "id=\"isometric-trajectory-view\"")
  message(FATAL_ERROR "SVG plot did not render the isometric trajectory view")
endif()

if(NOT EXISTS "${OUTPUT_CSV}")
  message(FATAL_ERROR "expected CSV samples were not written: ${OUTPUT_CSV}")
endif()
file(READ "${OUTPUT_CSV}" csv_content)
if(NOT csv_content MATCHES "\n[0-9.]+,[^,\n]+,[^,\n]+,0\\.750000")
  message(FATAL_ERROR "ground trajectory samples do not include the low stair level:\n${csv_content}")
endif()
if(NOT csv_content MATCHES "\n[0-9.]+,[^,\n]+,[^,\n]+,2\\.(250000|750000)")
  message(FATAL_ERROR "ground trajectory samples do not climb to the upper stair level:\n${csv_content}")
endif()
