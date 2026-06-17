if(NOT DEFINED DEMO_EXE)
  message(FATAL_ERROR "DEMO_EXE is required")
endif()
if(NOT DEFINED TEST_PCD)
  message(FATAL_ERROR "TEST_PCD is required")
endif()
if(NOT DEFINED OUTPUT_SVG)
  message(FATAL_ERROR "OUTPUT_SVG is required")
endif()

file(REMOVE "${OUTPUT_SVG}")
execute_process(
  COMMAND "${DEMO_EXE}"
    --map-backend octomap
    --planning-mode 2d
    --start -2 0 0
    --goal 2 0 0
    --resolution 0.5
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
if(NOT demo_stdout MATCHES "loaded_map .*map_backend=octomap.*planning_mode=2d.*occupied_leafs=8")
  message(FATAL_ERROR "demo did not load the mock corridor as expected:\n${demo_stdout}")
endif()
if(NOT demo_stdout MATCHES "trajectory_result success=true.*partial=false.*planned_goal=2,0,0")
  message(FATAL_ERROR "demo did not plan the full mock corridor trajectory:\n${demo_stdout}")
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

get_filename_component(output_dir "${OUTPUT_SVG}" DIRECTORY)
set(DEFAULT_SVG "${output_dir}/nav3d_octomap_trajectory_demo.svg")
file(REMOVE "${DEFAULT_SVG}")
execute_process(
  COMMAND "${DEMO_EXE}"
    --map-backend octomap
    --planning-mode 2d
    --start -2 0 0
    --goal 2 0 0
    --resolution 0.5
    "${TEST_PCD}"
  WORKING_DIRECTORY "${output_dir}"
  RESULT_VARIABLE default_result
  OUTPUT_VARIABLE default_stdout
  ERROR_VARIABLE default_stderr
)

if(NOT default_result EQUAL 0)
  message(FATAL_ERROR
    "nav3d_octomap_trajectory_demo default plot run failed with ${default_result}\nstdout:\n${default_stdout}\nstderr:\n${default_stderr}")
endif()
if(NOT default_stdout MATCHES "saved_trajectory_plot=nav3d_octomap_trajectory_demo.svg")
  message(FATAL_ERROR "demo did not report the default SVG plot path:\n${default_stdout}")
endif()
if(NOT EXISTS "${DEFAULT_SVG}")
  message(FATAL_ERROR "expected default SVG plot was not written: ${DEFAULT_SVG}")
endif()
