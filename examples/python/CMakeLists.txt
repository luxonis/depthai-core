# Prevent from being configured
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(FATAL_ERROR "Use projects root CMakeLists.txt to configure")
endif()

# Add options
option(DEPTHAI_PYTHON_TEST_EXAMPLES "Test examples - examples will be ran as a part of the test suite" OFF)

# Specify path separator
set(SYS_PATH_SEPARATOR ";")
if(UNIX)
    set(SYS_PATH_SEPARATOR ":")
endif()

set(TARGET_NAME "depthai")

# Regexes for example test failure
set(STRICT_EXAMPLE_TEST_FAIL_REGEX "\\[warning\\];\\[error\\];\\[critical\\]")
set(RELAXED_EXAMPLE_TEST_FAIL_REGEX "\\[error\\];\\[critical\\]")

# Add a target to install_requirements (added to ALL)
# For windows we don't install open3d-cpu
if(WIN32)
    add_custom_target(install_requirements ALL
        COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/install_requirements.py" "--skip_depthai"
        DEPENDS ${TARGET_NAME}
        VERBATIM
        COMMAND_EXPAND_LISTS
    )
else()
    add_custom_target(install_requirements ALL
        COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/install_requirements.py" "--skip_depthai" "--install_rerun" "--install_open3d_cpu"
        DEPENDS ${TARGET_NAME}
        VERBATIM
        COMMAND_EXPAND_LISTS
    )
endif()

# Function for adding new python test
function(add_python_example example_name python_script_path)
    # Modify example name to signify that its Python based
    set(example_name "py_${example_name}")

    # parse the rest of the arguments
    set(arguments ${ARGV})
    list(REMOVE_AT arguments 0 1)

    # Creates a target (python my_test [args])
    add_custom_target(${example_name}
        ${CMAKE_COMMAND} -E env
        # Environment variables
        # PATH (dlls)
        "PATH=${HUNTER_INSTALL_PREFIX}/bin${SYS_PATH_SEPARATOR}$ENV{PATH}"
        # Python path (to find compiled module)
        "PYTHONPATH=$<TARGET_FILE_DIR:${TARGET_NAME}>${SYS_PATH_SEPARATOR}$ENV{PYTHONPATH}"
        "DEPTHAI_SEARCH_TIMEOUT=20000"
        "DEPTHAI_CONNECT_TIMEOUT=10000"
        "DEPTHAI_RECONNECT_TIMEOUT=0"
        # ASAN in case of sanitizers
        "${ASAN_ENVIRONMENT_VARS}"
        # Example
        ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/${python_script_path} ${ARGN}
        DEPENDS ${TARGET_NAME} install_requirements
        VERBATIM
        COMMAND_EXPAND_LISTS
    )

    if(DEPTHAI_PYTHON_TEST_EXAMPLES)

        # Adds test with 5 seconds timeout and bumps all python warnings to errors
        add_test(NAME ${example_name} COMMAND
            ${CMAKE_COMMAND} -E env
            "PATH=${HUNTER_INSTALL_PREFIX}/bin${SYS_PATH_SEPARATOR}$ENV{PATH}"
            "PYTHONPATH=$<TARGET_FILE_DIR:${TARGET_NAME}>${SYS_PATH_SEPARATOR}$ENV{PYTHONPATH}"
            "DEPTHAI_SEARCH_TIMEOUT=20000"
            "DEPTHAI_CONNECT_TIMEOUT=10000"
            "DEPTHAI_RECONNECT_TIMEOUT=0"
            ${ASAN_ENVIRONMENT_VARS}

            ${CMAKE_COMMAND}
            -DFORCE_TIMEOUT_SECONDS=300
            -P ${CMAKE_CURRENT_LIST_DIR}/../cpp/cmake/ExecuteTestTimeout.cmake
            $<TARGET_FILE:test_wrapper>
            40 # A timeout to terminate the test when sucessful
            ${PYTHON_EXECUTABLE}
            -Werror
            "${CMAKE_CURRENT_LIST_DIR}/${python_script_path}"
            ${arguments}
        )

        add_dependencies(${example_name} test_wrapper)

        # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
        set_tests_properties (${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION ${STRICT_EXAMPLE_TEST_FAIL_REGEX})

    endif()

endfunction()


# Function for setting test labels, including label shortcuts
function(dai_set_example_test_labels example_name)
    # If not testing examples, return
    if(NOT DEPTHAI_PYTHON_TEST_EXAMPLES)
        return()
    endif()

    # Modify example name to match the one created by add_python_example
    set(example_name "py_${example_name}")
    set(expanded_labels "")
    foreach(label IN LISTS ARGN)
        if(label STREQUAL "rvc2_all")
            list(APPEND expanded_labels "rvc2" "poe" "usb")
        else()
            list(APPEND expanded_labels ${label})
        endif()
    endforeach()
    # Add "cpp_example" label
    list(APPEND expanded_labels "python_example")
    set_tests_properties(${example_name} PROPERTIES LABELS "${expanded_labels}")
endfunction()

if(DEPTHAI_PYTHON_TEST_EXAMPLES)

    # Adds install requirements test with 5 minute timeout
    add_test(NAME install_requirements COMMAND
        ${CMAKE_COMMAND} -E env
        # PATH (dlls)
        "PATH=${HUNTER_INSTALL_PREFIX}/bin${SYS_PATH_SEPARATOR}$ENV{PATH}"
        # Python path (to find compiled module)
        "PYTHONPATH=$<TARGET_FILE_DIR:${TARGET_NAME}>${SYS_PATH_SEPARATOR}$ENV{PYTHONPATH}"
        # ASAN in case of sanitizers
        ${ASAN_ENVIRONMENT_VARS}
        ${CMAKE_COMMAND} -DFORCE_TIMEOUT_SECONDS=300 -P ${CMAKE_CURRENT_LIST_DIR}/../cpp/cmake/ExecuteTestTimeout.cmake

        # Actual script to run
        ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/install_requirements.py" "--skip_depthai"
    )

    # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
    set_tests_properties (install_requirements PROPERTIES FAIL_REGULAR_EXPRESSION "${STRICT_EXAMPLE_TEST_FAIL_REGEX}")

endif()

function(dai_example_test_relax_fail_regex example_name)
    set(example_name "py_${example_name}")
    if(DEPTHAI_PYTHON_TEST_EXAMPLES AND TEST ${example_name})
        set_tests_properties(${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION "${RELAXED_EXAMPLE_TEST_FAIL_REGEX}")
    endif()
endfunction()


## Camera output
add_python_example(camera_output Camera/camera_output.py)
dai_set_example_test_labels(camera_output ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(camera_all Camera/camera_all.py)
dai_set_example_test_labels(camera_all ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(camera_multiple_outputs Camera/camera_multiple_outputs.py 640 480 0 30 CAM_A 300 300 0 30 CAM_A 300 300 1 30 CAM_A)
dai_set_example_test_labels(camera_multiple_outputs ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(camera_raw Camera/camera_raw.py)
dai_set_example_test_labels(camera_raw ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(camera_roi_exposure_focus Camera/camera_roi_exposure_focus.py)
dai_set_example_test_labels(camera_roi_exposure_focus ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(camera_still_max_res Camera/camera_still_max_res.py)
dai_set_example_test_labels(camera_still_max_res ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(camera_max_resolution Camera/camera_max_resolution.py)
dai_set_example_test_labels(camera_max_resolution ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(camera_undistortion Camera/camera_undistortion.py)
dai_set_example_test_labels(camera_undistortion ondevice rvc2_all rvc4 ci)

add_python_example(camera_hdr Camera/camera_hdr.py)
dai_set_example_test_labels(camera_hdr ondevice rvc4 rvc4rgb ci)

## AprilTags
add_python_example(april_tags_12mp AprilTags/april_tags_12mp.py)
dai_set_example_test_labels(april_tags_12mp ondevice rvc2_all rvc4 rvc4rgb ci nowindows)

add_python_example(april_tags AprilTags/april_tags.py)
dai_set_example_test_labels(april_tags ondevice rvc2_all rvc4 rvc4rgb ci nowindows)

add_python_example(april_tags_replay AprilTags/april_tags_replay.py)
dai_set_example_test_labels(april_tags_replay ondevice rvc2_all rvc4 rvc4rgb ci nowindows)

## Detection network
add_python_example(detection_network DetectionNetwork/detection_network.py)
dai_set_example_test_labels(detection_network ondevice rvc2_all rvc4 rvc4rgb ci)
add_python_example(detection_network_remap DetectionNetwork/detection_network_remap.py)
dai_set_example_test_labels(detection_network ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(detection_network_replay_rvc4 DetectionNetwork/detection_network_replay.py --webSocketPort 8761 --httpPort 8071)
dai_set_example_test_labels(detection_network_replay_rvc4 ondevice rvc4 rvc4rgb ci)

add_python_example(detection_network_replay_rvc2 DetectionNetwork/detection_network_replay.py --webSocketPort 8762 --httpPort 8072)
dai_set_example_test_labels(detection_network_replay_rvc2 ondevice rvc2 usb rvc4rgb ci)

## Host nodes
add_python_example(display HostNodes/display.py)
dai_set_example_test_labels(display ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(threaded_host_nodes HostNodes/threaded_host_nodes.py)
dai_set_example_test_labels(threaded_host_nodes ondevice rvc2_all rvc4 rvc4rgb ci)

## ImageManip
add_python_example(image_manip_multi_ops ImageManip/image_manip_multi_ops.py)
dai_set_example_test_labels(image_manip_multi_ops ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(image_manip_resize ImageManip/image_manip_resize.py)
dai_set_example_test_labels(image_manip_resize ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(image_manip_remap ImageManip/image_manip_remap.py)
dai_set_example_test_labels(image_manip_remap ondevice rvc2_all rvc4 rvc4rgb ci)

## Misc
add_python_example(reconnect_callback Misc/AutoReconnect/reconnect_callback.py)
dai_set_example_test_labels(reconnect_callback ondevice rvc2_all rvc4 rvc4rgb ci)

## SystemLogger
add_python_example(system_logger Misc/SystemLogger/system_information.py)
dai_set_example_test_labels(system_logger ondevice rvc2_all rvc4 ci)

## EdgeDetector
add_python_example(edge_detector RVC2/EdgeDetector/edge_detector.py)
dai_set_example_test_labels(edge_detector ondevice rvc2_all ci)

## RGBD
add_python_example(rgbd RGBD/rgbd.py)
dai_set_example_test_labels(rgbd ondevice rvc2_all rvc4 ci nowindows)

add_python_example(visualizer_rgbd RGBD/visualizer_rgbd.py)
dai_set_example_test_labels(visualizer_rgbd ondevice rvc2 usb rvc4 ci nowindows)

add_python_example(visualizer_rgbd_autocreate RGBD/visualizer_rgbd_autocreate.py)
dai_set_example_test_labels(visualizer_rgbd_autocreate ondevice rvc2 usb rvc4 ci nowindows)

add_python_example(rgbd_o3d RGBD/rgbd_o3d.py)
dai_set_example_test_labels(rgbd_o3d ondevice rvc2_all rvc4 ci nowindows)

add_python_example(rgbd_pcl_processing RGBD/rgbd_pcl_processing.py)
dai_set_example_test_labels(rgbd_pcl_processing ondevice rvc2_all rvc4 ci nowindows)

add_python_example(person_crop_out Misc/DigitalZoom/person_crop_out.py)
dai_set_example_test_labels(person_crop_out ondevice rvc2_all rvc4 rvc4rgb ci)

## NeuralNetwork
add_python_example(neural_network NeuralNetwork/neural_network.py)
dai_set_example_test_labels(neural_network ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(neural_network_multi_input_combined NeuralNetwork/neural_network_multi_input_combined.py)
dai_set_example_test_labels(neural_network_multi_input_combined ondevice rvc2_all rvc4 rvc4rgb ci)

add_python_example(neural_network_multi_input NeuralNetwork/neural_network_multi_input.py)
dai_set_example_test_labels(neural_network_multi_input ondevice rvc2_all rvc4 rvc4rgb ci)

## Spatial detection network
add_python_example(spatial_detection SpatialDetectionNetwork/spatial_detection.py)
dai_set_example_test_labels(spatial_detection ondevice rvc2_all rvc4 ci)

## StereoDepth
add_python_example(stereo StereoDepth/stereo.py)
dai_set_example_test_labels(stereo ondevice rvc2_all rvc4 ci)

add_python_example(stereo_depth_from_host StereoDepth/stereo_depth_from_host.py)
dai_set_example_test_labels(stereo_depth_from_host ondevice rvc2_all rvc4 ci)

## VideoEncoder
add_python_example(video_encoder VideoEncoder/video_encode.py)
dai_set_example_test_labels(video_encoder ondevice rvc2_all rvc4 rvc4rgb ci)

## SpatialLocationCalculator
add_python_example(spatial_location_calculator SpatialLocationCalculator/spatial_location_calculator.py)
dai_set_example_test_labels(spatial_location_calculator ondevice rvc2_all rvc4 ci)

## ImageAlign
add_python_example(depth_align ImageAlign/depth_align.py)
dai_set_example_test_labels(depth_align ondevice rvc2_all rvc4 ci)

## Visualizer

# Split into two examples for RVC2 and RVC4 since the tests are run concurrently, to avoid port conflicts
# Skip windows, since it asks for user input for the firewall
add_python_example(visualizer_rvc4 Visualizer/visualizer_yolo.py --webSocketPort 8765 --httpPort 8082)
dai_set_example_test_labels(visualizer_rvc4 ondevice rvc4 rvc4rgb ci nowindows)

add_python_example(visualizer_encoded_rvc4 Visualizer/visualizer_encoded.py --webSocketPort 8765 --httpPort 8082)
dai_set_example_test_labels(visualizer_encoded_rvc4 ondevice rvc4 rvc4rgb ci nowindows)

add_python_example(visualizer_rvc2 Visualizer/visualizer_yolo.py --webSocketPort 8766 --httpPort 8081)
dai_set_example_test_labels(visualizer_rvc2 ondevice rvc2 usb ci nowindows)

add_python_example(visualizer_encoded_rvc2 Visualizer/visualizer_encoded.py --webSocketPort 8766 --httpPort 8081)
dai_set_example_test_labels(visualizer_encoded_rvc2 ondevice rvc2 usb ci nowindows)

add_python_example(custom_services Visualizer/custom_services.py)
dai_set_example_test_labels(custom_services ondevice rvc2_all rvc4 rvc4rgb ci nowindows) # Host-only test


## Script node
add_python_example(script_simple Script/script_simple.py)
dai_set_example_test_labels(script_simple ondevice rvc2_all rvc4 rvc4rgb ci)
dai_example_test_relax_fail_regex(script_simple)

add_python_example(script_all_cameras Script/script_switch_all_cameras.py)
dai_set_example_test_labels(script_all_cameras ondevice rvc2_all rvc4 rvc4rgb ci)

## Benchmark node
add_python_example(benchmark_node Benchmark/benchmark_simple.py)
dai_set_example_test_labels(benchmark_node ondevice rvc2_all rvc4 rvc4rgb ci)
dai_example_test_relax_fail_regex(benchmark_node)

add_python_example(benchmark_cameras Benchmark/benchmark_camera.py)
dai_set_example_test_labels(benchmark_cameras ondevice rvc2_all rvc4 rvc4rgb ci)
dai_example_test_relax_fail_regex(benchmark_cameras)

add_python_example(benchmark_nn Benchmark/benchmark_nn.py)
dai_set_example_test_labels(benchmark_nn ondevice rvc2_all rvc4 rvc4rgb ci)

# IMU node
add_python_example(imu_gyroscope_accelerometer IMU/imu_gyroscope_accelerometer.py)
dai_set_example_test_labels(imu_gyroscope_accelerometer rvc2_all rvc4 rvc4rgb ci)
# On slow hosts the examples warns that batch size is too small
dai_example_test_relax_fail_regex(imu_gyroscope_accelerometer)

# Warp node
add_python_example(warp_mesh Warp/warp_mesh.py)
dai_set_example_test_labels(warp_mesh rvc2_all rvc4 rvc4rgb ci)

# Feature tracker
add_python_example(feature_tracker FeatureTracker/feature_tracker.py)
dai_set_example_test_labels(feature_tracker rvc2_all rvc4 rvc4rgb ci)

# Object tracker
add_python_example(object_tracker ObjectTracker/object_tracker.py)
dai_set_example_test_labels(object_tracker ondevice rvc2_all rvc4 ci)
add_python_example(object_tracker_replay ObjectTracker/object_tracker_replay.py)
dai_set_example_test_labels(object_tracker_replay ondevice rvc2_all rvc4 ci)
add_python_example(object_tracker_remap ObjectTracker/object_tracker_remap.py)
dai_set_example_test_labels(object_tracker_remap ondevice rvc2_all rvc4 ci)

# ImageFilters
add_python_example(image_filters StereoDepth/stereo_depth_filters.py)
dai_set_example_test_labels(image_filters rvc2_all rvc4 ci)

# DynamicCalibration
if(DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT)
    add_python_example(calibration_integration DynamicCalibration/calibration_integration.py)
    dai_set_example_test_labels(calibration_integration ondevice rvc2_all rvc4 ci)

    add_python_example(calibration_quality_dynamic DynamicCalibration/calibration_quality_dynamic.py)
    dai_set_example_test_labels(calibration_quality_dynamic ondevice rvc2_all rvc4 ci)

    add_python_example(calibration_dynamic DynamicCalibration/calibration_dynamic.py)
    dai_set_example_test_labels(calibration_dynamic ondevice rvc2_all rvc4 ci)
endif()
