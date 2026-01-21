# Add CMake testing infrastructure
include(CTest)
enable_testing()

# Add Catch2 for writing tests
find_package(Catch2 CONFIG REQUIRED)

# Print details of the Catch2 package
message(STATUS "Catch2_FOUND: ${Catch2_FOUND}")
message(STATUS "Catch2_VERSION: ${Catch2_VERSION}")
message(STATUS "Catch2_INCLUDE_DIRS: ${Catch2_INCLUDE_DIRS}")
message(STATUS "Catch2_LIBRARIES: ${Catch2_LIBRARIES}")

set(test_env
    # Misc
    "UBSAN_OPTIONS=halt_on_error=1"
    # DepthAI
    "DEPTHAI_SEARCH_TIMEOUT=15000"
    "DEPTHAI_RECONNECT_TIMEOUT=0"
    )

include(CMakeParseArguments)

include_directories(${CMAKE_SOURCE_DIR}/src)

# Function for adding new tests
function(dai_add_test test_name test_src)
    # Parse arguments
    cmake_parse_arguments(DAT "CONFORMING" "CXX_STANDARD" "" ${ARGN})

    # Create test executable
    add_executable(${test_name} ${test_src})
    add_default_flags(${test_name} LEAN)

    # Set compiler C++ standard if specified
    if(DAT_CXX_STANDARD)
        target_compile_features(${test_name} PRIVATE cxx_std_${DAT_CXX_STANDARD})
    endif()

    # Add conforming preprocessor flags for MSVC if requested
    if(MSVC AND DAT_CONFORMING)
        if(MSVC_VERSION LESS 1925)
            target_compile_options(${test_name} PRIVATE "/experimental:preprocessor")
        else()
            target_compile_options(${test_name} PRIVATE "/Zc:preprocessor")
        endif()
    endif()

    # Add to clang-format target
    if(COMMAND target_clangformat_setup)
        target_clangformat_setup(${test_name} "")
    endif()

    # Link to core and Catch2 testing framework
    set(DEPTHAI_TARGET depthai::core)
    if(NOT DEPTHAI_MERGED_TARGET)
        set(DEPTHAI_TARGET depthai::opencv)
    endif()
    target_link_libraries(${test_name} PRIVATE ${DEPTHAI_TARGET} ${OpenCV_LIBS} Catch2::Catch2WithMain Threads::Threads spdlog::spdlog)

    # Add sanitizers for tests as well
    if(COMMAND add_sanitizers)
        add_sanitizers(${test_name})
    endif()

    # Add test
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES ENVIRONMENT "${test_env}")


    # Copy over required DLLs (Windows)
    if(WIN32)
        # Copy DLLs to target directory - Windows only
        # TARGET_RUNTIME_DLLS generator expression available since CMake 3.21
        if(CMAKE_VERSION VERSION_LESS "3.21")
            file(GLOB depthai_dll_libraries "${HUNTER_INSTALL_PREFIX}/bin/*.dll")
        else()
            set(depthai_dll_libraries "$<TARGET_RUNTIME_DLLS:${test_name}>")
        endif()
        add_custom_command(TARGET ${test_name} POST_BUILD COMMAND
            "$<$<BOOL:${depthai_dll_libraries}>:${CMAKE_COMMAND};-E;copy_if_different;${depthai_dll_libraries};$<TARGET_FILE_DIR:${test_name}>>"
            COMMAND_EXPAND_LISTS
            VERBATIM
        )
    endif()
endfunction()

# Function for setting test labels, including label shortcuts
function(dai_set_test_labels test_name)
    set(expanded_labels "")
    foreach(label IN LISTS ARGN)
        if(label STREQUAL "rvc2_all")
            list(APPEND expanded_labels "rvc2" "poe" "usb")
        else()
            list(APPEND expanded_labels ${label})
        endif()
    endforeach()
    set_tests_properties(${test_name} PROPERTIES LABELS "${expanded_labels}")
endfunction()

# Function for adding compile definitions to tests
function(dai_test_compile_definitions)
    if(TARGET ${ARGV0})
        target_compile_definitions(${ARGV})
        if(TARGET ${ARGV0}_conforming)
            set(ARGV0 ${ARGV0}_conforming)
            list(REMOVE_AT ARGV 0)
            list(INSERT ARGV 0 ${ARGV0})
            target_compile_definitions(${ARGV})
        endif()
    endif()
endfunction()

if(DEPTHAI_FETCH_ARTIFACTS)
    # Mobilenet network
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_8shave.blob"
        SHA1 "3329bb8f3a9c881ef9756d232055f9d6f38aa07b"
        FILE "mobilenet-ssd_openvino_2021.2_8shave.blob"
        LOCATION mobilenet_blob
    )

    # OpenVINO 2020.3 blob
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2020.3_4shave.blob"
        SHA1 "f0134c9b843fe414f6d98b17a70f069d1ab0f3d8"
        FILE "text-image-super-resolution-0001_2020.3_4shave.blob"
        LOCATION openvino_2020_3_blob
    )
    # OpenVINO 2020.4 blob
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2020.4_4shave.blob"
        SHA1 "25dcf0b146da8c85c9c4cba00ad5fdd4ed02a1b6"
        FILE "text-image-super-resolution-0001_2020.4_4shave.blob"
        LOCATION openvino_2020_4_blob
    )

    # OpenVINO 2021.1 blob
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.1_4shave.blob"
        SHA1 "39c4f47f2a75627b7561e97dd7cdfcd0b1925a1e"
        FILE "text-image-super-resolution-0001_2021.1_4shave.blob"
        LOCATION openvino_2021_1_blob
    )
    # OpenVINO 2021.2 blob
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.2_4shave.blob"
        SHA1 "a204467f86aa4ad63d31782ada271bea6f57f789"
        FILE "text-image-super-resolution-0001_2021.2_4shave.blob"
        LOCATION openvino_2021_2_blob
    )
    # OpenVINO 2021.3 blob
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.3_4shave.blob"
        SHA1 "af19470feb59317e74d045bc31d93ca129c46674"
        FILE "text-image-super-resolution-0001_2021.3_4shave.blob"
        LOCATION openvino_2021_3_blob
    )
    # OpenVINO 2021.4.2 blob
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.4.2_4shave.blob"
        SHA1 "164b6b2ae48d38bc4f07cc8296b8bcb7644a1578"
        FILE "text-image-super-resolution-0001_2021.4.2_4shave.blob"
        LOCATION openvino_2021_4_2_blob
    )
    # OpenVINO 2022.1.0 blob
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2022.1.0_4shave.blob"
        SHA1 "98e94b865b9c48a92eaebd1ddc883712dfe7cfcb"
        FILE "text-image-super-resolution-0001_2022.1.0_4shave.blob"
        LOCATION openvino_2022_1_blob
    )
    # YoloV4 resource
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_4shave.blob"
        SHA1 "7da2f96f7300e3828940557e6a86ac6f243eef7e"
        FILE "yolo-v4-tiny-tf_openvino_2021.4_4shave.blob"
        LOCATION tiny_yolo_v4_2021-4_4shave_blob
    )

    # Superblob
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/person-reidentification-retail-0277_openvino_2022.1_8shave.superblob"
        SHA1 "6f14e3a5388946d6de849ff4f6432702601b1003"
        FILE "person-reidentification-retail-0277_openvino_2022.1_8shave.superblob"
        LOCATION superblob_path
    )

    # NNarchives of different types
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/nnarchive/yolo_blob_nnarchive.tar.xz"
        SHA1 "6b9697e5aaf1560efbdea7f8d7487bae51def619"
        FILE "yolo_blob_nnarchive.tar.xz"
        LOCATION yolo_blob_nnarchive_path
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/nnarchive/yolo_superblob_nnarchive.tar.xz"
        SHA1 "2356fd7e5a203446211a891a74f0b23e0912eaf6"
        FILE "yolo_superblob_nnarchive.tar.xz"
        LOCATION yolo_superblob_nnarchive_path
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/nnarchive/yolo_onnx_nnarchive.tar.xz"
        SHA1 "7abb6a8c05c5f66897cd2c5c6b4f0170620dff8b"
        FILE "yolo_onnx_nnarchive.tar.xz"
        LOCATION yolo_onnx_nnarchive_path
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/images/lenna.png"
        SHA1 "3ee0d360dc12003c0d43e3579295b52b64906e85"
        FILE "lenna.png"
        LOCATION lenna_png
    )
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/recording.tar"
        SHA1 "b1e31a26c83dc1e315132c9226097da4b1a5cbb7"
        FILE "recording.tar"
        LOCATION recording_path
    )
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/construction_vest.mp4"
        SHA1 "271d8d0b702e683ce02957db7c100843de5ceaec"
        FILE "construction_vest.mp4"
        LOCATION construction_vest
    )
    private_data(
        URL "https://artifacts.luxonis.com/artifactory//luxonis-depthai-data-local/misc/dynamic_calibration_test_data.tar"
        SHA1 "520d97199757f6fa0497c4037dcca5a43c7be3de"
        FILE "dynamic_calibration_test_data.tar"
        LOCATION dynamic_calibration_test_data
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolov6-nano_r2-coco-512x288_a26d1ee-detections.json"
        SHA1 "954ae635625e14cb5e73fd20f3c01eec31d0f97c"
        FILE "yolov6-nano_r2-coco-512x288_a26d1ee-detections.json"
        LOCATION yolo_v6_r2_coco_512x288_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolov6-nano_r2-coco-512x384_fb1429e-detections.json"
        SHA1 "af6cea1f293622fcc2c9cb8d78fadb510a2270c9"
        FILE "yolov6-nano_r2-coco-512x384_fb1429e-detections.json"
        LOCATION yolo_v6_r2_coco_512x384_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolov8-instance-segmentation-nano_coco-512x288_6c0402a-detections.json"
        SHA1 "bb29fb9a6bc041f7e90769df4cde3a6ca890e521"
        FILE "yolov8-instance-segmentation-nano_coco-512x288_6c0402a-detections.json"
        LOCATION yolo_v8_instance_segmentation_nano_coco_512x288_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolov8-large-pose-estimation_coco-640x352_1868e39-detections.json"
        SHA1 "397fd5e1b80f47050cb827af92b5b13fb4f35bc6"
        FILE "yolov8-large-pose-estimation_coco-640x352_1868e39-detections.json"
        LOCATION yolo_v8_large_pose_estimation_coco_640x352_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolov8-instance-segmentation-large_coco-640x352_701031f-detections.json"
        SHA1 "21ee42c2f560d7f977f1895387f6e6834c7dd2b5"
        FILE "yolov8-instance-segmentation-large_coco-640x352_701031f-detections.json"
        LOCATION yolo_v8_instance_segmentation_large_coco_640x352_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolov10-nano_coco-512x288_007b5fe-detections.json"
        SHA1 "3530b6c02a2cb8f18220de2dcb78c1e4d5fd4733"
        FILE "yolov10-nano_coco-512x288_007b5fe-detections.json"
        LOCATION yolo_v10_nano_coco_512x288_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/ppe-detection_640x640_419a4e5-detections.json"
        SHA1 "aff07108511958fcfa281bc8ce25dd0647d63ab5"
        FILE "ppe-detection_640x640_419a4e5-detections.json"
        LOCATION ppe_detection_640x640_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolo-p_bdd100k-320x320_7019bb8-detections.json"
        SHA1 "ec6b99aeb87a22ab98cb0b042a83ed7d4ae7756b"
        FILE "yolo-p_bdd100k-320x320_7019bb8-detections.json"
        LOCATION yolo_p_bdd100k_320x320_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/fire-detection_512x288_4a8263c-detections.json"
        SHA1 "a0fbdd1b8a92a041f7b8e79ce60fd7aa8fa2c36a"
        FILE "fire-detection_512x288_4a8263c-detections.json"
        LOCATION fire_detection_512x288_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/videos/pedestrians_walking_4k_25fps.mp4"
        SHA1 "9e4708d5579aa6b8e82f49d6999302eec6a5076b"
        FILE "pedestrians_walking_4k_25fps.mp4"
        LOCATION people_walking_video
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/videos/fire_hd_1920_1080_24fps.mp4"
        SHA1 "98bfa29dd30c10b58118e3e0a2d32e3f8c18b12c"
        FILE "fire_hd_1920_1080_24fps.mp4"
        LOCATION fire_video
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/images/kitchen1.png"
        SHA1 "e566d4d0dce0b30a3e4cdc3b693e1c6385867822"
        FILE "kitchen1.png"
        LOCATION kitchen_image
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolov8-instance-segmentation-large_coco-640x352_701031f-segmentation_mask.png"
        SHA1 "f837f89294ba8ab4ae2d30b75d1e2b141a930f8c"
        FILE "yolov8-instance-segmentation-large_coco-640x352_701031f-segmentation_mask.png"
        LOCATION yolo_v8_instance_segmentation_large_coco_640x352_kitchen_segmentation_gt
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/parser_testing/ground_truth_files/detection_parser/yolov8-instance-segmentation-large_coco-640x352_701031f-segmentation_mask_v2.png"
        SHA1 "cf947acf1c210efba34c363fe6d8fd8d02293601"
        FILE "yolov8-instance-segmentation-large_coco-640x352_701031f-segmentation_mask_v2.png"
        LOCATION yolo_v8_instance_segmentation_large_coco_640x352_kitchen_segmentation_gt_v2
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/test_recording.tar"
        SHA1 "5f93fba780c07ab858c8d5bef3a591078e021ff5"
        FILE "test_recording.tar"
        LOCATION test_recording
    )

    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/test_calib.json"
        SHA1 "d745e1cdf18dda1ef00311250676407df207df4c"
        FILE "test_calib.json"
        LOCATION test_calib
    )
endif()
### Helper script to wrap the tests with a timeout ############################

add_executable(test_wrapper src/helpers/test_wrapper.cpp)
target_include_directories(test_wrapper PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include/subprocess)
target_link_libraries(test_wrapper PRIVATE depthai::core Threads::Threads spdlog::spdlog)

### On-host tests #############################################################

# Superblob test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(openvino_test src/onhost_tests/openvino/openvino_test.cpp)
    target_compile_definitions(openvino_test PRIVATE SUPERBLOB_PATH="${superblob_path}")
    dai_set_test_labels(openvino_test onhost ci)
endif()

# CalibrationHandler test
dai_add_test(calibration_handler_test src/onhost_tests/calibration_handler_test.cpp)
dai_set_test_labels(calibration_handler_test onhost ci)

# NNArchive test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(nn_archive_test src/onhost_tests/nn_archive/nn_archive_test.cpp)
    target_compile_definitions(nn_archive_test PRIVATE
        BLOB_ARCHIVE_PATH="${yolo_blob_nnarchive_path}"
        SUPERBLOB_ARCHIVE_PATH="${yolo_superblob_nnarchive_path}"
        ONNX_ARCHIVE_PATH="${yolo_onnx_nnarchive_path}"
    )
    dai_set_test_labels(nn_archive_test onhost ci)
endif()

# Eeprom naming parsing tests
dai_add_test(naming_test src/onhost_tests/naming_test.cpp)
dai_set_test_labels(naming_test onhost ci)

# Image transformations test
dai_add_test(image_transformations_test src/onhost_tests/image_transformations_test.cpp)
dai_set_test_labels(image_transformations_test onhost ci)

# Normalization tests
dai_add_test(normalization_test src/onhost_tests/normalization_test.cpp)
dai_set_test_labels(normalization_test onhost ci)

# Depth unit tests
dai_add_test(depth_unit_test src/onhost_tests/depth_unit_test.cpp)
dai_set_test_labels(depth_unit_test onhost ci)

# MessageQueue tests
dai_add_test(message_queue_test src/onhost_tests/message_queue_test.cpp)
dai_set_test_labels(message_queue_test onhost ci)

# StreamMessageParser tests
dai_add_test(stream_message_parser_test src/onhost_tests/stream_message_parser_test.cpp)
dai_set_test_labels(stream_message_parser_test onhost ci)

# Bootloader version tests
dai_add_test(bootloader_version_test src/onhost_tests/bootloader_version_test.cpp)
dai_set_test_labels(bootloader_version_test onhost ci)

# Bootloader configuration tests
dai_add_test(bootloader_config_test src/onhost_tests/bootloader_config_test.cpp)
dai_set_test_labels(bootloader_config_test onhost ci)

# Environment test
dai_add_test(env_test src/onhost_tests/utility/env_test.cpp)
dai_set_test_labels(env_test onhost ci)

## Dummy filesystem lock process for `platform_test`
add_executable(fslock_dummy src/onhost_tests/utility/fslock_dummy.cpp)
add_default_flags(fslock_dummy LEAN)
target_link_libraries(fslock_dummy PRIVATE depthai::core)

# Platform tests
dai_add_test(platform_test src/onhost_tests/utility/platform_test.cpp)
dai_set_test_labels(platform_test onhost ci)
target_compile_definitions(platform_test PRIVATE FSLOCK_DUMMY_PATH="$<TARGET_FILE:fslock_dummy>")
add_dependencies(platform_test fslock_dummy)

# Circular buffer test
dai_add_test(circular_buffer_test src/onhost_tests/utility/circular_buffer_test.cpp)
dai_set_test_labels(circular_buffer_test onhost ci)

# Pipeline debugging tests
dai_add_test(pipeline_debugging_host_test src/onhost_tests/pipeline_debugging_host_test.cpp)
dai_set_test_labels(pipeline_debugging_host_test onhost ci)

# Datatype tests
dai_add_test(nndata_test src/onhost_tests/pipeline/datatype/nndata_test.cpp)
dai_set_test_labels(nndata_test onhost ci)

#ImgDetections tests
dai_add_test(imgdetections_test src/onhost_tests/pipeline/datatype/imgdetections_test.cpp)
dai_set_test_labels(imgdetections_test onhost ci)

#ImgFrame tests
dai_add_test(imgframe_test src/onhost_tests/pipeline/datatype/imgframe_test.cpp)
dai_set_test_labels(imgframe_test onhost ci)

# Model description tests
dai_add_test(model_slug_test src/onhost_tests/model_slug_test.cpp)
dai_set_test_labels(model_slug_test onhost ci)

# Remote connection tests
if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    dai_add_test(remote_connection_test src/onhost_tests/remote_connection_test.cpp)
    dai_set_test_labels(remote_connection_test onhost ci nowindows)
endif()
### On-device tests ###########################################################

# Camera stream restart test
dai_add_test(camera_stream_restart_test src/ondevice_tests/camera_stream_restart_test.cpp)
dai_set_test_labels(camera_stream_restart_test ondevice rvc2_all rvc4 rvc4rgb ci)

# DepthAI logging test
dai_add_test(logging_test src/ondevice_tests/logging_test.cpp)
dai_set_test_labels(logging_test ondevice rvc2_all rvc4 rvc4rgb ci)

# OpenVINO blob test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(openvino_blob_test src/ondevice_tests/openvino_blob_test.cpp)
    target_compile_definitions(openvino_blob_test PRIVATE
        OPENVINO_2020_3_BLOB_PATH="${openvino_2020_3_blob}"
        OPENVINO_2020_4_BLOB_PATH="${openvino_2020_4_blob}"
        OPENVINO_2021_1_BLOB_PATH="${openvino_2021_1_blob}"
        OPENVINO_2021_2_BLOB_PATH="${openvino_2021_2_blob}"
        OPENVINO_2021_3_BLOB_PATH="${openvino_2021_3_blob}"
        OPENVINO_2021_4_BLOB_PATH="${openvino_2021_4_2_blob}"
        OPENVINO_2022_1_BLOB_PATH="${openvino_2022_1_blob}"
    )
    dai_set_test_labels(openvino_blob_test ondevice rvc2_all ci)
endif()

# Vpp node tests
dai_add_test(vpp_test src/ondevice_tests/vpp_test.cpp)
dai_set_test_labels(vpp_test ondevice rvc4 ci)

# Dynamic calibration tests
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(dynamic_calibration_test src/ondevice_tests/dynamic_calibration_test.cpp)
    dai_set_test_labels(dynamic_calibration_test ondevice rvc2_all rvc4 ci)
    target_compile_definitions(dynamic_calibration_test PRIVATE RECORDING_PATH="${dynamic_calibration_test_data}")
endif()

dai_add_test(dynamic_calibration_onhost_test src/onhost_tests/dynamic_calibration_test.cpp)
dai_set_test_labels(dynamic_calibration_onhost_test onhost)

dai_add_test(flash_eeprom_fields_test src/ondevice_tests/flash_eeprom_fields_test.cpp)
dai_set_test_labels(flash_eeprom_fields_test ondevice rvc2_all rvc4)

# Neural network test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(neural_network_test src/ondevice_tests/neural_network_test.cpp)
    target_compile_definitions(neural_network_test PRIVATE BLOB_PATH="${mobilenet_blob}")
    dai_set_test_labels(neural_network_test ondevice rvc2_all ci)
endif()

dai_add_test(spatial_location_calculator_test src/ondevice_tests/pipeline/node/spatial_location_calculator_test.cpp)
dai_set_test_labels(spatial_location_calculator_test ondevice rvc2_all rvc4 ci)

# Color camera test
dai_add_test(color_camera_node_test src/ondevice_tests/color_camera_node_test.cpp)
dai_set_test_labels(color_camera_node_test ondevice rvc2_all ci)

# Pipeline test
dai_add_test(pipeline_test src/ondevice_tests/pipeline_test.cpp)
dai_set_test_labels(pipeline_test ondevice rvc2_all ci)

# Pipeline debugging tests
dai_add_test(pipeline_debugging_rvc4_test src/ondevice_tests/pipeline_debugging_rvc4_test.cpp)
dai_set_test_labels(pipeline_debugging_rvc4_test rvc4 ci)
target_compile_definitions(pipeline_debugging_rvc4_test PRIVATE VIDEO_PATH="${construction_vest}")
dai_add_test(pipeline_debugging_rvc2_test src/ondevice_tests/pipeline_debugging_rvc2_test.cpp)
dai_set_test_labels(pipeline_debugging_rvc2_test rvc2 ci)
target_compile_definitions(pipeline_debugging_rvc2_test PRIVATE VIDEO_PATH="${construction_vest}")

# Device USB Speed and serialization macros test
dai_add_test(device_usbspeed_test src/ondevice_tests/device_usbspeed_test.cpp)
dai_set_test_labels(device_usbspeed_test ondevice rvc2 usb ci)

dai_add_test(device_usbspeed_test_17 src/ondevice_tests/device_usbspeed_test.cpp CXX_STANDARD 17)
dai_set_test_labels(device_usbspeed_test_17 ondevice rvc2 usb ci)

dai_add_test(device_usbspeed_test_20 src/ondevice_tests/device_usbspeed_test.cpp CXX_STANDARD 20)
dai_set_test_labels(device_usbspeed_test_20 ondevice rvc2 usb ci)

# Filesystem test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(filesystem_test src/ondevice_tests/filesystem_test.cpp)
    target_compile_definitions(filesystem_test PRIVATE BLOB_PATH="${mobilenet_blob}")
    dai_set_test_labels(filesystem_test rvc2_all ci)

    dai_add_test(filesystem_test_17 src/ondevice_tests/filesystem_test.cpp CXX_STANDARD 17)
    target_compile_definitions(filesystem_test_17 PRIVATE BLOB_PATH="${mobilenet_blob}")
    dai_set_test_labels(filesystem_test_17 rvc2_all ci)

    dai_add_test(filesystem_test_20 src/ondevice_tests/filesystem_test.cpp CXX_STANDARD 20)
    target_compile_definitions(filesystem_test_20 PRIVATE BLOB_PATH="${mobilenet_blob}")
    dai_set_test_labels(filesystem_test_20 rvc2_all ci)
endif()

# Encoded frame test
dai_add_test(encoded_frame_test src/ondevice_tests/encoded_frame_test.cpp CXX_STANDARD 17)
dai_set_test_labels(encoded_frame_test ondevice rvc2_all rvc4 rvc4rgb ci)

# MessageGroup tests
dai_add_test(message_group_frame_test src/ondevice_tests/message_group_test.cpp CXX_STANDARD 17)
dai_set_test_labels(message_group_frame_test ondevice rvc2_all rvc4 ci)

# Pointcloud test
dai_add_test(pointcloud_test src/ondevice_tests/pointcloud_test.cpp)
dai_set_test_labels(pointcloud_test ondevice rvc2_all ci)

# RGBD test
dai_add_test(rgbd_test src/ondevice_tests/rgbd_test.cpp)
dai_set_test_labels(rgbd_test ondevice rvc2_all rvc4 ci)

# Input/Output naming test
dai_add_test(input_output_naming_test src/ondevice_tests/input_output_naming_test.cpp)
dai_set_test_labels(input_output_naming_test ondevice rvc2_all rvc4 rvc4rgb ci)

# Resolutions test
dai_add_test(resolutions_test src/ondevice_tests/resolutions_test.cpp)
dai_set_test_labels(resolutions_test ondevice) # TODO(jakob) Make the test runnable in CI

# Serialization test
dai_add_test(serialization_test src/onhost_tests/serialization_test.cpp)
dai_set_test_labels(serialization_test ondevice rvc2_all ci)

# Subnode test
dai_add_test(subnode_test src/ondevice_tests/pipeline/subnode_test.cpp)
dai_set_test_labels(subnode_test ondevice rvc2_all rvc4 rvc4rgb ci)

# Detection network test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(detection_network_test src/ondevice_tests/pipeline/node/detection_network_test.cpp)
    target_compile_definitions(detection_network_test PRIVATE
        BLOB_ARCHIVE_PATH="${yolo_blob_nnarchive_path}"
        SUPERBLOB_ARCHIVE_PATH="${yolo_superblob_nnarchive_path}"
        ONNX_ARCHIVE_PATH="${yolo_onnx_nnarchive_path}"
    )
    dai_set_test_labels(detection_network_test ondevice rvc2_all ci)
endif()

# DetectionParser test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(detection_parser_test src/ondevice_tests/pipeline/node/detection_parser_test.cpp)
    target_compile_definitions(detection_parser_test PRIVATE
        YOLO_V6_R2_COCO_512x288_GROUND_TRUTH="${yolo_v6_r2_coco_512x288_gt}"
        YOLO_V6_R2_COCO_512x384_GROUND_TRUTH="${yolo_v6_r2_coco_512x384_gt}"
        YOLO_V8_INSTANCE_SEGMENTATION_NANO_COCO_512x288_GROUND_TRUTH="${yolo_v8_instance_segmentation_nano_coco_512x288_gt}"
        YOLO_V8_LARGE_POSE_ESTIMATION_COCO_640x352_GROUND_TRUTH="${yolo_v8_large_pose_estimation_coco_640x352_gt}"
        YOLO_V8_INSTANCE_SEGMENTATION_LARGE_COCO_640x352_GROUND_TRUTH="${yolo_v8_instance_segmentation_large_coco_640x352_gt}"
        YOLO_V10_NANO_COCO_512x288_GROUND_TRUTH="${yolo_v10_nano_coco_512x288_gt}"
        PPE_DETECTION_640x640_GROUND_TRUTH="${ppe_detection_640x640_gt}"
        YOLO_P_BDD100K_320x320_GROUND_TRUTH="${yolo_p_bdd100k_320x320_gt}"
        FIRE_DETECTION_512x288_GROUND_TRUTH="${fire_detection_512x288_gt}"
        PEOPLE_WALKING_VIDEO="${people_walking_video}"
        FIRE_VIDEO="${fire_video}"
        KITCHEN_IMAGE_PATH="${kitchen_image}"
        YOLO_V8_INSTANCE_SEGMENTATION_LARGE_COCO_640x352_KITCHEN_SEGMENTATION_GROUND_TRUTH="${yolo_v8_instance_segmentation_large_coco_640x352_kitchen_segmentation_gt_v2}"
    )
    dai_set_test_labels(detection_parser_test ondevice rvc4 ci)
endif()

# Spatial detection network test
dai_add_test(spatial_detection_network_test src/ondevice_tests/pipeline/node/spatial_detection_network_test.cpp)
dai_set_test_labels(spatial_detection_network_test ondevice rvc2_all rvc4 ci)

# NeuralNetwork node test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(neural_network_node_test src/ondevice_tests/pipeline/node/neural_network_node_test.cpp)
    target_compile_definitions(neural_network_node_test PRIVATE LENNA_PATH="${lenna_png}")
    dai_set_test_labels(neural_network_node_test ondevice rvc2_all rvc4 ci)
endif()

# ImgTransformations tests
dai_add_test(img_transformation_test src/ondevice_tests/img_transformation_test.cpp CXX_STANDARD 17)
dai_set_test_labels(img_transformation_test ondevice rvc2_all rvc4 ci)

# Regression tests for encountered bugs
dai_add_test(regression_camera_concurrency_test src/ondevice_tests/regression/camera_concurrency.cpp)
dai_set_test_labels(regression_camera_concurrency_test ondevice rvc4 ci)

dai_add_test(regression_message_group_test src/ondevice_tests/regression/message_group.cpp)
dai_set_test_labels(regression_message_group_test ondevice rvc2_all rvc4 rvc4rgb ci)

# Sync test
dai_add_test(sync_test src/ondevice_tests/pipeline/node/sync_test.cpp)
dai_set_test_labels(sync_test ondevice rvc2_all rvc4 ci)

# Fsync test
dai_add_test(fsync_test src/ondevice_tests/pipeline/node/fsync_test.cpp)
dai_set_test_labels(fsync_test ondevice rvc2 usb rvc4 ci)

## Large messages tests
dai_add_test(bridge_large_messages src/ondevice_tests/bridge_large_messages.cpp)
# Only test on RVC4, as RVC2 is not yet supported to auto-increase the max stream capacity
dai_set_test_labels(bridge_large_messages ondevice rvc4 rvc4rgb ci)
# ImageAlign test
dai_add_test(image_align_node_test src/ondevice_tests/image_align_node_test.cpp)
dai_set_test_labels(image_align_node_test ondevice rvc2_all rvc4 ci)

# Script node test
dai_add_test(script_node_test src/ondevice_tests/script_node_test.cpp)
dai_set_test_labels(script_node_test ondevice rvc2_all rvc4 rvc4rgb ci)

# StereoDepth test
dai_add_test(stereo_depth_node_test src/ondevice_tests/stereo_depth_node_test.cpp)
dai_set_test_labels(stereo_depth_node_test ondevice rvc2_all rvc4 ci)

# ImageManip test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(image_manip_node_test src/ondevice_tests/pipeline/node/image_manip_test.cpp)
    target_compile_definitions(image_manip_node_test PRIVATE LENNA_PATH="${lenna_png}")
    dai_set_test_labels(image_manip_node_test ondevice rvc2_all rvc4 rvc4rgb ci)

    dai_add_test(image_manip_optimization_test src/ondevice_tests/pipeline/node/image_manip_optimization_test.cpp)
    target_compile_definitions(image_manip_optimization_test PRIVATE LENNA_PATH="${lenna_png}")
    dai_set_test_labels(image_manip_optimization_test ondevice rvc4 rvc4rgb ci)
endif()

# Benchmark tests
dai_add_test(benchmark_test src/ondevice_tests/pipeline/node/benchmark_test.cpp)
dai_set_test_labels(benchmark_test ondevice rvc2_all rvc4 rvc4rgb ci)

# SystemLogger test
dai_add_test(system_logger_test src/ondevice_tests/pipeline/node/system_logger_test.cpp)
dai_set_test_labels(system_logger_test ondevice rvc2_all rvc4 rvc4rgb ci)

# Manual focus test
dai_add_test(manual_focus_test src/ondevice_tests/manual_focus_test.cpp)
dai_set_test_labels(manual_focus_test rvc2_all rvc4 rvc4rgb ci)

# IMU tests
dai_add_test(imu_test src/ondevice_tests/pipeline/node/imu_test.cpp)
dai_set_test_labels(imu_test ondevice rvc4 rvc4rgb ci) # Many RVC2 devices do not have an IMU which supports the whole test suite

# ObjectTracker tests
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(object_tracker_test src/ondevice_tests/pipeline/node/object_tracker_test.cpp)
    dai_set_test_labels(object_tracker_test ondevice rvc2 rvc4 ci)
    target_compile_definitions(object_tracker_test PRIVATE VIDEO_PATH="${construction_vest}")
endif()

# Record & Replay tests
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(record_replay_test src/ondevice_tests/pipeline/node/record_replay_test.cpp)
    dai_set_test_labels(record_replay_test ondevice rvc2_all rvc4) # TODO(Morato) add to CI once the test is stable
    target_compile_definitions(record_replay_test PRIVATE RECORDING_PATH="${recording_path}")
    dai_add_test(replay_test src/onhost_tests/replay_test.cpp)
    dai_set_test_labels(replay_test onhost ci)
    target_compile_definitions(replay_test PRIVATE RECORDING_PATH="${recording_path}")
endif()

# Camera tests
dai_add_test(camera_test src/ondevice_tests/pipeline/node/camera_test.cpp)
dai_set_test_labels(camera_test ondevice rvc2_all rvc4 ci)

dai_add_test(camera_fps_config_test src/ondevice_tests/pipeline/node/camera_fps_config_test.cpp)
dai_set_test_labels(camera_fps_config_test ondevice rvc2_all rvc4 ci)

# VideoEncoder test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(video_encoder_test src/ondevice_tests/video_encoder_test.cpp)
    dai_set_test_labels(video_encoder_test ondevice rvc2 usb rvc4 rvc4rgb ci)
    target_compile_definitions(video_encoder_test PRIVATE VIDEO_PATH="${construction_vest}")
endif()

# XLinkIn test
dai_add_test(xlink_test src/ondevice_tests/xlink_test.cpp)
dai_set_test_labels(xlink_test ondevice rvc2_all rvc4 rvc4rgb ci)

# runtime_calibration_test test
dai_add_test(runtime_calibration_test src/ondevice_tests/runtime_calibration_test.cpp)
dai_set_test_labels(runtime_calibration_test ondevice rvc2_all rvc4 rvc4rgb ci)

# NeuralDepth test
if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_test(neural_depth_node_test src/ondevice_tests/neural_depth_node_test.cpp)
    dai_set_test_labels(neural_depth_node_test ondevice rvc4 ci)
    target_compile_definitions(neural_depth_node_test PRIVATE
        NEURAL_REPLAY_PATH="${test_recording}"
        NEURAL_CALIBRATION_PATH="${test_calib}"
    )
endif()

# NeuralAssistedStereo test
dai_add_test(neural_assisted_stereo_node_test src/ondevice_tests/pipeline/node/neural_assisted_stereo_node_test.cpp)
dai_set_test_labels(neural_assisted_stereo_node_test ondevice rvc4 ci)
