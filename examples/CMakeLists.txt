# Add an option to test examples
option(DEPTHAI_TEST_EXAMPLES "Test examples - examples will be ran as a part of test suite" OFF)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Sanitizers)

# Create utility library
add_library(utility src/utility.cpp)
target_link_libraries(utility FP16::fp16 ${OpenCV_LIBS})

# Helper for adding new examples
function(dai_add_example example_name example_src enable_test)

    # Add example
    add_executable(${example_name} ${example_src})
    target_link_libraries(${example_name} PRIVATE utility depthai::opencv ${OpenCV_LIBS})

    # Add sanitizers for example
    add_sanitizers(${example_name})
    # Add to clangformat target
    target_clangformat_setup(${example_name} "")

    # parse the rest of the arguments
    set(arguments ${ARGV})
    list(REMOVE_AT arguments 0 1 2)

    # If 'DEPTHAI_TEST_EXAMPLES' is ON, then examples will be part of the test suite
    if(${enable_test} AND DEPTHAI_TEST_EXAMPLES)
        add_test(NAME ${example_name} COMMAND ${CMAKE_COMMAND}
            -DTIMEOUT_SECONDS=10
            -P ${CMAKE_CURRENT_LIST_DIR}/cmake/ExecuteTestTimeout.cmake $<TARGET_FILE:${example_name}> ${arguments}
        )

        # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
        set_tests_properties(${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION "\\[warning\\];\\[error\\];\\[critical\\]")

        # Add ubsan halt on error
        set_tests_properties(${example_name} PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1")

    endif()
endfunction()

# Create a custom target which runs all examples for 10 seconds max, and check if they executed without errors



# Add any custom binary data using the following snippet
## # Hunter test data download
## hunter_private_data(
##     URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet.blob"
##     SHA1 "e89d3ee9f26d80397e44f89c6b375990064a4a42"
##     FILE "mobilenet.blob"
##     LOCATION mobilenet_blob
## )


# Hunter test data download

# NeuralNetwork node, mobilenet example
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_6shave.blob"
    SHA1 "4f4506726e3083981064938a0faaf9af6180d2c6"
    FILE "mobilenet-ssd_openvino_2021.4_6shave.blob"
    LOCATION mobilenet_blob
)

# YoloV3 resource
hunter_private_data(
    URL "http://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
    SHA1 "dedb2d4d96b23e42d15c15e454b8f02eca2713de"
    FILE "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
    LOCATION tiny_yolo_v3_blob
)

# YoloV4 resource
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob"
    SHA1 "d8d09b697dac298fe83cf8856740a21b1a61ab89"
    FILE "yolo-v4-tiny-tf_openvino_2021.4_6shave.blob"
    LOCATION tiny_yolo_v4_blob
)

# NeuralNetwork node, mobilenet example, 5 shaves
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_5shave.blob"
    SHA1 "14ab3ef446b8a649d6694d94c2fc8b798efd74a3"
    FILE "mobilenet-ssd_openvino_2021.4_5shave.blob"
    LOCATION mobilenet_5shaves_blob
)

# NeuralNetwork node, mobilenet example, 8 shaves
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_8shave.blob"
    SHA1 "7e7ea9a680668f3ca9e8f1d22cc8f3f1a02ac018"
    FILE "mobilenet-ssd_openvino_2021.4_8shave.blob"
    LOCATION mobilenet_8shaves_blob
)

# Video file with objects to detect
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/construction_vest.mp4"
    SHA1 "271d8d0b702e683ce02957db7c100843de5ceaec"
    FILE "construction_vest.mp4"
    LOCATION construction_vest
)

# Calibration Sample files
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/depthai_calib.json"
    SHA1 "ec50a47496473116c51cec1c8249bff2679383c3"
    FILE "depthai_calib.json"
    LOCATION calib_v6
)

hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/depthai_v5.calib"
    SHA1 "adfe9976b9aacfdaa76ac7138810999988177e71"
    FILE "depthai_v5.calib"
    LOCATION calib_v5
)

hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/BW1098OBC.json"
    SHA1 "1e84b6c339ac51e3b254775090b91235a40fac71"
    FILE "BW1098OBC.json"
    LOCATION device_config
)

# person-detection
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/person-detection-retail-0013_openvino_2021.4_7shave.blob"
    SHA1 "946d102874b287292cdb5952fa66356c8d36025f"
    FILE "person-detection-retail-0013_openvino_2021.4_7shave.blob"
    LOCATION person_detection
)

## message(STATUS "Location of test1.data: ${test1_data}")

# bootloader
dai_add_example(bootloader_version src/bootloader/bootloader_version.cpp ON)
dai_add_example(flash_bootloader src/bootloader/flash_bootloader.cpp OFF)
dai_add_example(bootloader_config src/bootloader/bootloader_config.cpp OFF)

# calibration
dai_add_example(calibration_flash src/calibration/calibration_flash.cpp OFF)
dai_add_example(calibration_flash_version5 src/calibration/calibration_flash_v5.cpp OFF)
dai_add_example(calibration_load src/calibration/calibration_load.cpp OFF)
dai_add_example(calibration_reader src/calibration/calibration_reader.cpp OFF)

target_compile_definitions(calibration_flash PRIVATE CALIB_PATH="${calib_v6}")
target_compile_definitions(calibration_flash_version5 PRIVATE CALIB_PATH="${calib_v5}" BOARD_PATH="${device_config}")
target_compile_definitions(calibration_load PRIVATE CALIB_PATH="${calib_v6}")

# ColorCamera
dai_add_example(rgb_camera_control src/ColorCamera/rgb_camera_control.cpp ON)
dai_add_example(rgb_preview src/ColorCamera/rgb_preview.cpp ON)
dai_add_example(rgb_video src/ColorCamera/rgb_video.cpp ON)

# EdgeDetector
dai_add_example(edge_detector src/EdgeDetector/edge_detector.cpp ON)

# FeatureTracker
dai_add_example(feature_detector src/FeatureTracker/feature_detector.cpp ON)
dai_add_example(feature_tracker src/FeatureTracker/feature_tracker.cpp ON)

# host_side
dai_add_example(device_queue_event src/host_side/device_queue_event.cpp ON)
dai_add_example(opencv_support src/host_side/opencv_support.cpp ON)
dai_add_example(queue_add_callback src/host_side/queue_add_callback.cpp ON)

# ImageManip
dai_add_example(image_manip src/ImageManip/image_manip_example.cpp ON)
dai_add_example(image_manip_rotate src/ImageManip/image_manip_rotate.cpp ON)
dai_add_example(image_manip_tiling src/ImageManip/image_manip_tiling.cpp ON)
dai_add_example(rgb_rotate_warp src/ImageManip/rgb_rotate_warp.cpp ON)

# IMU
dai_add_example(imu_gyroscope_accelerometer src/IMU/imu_gyroscope_accelerometer.cpp ON)
dai_add_example(imu_rotation_vector src/IMU/imu_rotation_vector.cpp ON)

# mixed
dai_add_example(mono_depth_mobilenetssd src/mixed/mono_depth_mobilenetssd.cpp ON)
dai_add_example(rgb_encoding_mono_mobilenet src/mixed/rgb_encoding_mono_mobilenet.cpp ON)
dai_add_example(rgb_encoding_mono_mobilenet_depth src/mixed/rgb_encoding_mono_mobilenet_depth.cpp ON)
dai_add_example(rgb_encoding_mobilenet src/mixed/rgb_encoding_mobilenet.cpp ON)

target_compile_definitions(mono_depth_mobilenetssd PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(rgb_encoding_mono_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(rgb_encoding_mono_mobilenet_depth PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(rgb_encoding_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

# MobileNet
dai_add_example(camera_mobilenet_sync src/MobileNet/camera_mobilenet_sync_example.cpp ON)
dai_add_example(rgb_mobilenet src/MobileNet/rgb_mobilenet.cpp  ON)
dai_add_example(rgb_mobilenet_4k src/MobileNet/rgb_mobilenet_4k.cpp ON)
dai_add_example(mono_mobilenet src/MobileNet/mono_mobilenet.cpp  ON)
dai_add_example(video_mobilenet src/MobileNet/video_mobilenet.cpp ON)
dai_add_example(webcam_mobilenet src/MobileNet/webcam_mobilenet_example.cpp ON)

target_compile_definitions(camera_mobilenet_sync PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(rgb_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}"  ON)
target_compile_definitions(rgb_mobilenet_4k PRIVATE BLOB_PATH="${mobilenet_5shaves_blob}")
target_compile_definitions(mono_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(video_mobilenet PRIVATE BLOB_PATH="${mobilenet_8shaves_blob}" VIDEO_PATH="${construction_vest}")
target_compile_definitions(webcam_mobilenet PRIVATE BLOB_PATH="${mobilenet_8shaves_blob}")

# MonoCamera
dai_add_example(mono_camera_control src/MonoCamera/mono_camera_control.cpp ON)
dai_add_example(mono_preview src/MonoCamera/mono_preview.cpp  ON)
dai_add_example(mono_full_resolution_saver src/MonoCamera/mono_full_resolution_saver.cpp  ON)

# NeuralNetwork

# ObjectTracker
dai_add_example(object_tracker src/ObjectTracker/object_tracker.cpp ON)
dai_add_example(spatial_object_tracker src/ObjectTracker/spatial_object_tracker.cpp ON)
dai_add_example(object_tracker_video src/ObjectTracker/object_tracker_video.cpp ON)

target_compile_definitions(object_tracker PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(spatial_object_tracker PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(object_tracker_video PRIVATE BLOB_PATH="${person_detection}" VIDEO_PATH="${construction_vest}")

# Script
dai_add_example(script_camera_control src/Script/script_camera_control.cpp ON)
dai_add_example(script_get_ip src/Script/script_get_ip.cpp OFF)
dai_add_example(script_http_client src/Script/script_http_client.cpp OFF)
dai_add_example(script_http_server src/Script/script_http_server.cpp OFF)
dai_add_example(script_http_stream src/Script/script_http_stream.cpp OFF)
dai_add_example(script_nndata_datatype src/Script/script_nndata_datatype.cpp ON)

# SpatialDetection
dai_add_example(spatial_location_calculator src/SpatialDetection/spatial_location_calculator.cpp ON)
dai_add_example(spatial_mobilenet_mono src/SpatialDetection/spatial_mobilenet_mono.cpp ON)
dai_add_example(spatial_mobilenet src/SpatialDetection/spatial_mobilenet.cpp ON)
dai_add_example(spatial_tiny_yolo_v3 src/SpatialDetection/spatial_tiny_yolo.cpp ON)
dai_add_example(spatial_tiny_yolo_v4 src/SpatialDetection/spatial_tiny_yolo.cpp ON)

target_compile_definitions(spatial_mobilenet_mono PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(spatial_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(spatial_tiny_yolo_v3 PRIVATE BLOB_PATH="${tiny_yolo_v3_blob}")
target_compile_definitions(spatial_tiny_yolo_v4 PRIVATE BLOB_PATH="${tiny_yolo_v4_blob}")

# StereoDepth
dai_add_example(depth_crop_control src/StereoDepth/depth_crop_control.cpp ON)
dai_add_example(depth_preview src/StereoDepth/depth_preview.cpp  ON)
dai_add_example(rgb_depth_aligned src/StereoDepth/rgb_depth_aligned.cpp ON)
dai_add_example(stereo_depth_video src/StereoDepth/stereo_depth_video.cpp ON)

# SystemLogger
dai_add_example(system_information src/SystemLogger/system_information.cpp ON)

# VideoEncoder
dai_add_example(rgb_encoding src/VideoEncoder/rgb_encoding.cpp  ON)
dai_add_example(rgb_full_resolution_saver src/VideoEncoder/rgb_full_resolution_saver.cpp  ON)
dai_add_example(rgb_mono_encoding src/VideoEncoder/rgb_mono_encoding.cpp  ON)
dai_add_example(encoding_max_limit src/VideoEncoder/encoding_max_limit.cpp ON)
dai_add_example(mjpeg_encoding src/VideoEncoder/mjpeg_encoding_example.cpp ON)

# Yolo
dai_add_example(tiny_yolo_v4_device_side_decoding src/Yolo/tiny_yolo.cpp ON)
dai_add_example(tiny_yolo_v3_device_side_decoding src/Yolo/tiny_yolo.cpp  ON)
target_compile_definitions(tiny_yolo_v4_device_side_decoding PRIVATE BLOB_PATH="${tiny_yolo_v4_blob}")
target_compile_definitions(tiny_yolo_v3_device_side_decoding PRIVATE BLOB_PATH="${tiny_yolo_v3_blob}")
