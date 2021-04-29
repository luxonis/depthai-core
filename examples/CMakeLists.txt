# Add an option to test examples
option(DEPTHAI_TEST_EXAMPLES "Test examples - examples will be ran as a part of test suite" OFF)

# Dependencies
find_package(OpenCV REQUIRED)

# Create utility library
add_library(utility src/utility.cpp)
target_link_libraries(utility FP16::fp16 ${OpenCV_LIBS})

# Macro for adding new examples
macro(dai_add_example example_name example_src)
    add_executable("${example_name}" "${example_src}")
    target_link_libraries("${example_name}" PRIVATE utility depthai::opencv ${OpenCV_LIBS})

    # Add to clangformat target
    target_clangformat_setup(${example_name} "")

    # parse the rest of the arguments
    set(arguments ${ARGV})
    list(REMOVE_AT arguments 0 1)

    # If 'DEPTHAI_TEST_EXAMPLES' is ON, then examples will be part of the test suite
    if(DEPTHAI_TEST_EXAMPLES)
        add_test(NAME ${example_name} COMMAND ${CMAKE_COMMAND}
            -DTIMEOUT_SECONDS=5
            -P ${CMAKE_CURRENT_LIST_DIR}/cmake/ExecuteTestTimeout.cmake $<TARGET_FILE:${example_name}> ${arguments}
        )

        # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
        set_tests_properties (${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION "\\[warning\\];\\[error\\];\\[critical\\]")

        # Add sanitizers for tests as well
        add_sanitizers(${example_name})

        # Add ubsan halt on error
        set_tests_properties(${example_name} PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1")

    endif()
endmacro()

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
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_6shave.blob"
    SHA1 "f0e14978b3f77a4f93b9f969cd39e58bb7aef490"
    FILE "mobilenet-ssd_openvino_2021.2_6shave.blob"
    LOCATION mobilenet_blob
)

# YoloV3 resource
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/tiny-yolo-v3_openvino_2021.2_6shave.blob"
    SHA1 "f0ac263a0d55c374e1892eea21c9b7d1170bde46"
    FILE "tiny-yolo-v3_openvino_2021.2_6shave.blob"
    LOCATION tiny_yolo_v3_blob
)

# YoloV4 resource
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/tiny-yolo-v4_openvino_2021.2_6shave.blob"
    SHA1 "219d949610a5760e62a8458941e1300b81c3fe4a"
    FILE "tiny-yolo-v4_openvino_2021.2_6shave.blob"
    LOCATION tiny_yolo_v4_blob
)

# NeuralNetwork node, mobilenet example, 5 shaves
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_5shave.blob"
    SHA1 "d715f85e474609cf3f696d7a2e3750804ed6c726"
    FILE "mobilenet-ssd_openvino_2021.2_5shave.blob"
    LOCATION mobilenet_5shaves_blob
)

# NeuralNetwork node, mobilenet example, 8 shaves
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_8shave.blob"
    SHA1 "3329bb8f3a9c881ef9756d232055f9d6f38aa07b"
    FILE "mobilenet-ssd_openvino_2021.2_8shave.blob"
    LOCATION mobilenet_8shaves_blob
)

# Video file with objects to detect
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/construction_vest.mp4"
    SHA1 "271d8d0b702e683ce02957db7c100843de5ceaec"
    FILE "construction_vest.mp4"
    LOCATION construction_vest
)

## message(STATUS "Location of test1.data: ${test1_data}")

# Color camera preview output example
dai_add_example(01_rgb_preview src/01_rgb_preview.cpp)

# Mono camera preview output example
dai_add_example(02_mono_preview src/02_mono_preview.cpp)

# Depth preview output example
dai_add_example(03_depth_preview src/03_depth_preview.cpp)

# rgb encoding
dai_add_example(04_rgb_encoding src/04_rgb_encoding.cpp)

# rgb+mono encoding
dai_add_example(05_rgb_mono_encoding src/05_rgb_mono_encoding.cpp)

dai_add_example(06_rgb_full_resolution_saver src/06_rgb_full_resolution_saver.cpp)

dai_add_example(07_mono_full_resolution_saver src/07_mono_full_resolution_saver.cpp)

dai_add_example(09_mono_mobilenet src/09_mono_mobilenet.cpp)
target_compile_definitions(09_mono_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(10_mono_depth_mobilenetssd src/10_mono_depth_mobilenetssd.cpp)
target_compile_definitions(10_mono_depth_mobilenetssd PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(11_rgb_encoding_mono_mobilenet src/11_rgb_encoding_mono_mobilenet.cpp)
target_compile_definitions(11_rgb_encoding_mono_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(12_rgb_encoding_mono_mobilenet_depth src/12_rgb_encoding_mono_mobilenet_depth.cpp)
target_compile_definitions(12_rgb_encoding_mono_mobilenet_depth PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(13_encoding_max_limit src/13_encoding_max_limit.cpp)

dai_add_example(14_2_mono_camera_control src/14_2_mono_camera_control.cpp)

dai_add_example(14_3_depth_crop_control src/14_3_depth_crop_control.cpp)

dai_add_example(15_rgb_mobilenet_4k src/15_rgb_mobilenet_4k.cpp)
target_compile_definitions(15_rgb_mobilenet_4k PRIVATE BLOB_PATH="${mobilenet_5shaves_blob}")

dai_add_example(18_rgb_encoding_mobilenet src/18_rgb_encoding_mobilenet.cpp)
target_compile_definitions(18_rgb_encoding_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")


# Color camera video output example
dai_add_example(28_1_rgb_camera_video src/28_1_rgb_camera_video.cpp)

# Mono camera video output example
dai_add_example(28_2_mono_camera_video src/28_2_mono_camera_video.cpp)

dai_add_example(08_rgb_mobilenet src/08_rgb_mobilenet.cpp)
target_compile_definitions(08_rgb_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

# NeuralNetwork node, webcam input (from host)
dai_add_example(webcam_mobilenet src/webcam_mobilenet_example.cpp)
target_compile_definitions(webcam_mobilenet PRIVATE BLOB_PATH="${mobilenet_8shaves_blob}")

# MJPEG encoding
dai_add_example(mjpeg_encoding src/mjpeg_encoding_example.cpp)

# Stereo Depth example
dai_add_example(30_2_stereo_depth_video src/30_2_stereo_depth_video.cpp)

# Image Manip node examples
dai_add_example(image_manip src/image_manip_example.cpp)
dai_add_example(20_image_manip_warp src/20_image_manip_warp.cpp)

# Color Camera config example
dai_add_example(14_1_color_camera_control src/14_1_color_camera_control.cpp)

# System information example
dai_add_example(25_system_information src/25_system_information.cpp)

# Device getQueueEvent example
dai_add_example(16_device_queue_event src/16_device_queue_event.cpp)

# OpenCV support example
dai_add_example(24_opencv_support src/24_opencv_support.cpp)

# RGB-depth alignment example
dai_add_example(rgb_depth_aligned src/rgb_depth_aligned_example.cpp)

# Device side decoding example for mobilenet-ssd with 3d coordinates on RGB camera
dai_add_example(26_1_spatial_mobilenet src/26_1_spatial_mobilenet.cpp)
target_compile_definitions(26_1_spatial_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

# Device side decoding example for mobilenet-ssd with 3d coordinates on right camera
dai_add_example(26_2_spatial_mobilenet_mono src/26_2_spatial_mobilenet_mono.cpp)
target_compile_definitions(26_2_spatial_mobilenet_mono PRIVATE BLOB_PATH="${mobilenet_blob}")

# Device side decoding example for tiny-yolo-v3
dai_add_example(22_1_tiny_yolo_v3_device_side_decoding src/22_1_tiny_yolo_v3_device_side_decoding.cpp)
target_compile_definitions(22_1_tiny_yolo_v3_device_side_decoding PRIVATE BLOB_PATH="${tiny_yolo_v3_blob}")

# Device side decoding example for tiny-yolo-v4
dai_add_example(22_2_tiny_yolo_v4_device_side_decoding src/22_2_tiny_yolo_v4_device_side_decoding.cpp)
target_compile_definitions(22_2_tiny_yolo_v4_device_side_decoding PRIVATE BLOB_PATH="${tiny_yolo_v4_blob}")

# Sync example between NN and camera frames
dai_add_example(camera_mobilenet_sync src/camera_mobilenet_sync_example.cpp)
target_compile_definitions(camera_mobilenet_sync PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(27_spatial_location_calculator src/27_spatial_location_calculator.cpp)

# Device side decoding example for tiny-yolo-v3 with 3d coordinates on RGB camera
dai_add_example(26_3_spatial_tiny_yolo_v3 src/26_3_spatial_tiny_yolo.cpp)
target_compile_definitions(26_3_spatial_tiny_yolo_v3 PRIVATE BLOB_PATH="${tiny_yolo_v3_blob}")

# Device side decoding example for tiny-yolo-v4 with 3d coordinates on RGB camera
dai_add_example(26_4_spatial_tiny_yolo_v4 src/26_3_spatial_tiny_yolo.cpp)
target_compile_definitions(26_4_spatial_tiny_yolo_v4 PRIVATE BLOB_PATH="${tiny_yolo_v4_blob}")

# Object tracker example on mobilenet SSD output
dai_add_example(29_1_object_tracker src/29_1_object_tracker.cpp)
target_compile_definitions(29_1_object_tracker PRIVATE BLOB_PATH="${mobilenet_blob}")

# Spatial Object tracker example on mobilenet SSD output
dai_add_example(29_2_spatial_object_tracker src/29_2_spatial_object_tracker.cpp)
target_compile_definitions(29_2_spatial_object_tracker PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(17_video_mobilenet src/17_video_mobilenet.cpp)
target_compile_definitions(17_video_mobilenet PRIVATE BLOB_PATH="${mobilenet_8shaves_blob}" VIDEO_PATH="${construction_vest}")

