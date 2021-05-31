# Add an option to test examples
option(DEPTHAI_TEST_EXAMPLES "Test examples - examples will be ran as a part of test suite" OFF)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Sanitizers)

# Create utility library
add_library(utility src/utility.cpp)
target_link_libraries(utility FP16::fp16 ${OpenCV_LIBS})

# Macro for adding new examples
macro(dai_add_example example_name example_src)
    add_executable("${example_name}" "${example_src}")
    target_link_libraries("${example_name}" PRIVATE utility depthai::opencv ${OpenCV_LIBS})

    # Add sanitizers for example
    add_sanitizers(${example_name})

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

# person-detection
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/person-detection-0201_openvino_2021.3_7shave.blob"
    SHA1 "a8d4b0067deb889ec66b2204573d9ee1e0039c8e"
    FILE "person-detection-0201_openvino_2021.3_7shave.blob"
    LOCATION person_detection
)

## message(STATUS "Location of test1.data: ${test1_data}")

# RGB camera preview output example
dai_add_example(rgb_preview src/rgb_preview.cpp)

# Mono camera preview output example
dai_add_example(mono_preview src/mono_preview.cpp)

# Depth preview output example
dai_add_example(depth_preview src/depth_preview.cpp)

# rgb encoding
dai_add_example(rgb_encoding src/rgb_encoding.cpp)

# rgb+mono encoding
dai_add_example(rgb_mono_encoding src/rgb_mono_encoding.cpp)

dai_add_example(rgb_full_resolution_saver src/rgb_full_resolution_saver.cpp)

dai_add_example(mono_full_resolution_saver src/mono_full_resolution_saver.cpp)

dai_add_example(rgb_mobilenet src/rgb_mobilenet.cpp)
target_compile_definitions(rgb_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(mono_mobilenet src/mono_mobilenet.cpp)
target_compile_definitions(mono_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(mono_depth_mobilenetssd src/mono_depth_mobilenetssd.cpp)
target_compile_definitions(mono_depth_mobilenetssd PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(rgb_encoding_mono_mobilenet src/rgb_encoding_mono_mobilenet.cpp)
target_compile_definitions(rgb_encoding_mono_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(rgb_encoding_mono_mobilenet_depth src/rgb_encoding_mono_mobilenet_depth.cpp)
target_compile_definitions(rgb_encoding_mono_mobilenet_depth PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(encoding_max_limit src/encoding_max_limit.cpp)

# RGB Camera config example
dai_add_example(rgb_camera_control src/rgb_camera_control.cpp)

dai_add_example(mono_camera_control src/mono_camera_control.cpp)

dai_add_example(depth_crop_control src/depth_crop_control.cpp)

dai_add_example(rgb_mobilenet_4k src/rgb_mobilenet_4k.cpp)
target_compile_definitions(rgb_mobilenet_4k PRIVATE BLOB_PATH="${mobilenet_5shaves_blob}")

# Device getQueueEvent example
dai_add_example(device_queue_event src/device_queue_event.cpp)

dai_add_example(video_mobilenet src/video_mobilenet.cpp)
target_compile_definitions(video_mobilenet PRIVATE BLOB_PATH="${mobilenet_8shaves_blob}" VIDEO_PATH="${construction_vest}")

dai_add_example(rgb_encoding_mobilenet src/rgb_encoding_mobilenet.cpp)
target_compile_definitions(rgb_encoding_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

# Image Manip node examples
dai_add_example(image_manip src/image_manip_example.cpp)

# Imagae manip node exapmle with warping
dai_add_example(rgb_rotate_warp src/rgb_rotate_warp.cpp)

# Device side decoding example for tiny-yolo-v3
dai_add_example(tiny_yolo_v3_device_side_decoding src/tiny_yolo_v3_device_side_decoding.cpp)
target_compile_definitions(tiny_yolo_v3_device_side_decoding PRIVATE BLOB_PATH="${tiny_yolo_v3_blob}")

# Device side decoding example for tiny-yolo-v4
dai_add_example(tiny_yolo_v4_device_side_decoding src/tiny_yolo_v4_device_side_decoding.cpp)
target_compile_definitions(tiny_yolo_v4_device_side_decoding PRIVATE BLOB_PATH="${tiny_yolo_v4_blob}")

# OpenCV support example
dai_add_example(opencv_support src/opencv_support.cpp)

# System information example
dai_add_example(system_information src/system_information.cpp)

# Device side decoding example for mobilenet-ssd with 3d coordinates on RGB camera
dai_add_example(spatial_mobilenet src/spatial_mobilenet.cpp)
target_compile_definitions(spatial_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

# Device side decoding example for mobilenet-ssd with 3d coordinates on right camera
dai_add_example(spatial_mobilenet_mono src/spatial_mobilenet_mono.cpp)
target_compile_definitions(spatial_mobilenet_mono PRIVATE BLOB_PATH="${mobilenet_blob}")

# Device side decoding example for tiny-yolo-v3 with 3d coordinates on RGB camera
dai_add_example(spatial_tiny_yolo_v3 src/spatial_tiny_yolo.cpp)
target_compile_definitions(spatial_tiny_yolo_v3 PRIVATE BLOB_PATH="${tiny_yolo_v3_blob}")

# Device side decoding example for tiny-yolo-v4 with 3d coordinates on RGB camera
dai_add_example(spatial_tiny_yolo_v4 src/spatial_tiny_yolo.cpp)
target_compile_definitions(spatial_tiny_yolo_v4 PRIVATE BLOB_PATH="${tiny_yolo_v4_blob}")

dai_add_example(spatial_location_calculator src/spatial_location_calculator.cpp)

# RGB camera video output example
dai_add_example(rgb_video src/rgb_video.cpp)

# Object tracker example on mobilenet SSD output
dai_add_example(object_tracker src/object_tracker.cpp)
target_compile_definitions(object_tracker PRIVATE BLOB_PATH="${mobilenet_blob}")

# Spatial Object tracker example on mobilenet SSD output
dai_add_example(spatial_object_tracker src/spatial_object_tracker.cpp)
target_compile_definitions(spatial_object_tracker PRIVATE BLOB_PATH="${mobilenet_blob}")

# Stereo Depth example
dai_add_example(stereo_depth_video src/stereo_depth_video.cpp)

# NeuralNetwork node, webcam input (from host)
dai_add_example(webcam_mobilenet src/webcam_mobilenet_example.cpp)
target_compile_definitions(webcam_mobilenet PRIVATE BLOB_PATH="${mobilenet_8shaves_blob}")

# MJPEG encoding
dai_add_example(mjpeg_encoding src/mjpeg_encoding_example.cpp)

# RGB-depth alignment example
dai_add_example(rgb_depth_aligned src/rgb_depth_aligned.cpp)

# Sync example between NN and camera frames
dai_add_example(camera_mobilenet_sync src/camera_mobilenet_sync_example.cpp)
target_compile_definitions(camera_mobilenet_sync PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_example(object_tracker_video src/object_tracker_video.cpp)
target_compile_definitions(object_tracker_video PRIVATE BLOB_PATH="${person_detection}" VIDEO_PATH="${construction_vest}")
