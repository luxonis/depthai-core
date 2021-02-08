# Dependencies
find_package(OpenCV REQUIRED)

# Create utility library
add_library(utility src/utility.cpp)
target_link_libraries(utility FP16::fp16 ${OpenCV_LIBS})

# Macro for adding new examples
macro(dai_add_example example_name example_src)
    add_executable("${example_name}" "${example_src}")
    target_link_libraries("${example_name}" PRIVATE utility depthai-core ${OpenCV_LIBS})

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

## message(STATUS "Location of test1.data: ${test1_data}")

# Color camera preview output example
dai_add_example(camera_preview src/camera_preview_example.cpp)

# Color camera video output example
dai_add_example(camera_video src/camera_video_example.cpp)

# Mono camera video output example
dai_add_example(mono_camera src/mono_camera_example.cpp)

# NeuralNetwork node, mobilenet example
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet.blob"
    SHA1 "e89d3ee9f26d80397e44f89c6b375990064a4a42"
    FILE "mobilenet.blob"
    LOCATION mobilenet_blob
)
dai_add_example(camera_mobilenet src/camera_mobilenet_example.cpp)
target_compile_definitions(camera_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

# NeuralNetwork node, webcam input (from host)
dai_add_example(webcam_mobilenet src/webcam_mobilenet_example.cpp)
target_compile_definitions(webcam_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

# MJPEG encoding
dai_add_example(mjpeg_encoding src/mjpeg_encoding_example.cpp)

# StereoDepth example
dai_add_example(stereo src/stereo_example.cpp)

# Image Manip node example
dai_add_example(image_manip src/image_manip_example.cpp)

# Color Camera config example
dai_add_example(color_camera_control src/color_camera_control_example.cpp)

# System information example
dai_add_example(system_information src/system_information_example.cpp)

# Device getQueueEvent example
dai_add_example(device_queue_event src/device_queue_event_example.cpp)

# Device side decoding example for mobilenet-ssd
dai_add_example(mobilenet_device_side_decoding src/mobilenet_device_side_decoding_example.cpp)
target_compile_definitions(mobilenet_device_side_decoding PRIVATE BLOB_PATH="${mobilenet_blob}")
