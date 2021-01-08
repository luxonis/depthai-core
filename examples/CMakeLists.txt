# Dependencies
find_package(OpenCV REQUIRED)

# Create utility library
add_library(utility src/utility.cpp)
target_link_libraries(utility FP16::fp16 ${OpenCV_LIBS})

# Macro for adding new examples
macro(dai_add_example example_name example_src)
    add_executable("${example_name}" "${example_src}")
    target_link_libraries("${example_name}" PRIVATE utility depthai-core ${OpenCV_LIBS})

    # If 'DEPTHAI_TEST_EXAMPLES' is ON, then examples will be part of the test suite
    if(DEPTHAI_TEST_EXAMPLES)
        add_test(NAME ${example_name} COMMAND ${CMAKE_COMMAND} --trace-expand
            -DPATH_TO_TEST_EXECUTABLE=$<TARGET_FILE:${example_name}> 
            -DTIMEOUT_SECONDS=5
            -P ${CMAKE_CURRENT_LIST_DIR}/cmake/ExecuteTestTimeout.cmake
        )
        #add_test(NAME ${example_name} COMMAND timeout -s SIGINT -k 10 5 $<TARGET_FILE:${example_name}>)
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


