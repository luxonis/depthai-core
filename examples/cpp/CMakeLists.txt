# Add an option to test examples
option(DEPTHAI_TEST_EXAMPLES "Test examples - examples will be ran as a part of test suite" OFF)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Sanitizers)

# Create utility library
add_library(utility utility/utility.cpp)
target_include_directories(utility PUBLIC "utility")
add_default_flags(utility LEAN)
target_link_libraries(utility ${OpenCV_LIBS})

set(test_env
    # Misc
    "UBSAN_OPTIONS=halt_on_error=1"
    # DepthAI
    "DEPTHAI_SEARCH_TIMEOUT=15000"
    "DEPTHAI_RECONNECT_TIMEOUT=0"
    )

# Helper for adding new examples
function(dai_add_example example_name example_src enable_test use_pcl)

    # Add example
    add_executable(${example_name} ${example_src})
    add_default_flags(${example_name} LEAN)
    set(DEPTHAI_TARGET depthai::core)
    if(use_pcl AND NOT DEPTHAI_MERGED_TARGET)
        set(DEPTHAI_TARGET depthai::all)
    elseif(NOT DEPTHAI_MERGED_TARGET)
        set(DEPTHAI_TARGET depthai::opencv)
    endif()
    target_link_libraries(${example_name} PRIVATE utility ${DEPTHAI_TARGET} ${OpenCV_LIBS} Threads::Threads)
    # Set compiler features (c++17), and disables extensions (g++17)
    set_property(TARGET ${example_name} PROPERTY CXX_STANDARD 17)
    set_property(TARGET ${example_name} PROPERTY CXX_STANDARD_REQUIRED ON)
    set_property(TARGET ${example_name} PROPERTY CXX_EXTENSIONS OFF)

    # Add sanitizers for example
    if(COMMAND add_sanitizers)
        add_sanitizers(${example_name})
    endif()
    # Add to clangformat target
    if(COMMAND target_clangformat_setup)
        target_clangformat_setup(${example_name} "")
    endif()

    # parse the rest of the arguments
    set(arguments ${ARGV})
    list(REMOVE_AT arguments 0 1 2 3)

    # If 'DEPTHAI_TEST_EXAMPLES' is ON, then examples will be part of the test suite
    if(${enable_test} AND DEPTHAI_TEST_EXAMPLES)
        add_test(NAME ${example_name} COMMAND ${CMAKE_COMMAND}
            -DTIMEOUT_SECONDS=10
            -P ${CMAKE_CURRENT_LIST_DIR}/cmake/ExecuteTestTimeout.cmake $<TARGET_FILE:${example_name}> ${arguments}
        )

        # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
        set_tests_properties(${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION "\\[warning\\];\\[error\\];\\[critical\\]")

        # Add ubsan halt on error
        set_tests_properties(${example_name} PROPERTIES ENVIRONMENT "${test_env}")

    endif()

    # Copy over required DLLs (Windows)
    if(WIN32)
        # Copy dlls to target directory - Windows only
        # TARGET_RUNTIME_DLLS generator expression available since CMake 3.21
        if(CMAKE_VERSION VERSION_LESS "3.21")
            file(GLOB depthai_dll_libraries "${HUNTER_INSTALL_PREFIX}/bin/*.dll")
        else()
            set(depthai_dll_libraries "$<TARGET_RUNTIME_DLLS:${example_name}>")
        endif()
        add_custom_command(TARGET ${example_name} POST_BUILD COMMAND
            "$<$<BOOL:${depthai_dll_libraries}>:${CMAKE_COMMAND};-E;copy_if_different;${depthai_dll_libraries};$<TARGET_FILE_DIR:${example_name}>>"
            COMMAND_EXPAND_LISTS
            VERBATIM
        )
    endif()
endfunction()


# Function for setting test labels, including label shortcuts
function(dai_set_example_test_labels example_name)
    # If the example is not enabled for testing, return
    if(NOT DEPTHAI_TEST_EXAMPLES)
        return()
    endif()
    set(expanded_labels "")
    foreach(label IN LISTS ARGN)
        if(label STREQUAL "rvc2_all")
            list(APPEND expanded_labels "rvc2" "poe" "usb")
        else()
            list(APPEND expanded_labels ${label})
        endif()
    endforeach()
    # Add "cpp_example" label
    list(APPEND expanded_labels "cpp_example")
    set_tests_properties(${example_name} PROPERTIES LABELS "${expanded_labels}")
endfunction()


# Add any custom binary data using the following snippet
## # Hunter test data download
## private_data(
##     URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet.blob"
##     SHA1 "e89d3ee9f26d80397e44f89c6b375990064a4a42"
##     FILE "mobilenet.blob"
##     LOCATION mobilenet_blob
## )


# Hunter test data download

# NeuralNetwork node, mobilenet example
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_6shave.blob"
    SHA1 "4f4506726e3083981064938a0faaf9af6180d2c6"
    FILE "mobilenet-ssd_openvino_2021.4_6shave.blob"
    LOCATION mobilenet_blob
)

# YoloV3 resource
private_data(
    URL "http://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
    SHA1 "dedb2d4d96b23e42d15c15e454b8f02eca2713de"
    FILE "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
    LOCATION tiny_yolo_v3_blob
)

# YoloV4 resource
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob"
    SHA1 "d8d09b697dac298fe83cf8856740a21b1a61ab89"
    FILE "yolo-v4-tiny-tf_openvino_2021.4_6shave.blob"
    LOCATION tiny_yolo_v4_blob
)

# NeuralNetwork node, mobilenet example, 5 shaves
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_5shave.blob"
    SHA1 "14ab3ef446b8a649d6694d94c2fc8b798efd74a3"
    FILE "mobilenet-ssd_openvino_2021.4_5shave.blob"
    LOCATION mobilenet_5shaves_blob
)

# NeuralNetwork node, mobilenet example, 8 shaves
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_8shave.blob"
    SHA1 "7e7ea9a680668f3ca9e8f1d22cc8f3f1a02ac018"
    FILE "mobilenet-ssd_openvino_2021.4_8shave.blob"
    LOCATION mobilenet_8shaves_blob
)

# Video file with objects to detect
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/construction_vest.mp4"
    SHA1 "271d8d0b702e683ce02957db7c100843de5ceaec"
    FILE "construction_vest.mp4"
    LOCATION construction_vest
)

# Calibration Sample files
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/depthai_calib.json"
    SHA1 "ec50a47496473116c51cec1c8249bff2679383c3"
    FILE "depthai_calib.json"
    LOCATION calib_v6
)

private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/depthai_v5.calib"
    SHA1 "adfe9976b9aacfdaa76ac7138810999988177e71"
    FILE "depthai_v5.calib"
    LOCATION calib_v5
)

private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/BW1098OBC.json"
    SHA1 "1e84b6c339ac51e3b254775090b91235a40fac71"
    FILE "BW1098OBC.json"
    LOCATION device_config
)

# person-detection
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/person-detection-retail-0013_openvino_2021.4_7shave.blob"
    SHA1 "946d102874b287292cdb5952fa66356c8d36025f"
    FILE "person-detection-retail-0013_openvino_2021.4_7shave.blob"
    LOCATION person_detection
)

# concat model
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/concat_openvino_2021.4_6shave.blob"
    SHA1 "e4af6fb2bed39648a2a4424d8bdc5bac56d616df"
    FILE "concat_openvino_2021.4_6shave.blob"
    LOCATION concat_model
)

# normalization model
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/normalize_openvino_2021.4_4shave.blob"
    SHA1 "1435f6d11e0dfdbc60f2dfa45aecd3d530b6153b"
    FILE "normalize_openvino_2021.4_4shave.blob"
    LOCATION normalization_model
)

# mobilnet xml model for S3 example
private_data(
    URL "https://download.01.org/opencv/2021/openvinotoolkit/2021.1/open_model_zoo/models_bin/1/person-detection-0202/FP32/person-detection-0202.xml"
    SHA1 "87f9cf8833043162f1b728e9afa6f38ba177d9db"
    FILE "person-detection-0202.xml"
    LOCATION person_detection_xml
)

# mobilnet xml model for S3 example
private_data(
    URL "https://download.01.org/opencv/2021/openvinotoolkit/2021.1/open_model_zoo/models_bin/1/person-detection-0202/FP32/person-detection-0202.bin"
    SHA1 "1560bf2ee372a0640539780ea8bd061278a6cc20"
    FILE "person-detection-0202.bin"
    LOCATION person_detection_bin
)

# yolo xml model for S3 example
private_data(
    URL "https://github.com/luxonis/depthai-model-zoo/raw/main/models/yolov4_tiny_coco_416x416/yolov4_tiny_coco_416x416.xml"
    SHA1 "423069bdff250643cf080c76e15594cfd72cb816"
    FILE "yolov4_tiny_coco_416x416.xml"
    LOCATION tiny-vechicle-detection_xml
)

# yolo bin model for S3 example
private_data(
    URL "https://robothub.fra1.cdn.digitaloceanspaces.com/models/yolov4_tiny_coco_416x416/yolov4_tiny_coco_416x416.bin"
    SHA1 "a7627c074ee76ceb9ff9bb8529b67867e3e440bc"
    FILE "yolov4_tiny_coco_416x416.bin"
    LOCATION tiny-vechicle-detection_bin
)

# yolo v5 nn archive
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/nnarchive/yolo-v6-openvino_2022.1_6shave-rvc2.tar.xz"
    SHA1 "63a5bb29d1ca0d3b019f13d66401e634539aac82"
    FILE "yolo-v6-openvino_2022.1_6shave-rvc2.tar.xz"
    LOCATION nn_archive_yolo_v6_tar_xz
)

# Camera
dai_add_example(camera Camera/camera_output.cpp ON OFF)
dai_set_example_test_labels(camera ondevice rvc2_all rvc4 ci)

dai_add_example(camera_multiple_outputs Camera/camera_multiple_outputs.cpp OFF OFF)

# ImageManip
dai_add_example(image_manip_host HostNodes/image_manip_host.cpp OFF OFF)
dai_add_example(image_manip_color_conversion_host HostNodes/image_manip_color_conversion.cpp OFF OFF)
dai_add_example(image_manip_v2_resize ImageManip/image_manip_v2_resize.cpp OFF OFF)
dai_add_example(image_manip_v2_mod ImageManip/image_manip_v2_mod.cpp OFF OFF)
dai_add_example(image_manip_color_conversion ImageManip/image_manip_color_conversion.cpp OFF OFF)

# Record
dai_add_example(record_encoded_h264 RVC2/Record/record_encoded_h264.cpp OFF OFF)
dai_add_example(record_encoded_mjpeg RVC2/Record/record_encoded_mjpeg.cpp OFF OFF)
dai_add_example(record_raw_color RVC2/Record/record_raw_color.cpp OFF OFF)
dai_add_example(record_raw_gray RVC2/Record/record_raw_gray.cpp OFF OFF)
dai_add_example(record_imu RVC2/Record/record_imu.cpp OFF OFF)

# Replay
dai_add_example(replay_video_meta RVC2/Replay/replay_video_meta.cpp OFF OFF)
dai_add_example(replay_imu RVC2/Replay/replay_imu.cpp OFF OFF)

# Thermal
dai_add_example(thermal RVC2/Thermal/thermal.cpp OFF OFF)

# Host nodes
dai_add_example(rgb_video RVC2/ColorCamera/rgb_video.cpp OFF OFF)
dai_add_example(host_node HostNodes/host_node.cpp OFF OFF)
dai_add_example(threaded_host_node HostNodes/threaded_host_node.cpp OFF OFF)

# Model zoo
dai_add_example(model_zoo RVC2/ModelZoo/model_zoo.cpp OFF OFF)

# Image Align
dai_add_example(image_align RVC2/ImageAlign/image_align.cpp OFF OFF)

# Visualizer
dai_add_example(visualizer_yolo Visualizer/visualizer_yolo.cpp ON OFF)
dai_set_example_test_labels(visualizer_yolo ondevice rvc2_all rvc4)

# VideoEncoder
dai_add_example(video_encode VideoEncoder/video_encode.cpp ON OFF)
dai_set_example_test_labels(video_encode ondevice rvc2_all rvc4)

if(DEPTHAI_RTABMAP_SUPPORT)
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL  https://github.com/rerun-io/rerun/releases/download/0.16.1/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)
    dai_add_example(rtabmap_vio RVC2/VSLAM/rtabmap_vio.cpp OFF OFF)
    target_link_libraries(rtabmap_vio PRIVATE depthai::rtabmap rerun_sdk)

    dai_add_example(rtabmap_vio_slam RVC2/VSLAM/rtabmap_vio_slam.cpp OFF ON)
    target_link_libraries(rtabmap_vio_slam PRIVATE depthai::rtabmap rerun_sdk)

    if(DEPTHAI_BASALT_SUPPORT)
        dai_add_example(basalt_vio_rtabmap_slam RVC2/VSLAM/basalt_vio_rtabmap_slam.cpp OFF ON)
        target_link_libraries(basalt_vio_rtabmap_slam PRIVATE depthai::basalt depthai::rtabmap rerun_sdk)
    endif()

endif()

if(DEPTHAI_BASALT_SUPPORT)
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL  https://github.com/rerun-io/rerun/releases/download/0.16.1/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)

    dai_add_example(basalt_vio RVC2/VSLAM/basalt_vio.cpp OFF OFF)
    target_link_libraries(basalt_vio PRIVATE depthai::basalt rerun_sdk)
endif()


# if(DEPTHAI_HAVE_PCL_SUPPORT)
#     dai_add_example(visualize_pointcloud PointCloud/visualize_pointcloud.cpp ON ON)
# endif()
