project(vslam_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

if(DEPTHAI_RTABMAP_SUPPORT)
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL  https://github.com/rerun-io/rerun/releases/download/0.16.1/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)
    dai_add_example(rtabmap_vio rtabmap_vio.cpp OFF OFF)
    target_link_libraries(rtabmap_vio PRIVATE depthai::rtabmap rerun_sdk)

    dai_add_example(rtabmap_vio_slam rtabmap_vio_slam.cpp OFF ON)
    target_link_libraries(rtabmap_vio_slam PRIVATE depthai::rtabmap rerun_sdk)

    if(DEPTHAI_BASALT_SUPPORT)
        dai_add_example(basalt_vio_rtabmap_slam basalt_vio_rtabmap_slam.cpp OFF ON)
        target_link_libraries(basalt_vio_rtabmap_slam PRIVATE depthai::basalt depthai::rtabmap rerun_sdk)
    endif()

endif()

if(DEPTHAI_BASALT_SUPPORT)
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL  https://github.com/rerun-io/rerun/releases/download/0.16.1/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)

    dai_add_example(basalt_vio basalt_vio.cpp OFF OFF)
    target_link_libraries(basalt_vio PRIVATE depthai::basalt rerun_sdk)
endif()