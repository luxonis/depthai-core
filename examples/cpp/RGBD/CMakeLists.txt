project(rgbd_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

# Skip compiling rerun examples on Windows, since the used version of rerun doesn't compile on Windows
if(NOT WIN32 AND DEPTHAI_PCL_SUPPORT)
    find_package(rerun_sdk CONFIG REQUIRED)
    dai_add_example(rgbd rgbd.cpp ON OFF)
    target_link_libraries(rgbd PRIVATE rerun_sdk)
endif()

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    dai_add_example(visualizer_rgbd visualizer_rgbd.cpp ON OFF)
endif()

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    dai_add_example(visualizer_rgbd_autocreate visualizer_rgbd_autocreate.cpp ON OFF)
endif()

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    dai_add_example(rgbd_pcl_processing rgbd_pcl_processing.cpp ON OFF)
endif()