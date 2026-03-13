project(pointcloud_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(PointCloudShowcase PointCloudShowcase.cpp OFF OFF)

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    dai_add_example(PointCloudVisualizer PointCloudVisualizer.cpp OFF OFF)
endif()