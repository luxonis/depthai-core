project(depth_align_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(depth_align depth_align.cpp ON OFF)
dai_set_example_test_labels(depth_align ondevice rvc2_all rvc4 ci)