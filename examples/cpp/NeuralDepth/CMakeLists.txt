project(stereo_depth_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(neural_depth neural_depth.cpp ON OFF)
dai_set_example_test_labels(neural_depth ondevice rvc4 ci)