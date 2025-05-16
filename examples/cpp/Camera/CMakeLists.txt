project(camera_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(camera camera_output.cpp ON OFF)
dai_set_example_test_labels(camera ondevice rvc2_all rvc4 ci)

dai_add_example(camera_multiple_outputs camera_multiple_outputs.cpp OFF OFF)