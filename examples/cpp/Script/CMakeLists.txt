project(script_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(script_simple script_simple.cpp OFF OFF)

dai_add_example(script_switch_all_cameras script_switch_all_cameras.cpp OFF OFF)