project(tof_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(tof_minimal tof_minimal.cpp OFF OFF)
dai_add_example(tof_align tof_align.cpp OFF OFF)
