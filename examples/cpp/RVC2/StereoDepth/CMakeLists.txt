project(stereo_autocreate_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(stereo_autocreate stereo_autocreate.cpp OFF OFF)

dai_add_example(rvc2_stereo rvc2_stereo.cpp OFF OFF)