project(dcl_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(calib_quality_dynamic calib_quality_dynamic.cpp ON OFF)
dai_add_example(calibrate_dynamic calibrate_dynamic.cpp ON OFF)
dai_add_example(calib_integration calib_integration.cpp ON OFF)
