project(dcl_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(calibration_quality_dynamic calibration_quality_dynamic.cpp ON OFF)
dai_add_example(calibration_dynamic calibration_dynamic.cpp ON OFF)
dai_add_example(calibration_integration calibration_integration.cpp ON OFF)
