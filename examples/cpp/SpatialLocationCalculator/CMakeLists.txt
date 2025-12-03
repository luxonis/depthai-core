project(spatial_location_calculator_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(spatial_location_calculator spatial_location_calculator.cpp ON OFF)
dai_set_example_test_labels(spatial_location_calculator ondevice rvc2_all rvc4 ci)

dai_add_example(spatial_segmentation spatial_segmentation.cpp ON OFF)
dai_set_example_test_labels(spatial_segmentation ondevice rvc2_all rvc4 ci)
