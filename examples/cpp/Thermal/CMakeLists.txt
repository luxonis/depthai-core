project(thermal_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(thermal thermal.cpp OFF OFF)
dai_add_example(thermal_crosshair thermal_crosshair.cpp OFF OFF)
dai_add_example(thermal_align thermal_align.cpp OFF OFF)
dai_add_example(thermal_detection_network_remap thermal_detection_network_remap.cpp OFF OFF)