project(multi_device_calibration_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

# Registered without test labels: these examples need at least two devices
dai_add_example(multi_device_calibration multi_device_calibration.cpp OFF OFF)
dai_add_example(multi_device_calibration_constraints multi_device_calibration_constraints.cpp OFF OFF)
