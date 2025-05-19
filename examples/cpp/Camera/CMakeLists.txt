project(camera_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(camera_all camera_all.cpp OFF OFF)

dai_add_example(camera_max_resolution camera_max_resolution.cpp OFF OFF)

dai_add_example(camera_multiple_outputs camera_multiple_outputs.cpp OFF OFF)

dai_add_example(camera_output camera_output.cpp OFF OFF)

dai_add_example(camera_raw camera_raw.cpp OFF OFF)

dai_add_example(camera_roi_exposure_focus camera_roi_exposure_focus.cpp OFF OFF)

dai_add_example(camera_still_max_res camera_still_max_res.cpp OFF OFF)