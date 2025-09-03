project(camera_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(camera_all camera_all.cpp ON OFF)
dai_set_example_test_labels(camera_all ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(camera_max_resolution camera_max_resolution.cpp ON OFF)
dai_set_example_test_labels(camera_max_resolution ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(camera_multiple_outputs camera_multiple_outputs.cpp ON OFF 640 480 0 30 CAM_A 300 300 0 30 CAM_A 300 300 1 30 CAM_A)
dai_set_example_test_labels(camera_multiple_outputs ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(camera_output camera_output.cpp ON OFF)
dai_set_example_test_labels(camera_output ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(camera_raw camera_raw.cpp ON OFF)
dai_set_example_test_labels(camera_raw ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(camera_roi_exposure_focus camera_roi_exposure_focus.cpp ON OFF)
dai_set_example_test_labels(camera_roi_exposure_focus ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(camera_still_max_res camera_still_max_res.cpp ON OFF)
dai_set_example_test_labels(camera_still_max_res ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(camera_undistortion camera_undistortion.cpp ON OFF)
dai_set_example_test_labels(camera_undistortion ondevice rvc2_all rvc4 ci)
