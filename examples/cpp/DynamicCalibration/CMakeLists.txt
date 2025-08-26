project(dcl_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(recalibration_one_image recalibration_one_image.cpp OFF OFF)
target_link_libraries(recalibration_one_image PRIVATE dynamic_calibration_imported)
target_compile_definitions(recalibration_one_image PRIVATE DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT)

dai_add_example(recalibrate recalibrate.cpp OFF OFF)
target_link_libraries(recalibrate PRIVATE dynamic_calibration_imported)
target_compile_definitions(recalibrate PRIVATE DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT)
