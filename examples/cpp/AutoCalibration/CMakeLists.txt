project(auto_calibration_examples)
cmake_minimum_required(VERSION 3.10)

if(DEPTHAI_MERGED_TARGET)
    dai_add_example(auto_calibration_example auto_calibration_example.cpp ON OFF)
endif()
