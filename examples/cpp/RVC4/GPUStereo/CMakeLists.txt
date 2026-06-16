project(rvc4_gpu_stereo_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(check_device_gpu check_device_gpu.cpp ON OFF)
dai_set_example_test_labels(check_device_gpu rvc4)
