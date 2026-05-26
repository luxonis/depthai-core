project(gpu_stereo_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(gpu_stereo gpu_stereo.cpp ON OFF)
dai_set_example_test_labels(gpu_stereo ondevice rvc4 ci)
