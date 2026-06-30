project(depth_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(unified_depth unified_depth.cpp ON OFF)
dai_set_example_test_labels(unified_depth ondevice rvc2_all rvc4)
