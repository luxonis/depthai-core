project(rectification_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(rectification_alpha rectification_alpha.cpp ON OFF)
dai_set_example_test_labels(rectification_alpha ondevice rvc2_all rvc4 ci)
