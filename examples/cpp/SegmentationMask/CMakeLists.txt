project(segmentation_mask_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(segmentation_tst segmentation_tst.cpp ON OFF)
dai_set_example_test_labels(segmentation_tst rvc2_all rvc4 ci)
