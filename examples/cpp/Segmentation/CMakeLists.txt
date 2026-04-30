project(segmentation_mask_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(semantic_segmentation semantic_segmentation.cpp ON OFF)
dai_set_example_test_labels(semantic_segmentation rvc2_all rvc4 ci)
