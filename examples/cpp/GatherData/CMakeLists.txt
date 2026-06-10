project(gather_data_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(gather_detections gather_detections.cpp OFF OFF)
dai_add_example(gather_and_split_detections_simple gather_and_split_detections_simple.cpp OFF OFF)
dai_add_example(split_crops split_crops.cpp OFF OFF)
dai_add_example(multi_stage_detections multi_stage_detections.cpp OFF OFF)
