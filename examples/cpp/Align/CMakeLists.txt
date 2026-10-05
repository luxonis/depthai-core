project(align_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(img_detections_align img_detections_align.cpp ON OFF)
dai_set_example_test_labels(img_detections_align ondevice rvc4 ci)

dai_add_example(custom_message_align custom_message_align.cpp ON OFF)
dai_set_example_test_labels(custom_message_align ondevice rvc2_all rvc4 ci)
