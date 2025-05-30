project(stereo_depth_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(depth_preview depth_preview.cpp ON OFF)
dai_set_example_test_labels(depth_preview ondevice rvc2_all rvc4 ci)

dai_add_example(stereo stereo.cpp ON OFF)
dai_set_example_test_labels(stereo ondevice rvc2_all rvc4 ci)

dai_add_example(stereo_depth_remap stereo_depth_remap.cpp ON OFF)
dai_set_example_test_labels(stereo_depth_remap ondevice rvc2_all rvc4 ci)