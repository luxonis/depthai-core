project(warp_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(warp_mesh warp_mesh.cpp ON OFF)
dai_set_example_test_labels(warp_mesh ondevice rvc2_all rvc4 rvc4rgb ci)
