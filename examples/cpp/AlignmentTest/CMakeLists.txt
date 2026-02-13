project(alignment_test)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)


dai_add_example(alignment_test alignmentTest.cpp ON OFF)
dai_set_example_test_labels(alignment_test ondevice rvc2_all rvc4 rvc4rgb ci)
