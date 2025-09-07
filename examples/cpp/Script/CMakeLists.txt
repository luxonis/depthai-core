project(script_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(script_simple script_simple.cpp ON OFF)
dai_set_example_test_labels(script_simple ondevice rvc2_all rvc4 rvc4rgb ci)
dai_example_test_relax_fail_regex(script_simple)

dai_add_example(script_switch_all_cameras script_switch_all_cameras.cpp ON OFF)
dai_set_example_test_labels(script_switch_all_cameras ondevice rvc2_all rvc4 rvc4rgb ci)