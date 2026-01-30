project(events_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

if(DEPTHAI_ENABLE_EVENTS_MANAGER)
    dai_add_example(events events.cpp OFF OFF)
    dai_add_example(events_file_group events_file_group.cpp OFF OFF)
endif()