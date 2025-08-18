project(object_tracker_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)
# Video file with objects to detect
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/construction_vest.mp4"
    SHA1 "271d8d0b702e683ce02957db7c100843de5ceaec"
    FILE "construction_vest.mp4"
    LOCATION construction_vest
)

dai_add_example(object_tracker object_tracker.cpp ON OFF) 
dai_add_example(object_tracker_replay object_tracker_replay.cpp ON OFF)
target_compile_definitions(object_tracker_replay PRIVATE VIDEO_PATH="${construction_vest}")
dai_add_example(object_tracker_remap object_tracker_remap.cpp ON OFF) 
