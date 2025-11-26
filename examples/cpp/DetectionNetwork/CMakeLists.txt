project(detection_network_examples)
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

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    dai_add_example(detection_network_replay detection_network_replay.cpp OFF OFF)
    target_compile_definitions(detection_network_replay PRIVATE VIDEO_PATH="${construction_vest}")
endif()

dai_add_example(detection_network detection_network.cpp ON OFF)
dai_set_example_test_labels(detection_network ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(detection_network_remap detection_network_remap.cpp ON OFF)
dai_set_example_test_labels(detection_network_remap ondevice rvc2_all rvc4 ci)

dai_add_example(detection_and_segmentation detection_and_segmentation.cpp ON OFF)
dai_set_example_test_labels(detection_and_segmentation rvc2_all rvc4 ci)

dai_add_example(detection_and_keypoints detection_and_keypoints.cpp ON OFF)
dai_set_example_test_labels(detection_and_keypoints rvc2_all rvc4 ci)
