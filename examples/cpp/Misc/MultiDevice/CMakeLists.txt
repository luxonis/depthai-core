project(multi_device_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(multi_device_frame_sync multi_device_frame_sync.cpp OFF OFF)
dai_add_example(multi_device_host_node multi_device_host_node.cpp OFF OFF)
dai_add_example(device_to_device_relay device_to_device_relay.cpp OFF OFF)
