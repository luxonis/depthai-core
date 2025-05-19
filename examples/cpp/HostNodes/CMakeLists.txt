project(host_nodes_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(host_node host_node.cpp OFF OFF)

dai_add_example(threaded_host_node threaded_host_node.cpp OFF OFF)

dai_add_example(image_manip_host image_manip_host.cpp OFF OFF)

dai_add_example(image_manip_color_conversion_host image_manip_color_conversion.cpp OFF OFF)

dai_add_example(display display.cpp OFF OFF)