project(host_nodes_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(host_node host_node.cpp ON OFF)
dai_set_example_test_labels(host_node ondevice rvc2_all rvc4 ci)

dai_add_example(image_manip_host image_manip_host.cpp ON OFF)
dai_set_example_test_labels(image_manip_host ondevice rvc2_all rvc4 ci)

dai_add_example(image_manip_color_conversion_host image_manip_color_conversion.cpp ON OFF)
dai_set_example_test_labels(image_manip_color_conversion_host ondevice rvc2_all rvc4 ci)

dai_add_example(display display.cpp ON OFF)
dai_set_example_test_labels(display ondevice rvc2_all rvc4 ci)

dai_add_example(host_camera host_camera.cpp OFF OFF)

dai_add_example(threaded_host_node threaded_host_node.cpp ON OFF)
dai_set_example_test_labels(threaded_host_node ondevice rvc2_all rvc4 ci)

dai_add_example(host_pipeline_synced_node host_pipeline_synced_node.cpp ON OFF)
dai_set_example_test_labels(host_pipeline_synced_node ondevice rvc2_all rvc4 ci)

dai_add_example(host_only_camera host_only_camera.cpp OFF OFF)