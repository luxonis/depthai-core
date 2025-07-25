## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(system_logger system_logger.cpp ON OFF)
dai_set_example_test_labels(system_logger ondevice rvc2_all ci)