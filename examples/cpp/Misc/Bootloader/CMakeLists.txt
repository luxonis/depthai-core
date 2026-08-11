project(bootloader_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
dai_add_example(bootloader_dump bootloader_dump.cpp OFF OFF)
dai_add_example(flash_network_bootloader flash_network_bootloader.cpp OFF OFF)
