project(nn_archive_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(nn_archive nn_archive.cpp OFF OFF)

dai_add_example(nn_archive_superblob nn_archive_superblob.cpp OFF OFF)