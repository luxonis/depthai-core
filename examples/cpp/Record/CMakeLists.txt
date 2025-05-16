project(record_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(record_encoded record_encoded.cpp ON OFF)

dai_add_example(record_raw record_raw.cpp ON OFF)

dai_add_example(record_imu record_imu.cpp ON OFF)

dai_add_example(holistic_record holistic_record.cpp ON OFF)