project(image_manip_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(image_manip_v2_resize image_manip_v2_resize.cpp OFF OFF)

dai_add_example(image_manip_v2_multi_ops image_manip_v2_multi_ops.cpp OFF OFF)

dai_add_example(image_manip_color_conversion image_manip_color_conversion.cpp OFF OFF)
