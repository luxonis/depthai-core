project(image_manip_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)


# Download lenna :0
private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/images/lenna.png"
    FILE "lenna.png"
    SHA1 "3ee0d360dc12003c0d43e3579295b52b64906e85"
    LOCATION lenna
)

dai_add_example(image_manip_v2_resize image_manip_v2_resize.cpp OFF OFF)

dai_add_example(image_manip_v2_multi_ops image_manip_v2_multi_ops.cpp OFF OFF)

dai_add_example(image_manip_v2_all_ops image_manip_v2_all_ops.cpp OFF OFF)
target_compile_definitions(image_manip_v2_all_ops PRIVATE LENNA_PATH="${lenna}")

dai_add_example(image_manip_v2_remap image_manip_v2_remap.cpp OFF OFF)