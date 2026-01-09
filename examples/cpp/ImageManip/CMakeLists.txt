project(image_manip_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)


# Download lenna :0
if(DEPTHAI_FETCH_ARTIFACTS)
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/images/lenna.png"
        FILE "lenna.png"
        SHA1 "3ee0d360dc12003c0d43e3579295b52b64906e85"
        LOCATION lenna
    )
endif()

dai_add_example(image_manip_resize image_manip_resize.cpp ON OFF)
dai_set_example_test_labels(image_manip_resize ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(image_manip_multi_ops image_manip_multi_ops.cpp ON OFF)
dai_set_example_test_labels(image_manip_multi_ops ondevice rvc2_all rvc4 rvc4rgb ci)

if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_example(image_manip_all_ops image_manip_all_ops.cpp ON OFF)
    target_compile_definitions(image_manip_all_ops PRIVATE LENNA_PATH="${lenna}")
    dai_set_example_test_labels(image_manip_all_ops ondevice rvc2_all rvc4 rvc4rgb ci)
endif()

dai_add_example(image_manip_remap image_manip_remap.cpp ON OFF)
dai_set_example_test_labels(image_manip_remap ondevice rvc2_all rvc4 rvc4rgb ci)
