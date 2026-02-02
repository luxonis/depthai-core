project(april_tags_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

# Download lenna :0
if(DEPTHAI_FETCH_ARTIFACTS)
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/images/april_tags.jpg"
        FILE "april_tags.jpg"
        SHA1 "6818a531e71948bd28e1f0ab3e76b18aff6150fb"
        LOCATION april_tags
    )
endif()

dai_add_example(april_tags april_tags.cpp ON OFF)
dai_set_example_test_labels(april_tags ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(april_tags_12mp april_tags_12mp.cpp ON OFF)
dai_set_example_test_labels(april_tags_12mp ondevice rvc2_all rvc4 rvc4rgb ci)

if(DEPTHAI_FETCH_ARTIFACTS)
    dai_add_example(april_tags_replay april_tags_replay.cpp ON OFF)
    target_compile_definitions(april_tags_replay PRIVATE APRIL_TAGS_PATH="${april_tags}")
    dai_set_example_test_labels(april_tags_replay ondevice rvc2_all rvc4 rvc4rgb ci)
endif()
