project(neural_network_examples)
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

dai_add_example(neural_network neural_network.cpp ON OFF)
dai_set_example_test_labels(neural_network ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(neural_network_multi_input neural_network_multi_input.cpp ON OFF)
target_compile_definitions(neural_network_multi_input PRIVATE LENNA_PATH="${lenna}")
dai_set_example_test_labels(neural_network_multi_input ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(neural_network_multi_input_combined neural_network_multi_input_combined.cpp ON OFF)
target_compile_definitions(neural_network_multi_input_combined PRIVATE LENNA_PATH="${lenna}")
