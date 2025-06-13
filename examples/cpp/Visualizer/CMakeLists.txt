project(visualizer_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    dai_add_example(visualizer_yolo visualizer_yolo.cpp ON OFF)
    dai_set_example_test_labels(visualizer_yolo ondevice rvc2_all rvc4)

    dai_add_example(custom_services custom_services.cpp OFF OFF)

    dai_add_example(visualizer_encoded visualizer_encoded.cpp OFF OFF)

    dai_add_example(custom_visualizations custom_visualizations.cpp OFF OFF)
endif()

