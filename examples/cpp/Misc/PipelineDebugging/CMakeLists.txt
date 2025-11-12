project(pipeline_debugging_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(get_pipeline_state get_pipeline_state.cpp ON OFF)
dai_add_example(node_pipeline_events node_pipeline_events.cpp ON OFF)
