project(benchmark_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(benchmark_camera "benchmark_camera.cpp" FALSE FALSE)
dai_add_example(benchmark_nn "benchmark_nn.cpp" FALSE FALSE)
dai_add_example(benchmark_simple "benchmark_simple.cpp" FALSE FALSE)
