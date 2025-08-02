project(benchmark_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(benchmark_camera "benchmark_camera.cpp" ON OFF)
dai_set_example_test_labels(benchmark_camera ondevice rvc2_all rvc4 rvc4rgb ci)
dai_example_test_relax_fail_regex(benchmark_camera)

dai_add_example(benchmark_nn "benchmark_nn.cpp" ON OFF)
dai_set_example_test_labels(benchmark_nn ondevice rvc2_all rvc4 rvc4rgb ci)

dai_add_example(benchmark_simple "benchmark_simple.cpp" ON OFF)
dai_set_example_test_labels(benchmark_simple ondevice rvc2_all rvc4 rvc4rgb ci)