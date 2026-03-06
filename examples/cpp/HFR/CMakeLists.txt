project(hfr_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(hfr_nn "hfr_nn.cpp" ON OFF)
dai_set_example_test_labels(hfr_nn ondevice rvc4 rvc4rgb ci)
dai_example_test_relax_fail_regex(hfr_nn)

dai_add_example(hfr_save_encoded "hfr_save_encoded.cpp" ON OFF)
dai_set_example_test_labels(hfr_save_encoded ondevice rvc4 rvc4rgb ci)
dai_example_test_relax_fail_regex(hfr_save_encoded)

dai_add_example(hfr_small_preview "hfr_small_preview.cpp" ON OFF)
dai_set_example_test_labels(hfr_small_preview ondevice rvc4 rvc4rgb ci)
dai_example_test_relax_fail_regex(hfr_small_preview)
