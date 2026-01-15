project(gate_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(gate_node_example gate_example.cpp ON OFF)
dai_set_example_test_labels(gate_node_example ondevice rvc2_all rvc4 rvc4rgb ci)
