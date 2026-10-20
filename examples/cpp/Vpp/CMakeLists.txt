project(vpp_examples)
cmake_minimum_required(VERSION 3.10)

dai_add_example(vpp virtual_patern_projection.cpp ON OFF)
dai_add_example(tof_stereo_vpp tof_stereo_vpp.cpp ON OFF)
