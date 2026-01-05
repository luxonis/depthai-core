dai_add_example(simcc_parser simcc_parser.cpp ON OFF)
dai_set_example_test_labels(simcc_parser ondevice rvc4 ci)
target_link_libraries(simcc_parser PRIVATE utility depthai::hostNodesExt)
