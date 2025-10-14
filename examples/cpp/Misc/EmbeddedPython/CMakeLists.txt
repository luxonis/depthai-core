project(python_from_cpp_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(build_only_pipeline_from_cpp build_only_pipeline_from_cpp.cpp OFF OFF)
target_link_libraries(build_only_pipeline_from_cpp PRIVATE depthai-embed
			pybind11::pybind11
			pybind11::module
			pybind11::embed)

dai_add_example(run_pipeline_from_cpp run_pipeline_from_cpp.cpp OFF OFF)
target_link_libraries(run_pipeline_from_cpp PRIVATE depthai-embed
		pybind11::pybind11
		pybind11::module
		pybind11::embed)

# Copy Python file to the same directory as the executable
add_custom_command(TARGET build_only_pipeline_from_cpp POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		${CMAKE_CURRENT_SOURCE_DIR}/build_pipeline.py
		$<TARGET_FILE_DIR:build_only_pipeline_from_cpp>/build_pipeline.py
		COMMENT "Copying Python script to build directory"
)
add_custom_command(TARGET run_pipeline_from_cpp POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		${CMAKE_CURRENT_SOURCE_DIR}/build_pipeline.py
		$<TARGET_FILE_DIR:run_pipeline_from_cpp>/build_pipeline.py
		COMMENT "Copying Python script to build directory"
)
