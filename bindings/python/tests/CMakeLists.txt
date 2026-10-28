
set(TARGET_TEST_MODULE "depthai_pybind11_tests")

# Specify path separator
set(SYS_PATH_SEPARATOR ";")
if(UNIX)
    set(SYS_PATH_SEPARATOR ":")
endif()

set(PYBIND11_TEST_FILES
    "test.py"
    "xlink_exceptions_test.cpp"
    "xlink_protocol_test.py"
    "utf8_support_test.py"
    # "dai_path_conversion_test.py" requires device
    "nn_archive_gil_test.py"
    "test_registration_test.py"
    "nndata_tensor_test.py"
    "pcldata_set_points_test.py"
    "remote_connection_service_test.py"
    "messsage_queue_test.py"
    "imgframe_test.py"
    "inherited_messages_test.py"
    "dynamic_calibration_test.py"
    "auto_calibration_config_test.py"
    "host_node_create_deprecation_test.py"
    "pipeline_create_device_arg_test.py"
    "stitching_test.py"
    "neural_network_model_test.cpp"
)

if(DEPTHAI_PYTHON_GIL_FREE)
    list(APPEND PYBIND11_TEST_FILES "freethreading_test.py")
endif()

if(DEPTHAI_ENABLE_CURL)
    list(APPEND PYBIND11_TEST_FILES
        "model_zoo_gil_test.py"
    )
endif()

if(DEPTHAI_BUILD_BETA)
    list(APPEND PYBIND11_TEST_FILES
        "beta_img_detections_filter_test.py"
        "beta_parser_config_test.py"
    )
endif()

string(REPLACE ".cpp" ".py" PYBIND11_PYTEST_FILES "${PYBIND11_TEST_FILES}")

# Create the binding library at the end
pybind11_add_module(${TARGET_TEST_MODULE} THIN_LTO ${TARGET_TEST_MODULE}.cpp ${PYBIND11_TEST_FILES})
target_compile_definitions(${TARGET_TEST_MODULE} PRIVATE
    "DEPTHAI_PYTHON_GIL_FREE=$<BOOL:${DEPTHAI_PYTHON_GIL_FREE}>")

set(DEPTHAI_PYTEST_COMMAND
    ${CMAKE_COMMAND} -E env
    "PATH=${HUNTER_INSTALL_PREFIX}/bin${SYS_PATH_SEPARATOR}$ENV{PATH}"
    "PYTHONPATH=$<TARGET_FILE_DIR:${TARGET_NAME}>${SYS_PATH_SEPARATOR}$<TARGET_FILE_DIR:${TARGET_TEST_MODULE}>${SYS_PATH_SEPARATOR}${DEPTHAI_PYTHON_TEST_EXTRA_PATH}${SYS_PATH_SEPARATOR}$ENV{PYTHONPATH}"
    ${ASAN_ENVIRONMENT_VARS}
    ${DEPTHAI_PYTHON_TEST_ENVIRONMENT}
    ${PYTHON_EXECUTABLE} -m pytest ${PYBIND11_PYTEST_FILES}
)

add_test(NAME python_bindings COMMAND ${DEPTHAI_PYTEST_COMMAND})
set_tests_properties(python_bindings PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    LABELS "python_unit"
    TIMEOUT 300
)

# A single command to compile and run the tests
add_custom_target(
    pytest COMMAND ${DEPTHAI_PYTEST_COMMAND}
    DEPENDS
        ${TARGET_TEST_MODULE} # Compiled tests
        ${TARGET_NAME} # DepthAI Python Library
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    USES_TERMINAL
)

# Link to depthai
target_link_libraries(${TARGET_TEST_MODULE} PRIVATE pybind11::pybind11 depthai::core)
