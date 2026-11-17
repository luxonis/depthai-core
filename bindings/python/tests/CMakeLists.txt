
set(TARGET_TEST_MODULE "depthai_pybind11_tests")

# Specify path separator
set(SYS_PATH_SEPARATOR ";")
if(UNIX)
    set(SYS_PATH_SEPARATOR ":")
endif()

set(PYBIND11_TEST_FILES
    "test.py"
    "api_binding_coverage_test.py"
    # TODO: validate api_binding_device_test.py on RVC2/RVC4 in HIL before enabling.
    "queue_gil_test.py"
    "xlink_exceptions_test.cpp"
    "xlink_protocol_test.py"
    "chrono_operator_lookup_test.cpp"
    "utf8_support_test.py"
    "dai_path_conversion_test.py"
    "nn_archive_gil_test.py"
    "nn_archive_head_test.py"
    "test_registration_test.py"
    "nndata_tensor_test.py"
    "nndata_dtype_test.py"
    "buffer_lifetime_test.py"
    "calibration_constructor_test.py"
    "openvino_version_test.py"
    "pcldata_set_points_test.py"
    "backend_compatibility_test.py"
    "remote_connection_service_test.py"
    "messsage_queue_test.py"
    "imgframe_test.py"
    "inherited_messages_test.py"
    "dynamic_calibration_test.py"
    "auto_calibration_config_test.py"
    "host_node_create_deprecation_test.py"
    "pipeline_create_device_arg_test.py"
    "compiler_warning_bindings_roundtrip_test.py"
    "stitching_test.py"
    "img_detections_filter_test.py"
    "filter_params_test.py"
    "neural_network_model_test.cpp"
    "node_holder_test.cpp"
    "message_holder_test.cpp"
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

# On macOS, resolve SDK symbols from the depthai Python extension. Statically
# linking the SDK here copies its Objective-C classes into a second image and
# causes duplicate-class registration when both Python modules are imported.
# That link is also what carries cxx_std_17 and system-header usage
# requirements. Apply them by hand. Without cxx_std_17 Apple Clang compiles
# this target as C++14, and copying INCLUDE_DIRECTORIES drops SYSTEM, so
# libnop's `.template Construct(...)` calls are rejected.
if(APPLE)
    target_link_libraries(${TARGET_TEST_MODULE} PRIVATE pybind11::pybind11)
    target_compile_features(${TARGET_TEST_MODULE} PRIVATE cxx_std_17)
    # INTERFACE libraries: restores -isystem without linking an archive.
    # List these before the include copy. CMake keeps the first classification
    # of a path, and the copy below is non-system.
    target_link_libraries(${TARGET_TEST_MODULE} PRIVATE libnop)
    if(TARGET xtensor)
        target_link_libraries(${TARGET_TEST_MODULE} PRIVATE xtensor)
    endif()
    target_include_directories(${TARGET_TEST_MODULE} SYSTEM PRIVATE
        "$<TARGET_PROPERTY:depthai::core,INTERFACE_SYSTEM_INCLUDE_DIRECTORIES>")
    target_include_directories(${TARGET_TEST_MODULE} PRIVATE
        "$<TARGET_PROPERTY:${TARGET_NAME},INCLUDE_DIRECTORIES>")
    target_compile_definitions(${TARGET_TEST_MODULE} PRIVATE
        "$<TARGET_PROPERTY:${TARGET_NAME},COMPILE_DEFINITIONS>")
    # pybind11_add_module supplies -undefined dynamic_lookup on macOS.
    # Import depthai before this module so its exported SDK symbols are loaded.
    add_dependencies(${TARGET_TEST_MODULE} ${TARGET_NAME})
else()
    target_link_libraries(${TARGET_TEST_MODULE} PRIVATE pybind11::pybind11 depthai::core)
endif()
