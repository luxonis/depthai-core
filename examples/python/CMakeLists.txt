# Prevent from being configured
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(FATAL_ERROR "Use projects root CMakeLists.txt to configure")
endif()

# Add options
option(DEPTHAI_PYTHON_TEST_EXAMPLES "Test examples - examples will be ran as a part of the test suite" OFF)

# Specify path separator
set(SYS_PATH_SEPARATOR ";")
if(UNIX)
    set(SYS_PATH_SEPARATOR ":")
endif()

set(TARGET_NAME "depthai")

# Add a target to install_requirements (added to ALL)
add_custom_target(install_requirements ALL
    COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/install_requirements.py" "--skip_depthai"
    DEPENDS ${TARGET_NAME}
    VERBATIM
    COMMAND_EXPAND_LISTS
)

# Function for adding new python test
function(add_python_example example_name python_script_path)
    # Modify example name to signify that its Python based
    set(example_name "py_${example_name}")

    # parse the rest of the arguments
    set(arguments ${ARGV})
    list(REMOVE_AT arguments 0 1)

    # Creates a target (python my_test [args])
    add_custom_target(${example_name}
        ${CMAKE_COMMAND} -E env
        # Environment variables
        # PATH (dlls)
        "PATH=${HUNTER_INSTALL_PREFIX}/bin${SYS_PATH_SEPARATOR}$ENV{PATH}"
        # Python path (to find compiled module)
        "PYTHONPATH=$<TARGET_FILE_DIR:${TARGET_NAME}>${SYS_PATH_SEPARATOR}$ENV{PYTHONPATH}"
        # ASAN in case of sanitizers
        "${ASAN_ENVIRONMENT_VARS}"
        # Example
        ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/${python_script_path} ${ARGN}
        DEPENDS ${TARGET_NAME} install_requirements
        VERBATIM
        COMMAND_EXPAND_LISTS
    )

    if(DEPTHAI_PYTHON_TEST_EXAMPLES)

        # Adds test with 5 seconds timeout and bumps all python warnings to errors
        add_test(NAME ${example_name} COMMAND
            ${CMAKE_COMMAND} -E env
            # PATH (dlls)
            "PATH=${HUNTER_INSTALL_PREFIX}/bin${SYS_PATH_SEPARATOR}$ENV{PATH}"
            # Python path (to find compiled module)
            "PYTHONPATH=$<TARGET_FILE_DIR:${TARGET_NAME}>${SYS_PATH_SEPARATOR}$ENV{PYTHONPATH}"
            "DEPTHAI_SEARCH_TIMEOUT=15000"
            # ASAN in case of sanitizers
            ${ASAN_ENVIRONMENT_VARS}
            ${CMAKE_COMMAND} -DFORCE_TIMEOUT_SECONDS=10 -P ${CMAKE_CURRENT_LIST_DIR}/../cpp/cmake/ExecuteTestTimeout.cmake

            # Actual script to run
            ${PYTHON_EXECUTABLE} -Werror "${CMAKE_CURRENT_LIST_DIR}/${python_script_path}" ${arguments}
        )

        # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
        set_tests_properties (${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION "\\[warning\\];\\[error\\];\\[critical\\]")

    endif()

endfunction()


# Function for setting test labels, including label shortcuts
function(dai_set_example_test_labels example_name)
    # Modify example name to match the one created by add_python_example
    set(example_name "py_${example_name}")
    set(expanded_labels "")
    foreach(label IN LISTS ARGN)
        if(label STREQUAL "rvc2_all")
            list(APPEND expanded_labels "rvc2" "poe" "usb")
        else()
            list(APPEND expanded_labels ${label})
        endif()
    endforeach()
    # Add "cpp_example" label
    list(APPEND expanded_labels "python_example")
    set_tests_properties(${example_name} PROPERTIES LABELS "${expanded_labels}")
endfunction()

if(DEPTHAI_PYTHON_TEST_EXAMPLES)

    # Adds install requirements test with 5 minute timeout
    add_test(NAME install_requirements COMMAND
        ${CMAKE_COMMAND} -E env
        # PATH (dlls)
        "PATH=${HUNTER_INSTALL_PREFIX}/bin${SYS_PATH_SEPARATOR}$ENV{PATH}"
        # Python path (to find compiled module)
        "PYTHONPATH=$<TARGET_FILE_DIR:${TARGET_NAME}>${SYS_PATH_SEPARATOR}$ENV{PYTHONPATH}"
        # ASAN in case of sanitizers
        ${ASAN_ENVIRONMENT_VARS}
        ${CMAKE_COMMAND} -DFORCE_TIMEOUT_SECONDS=300 -P ${CMAKE_CURRENT_LIST_DIR}/../cpp/cmake/ExecuteTestTimeout.cmake

        # Actual script to run
        ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/install_requirements.py" "--skip_depthai"
    )

    # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
    set_tests_properties (install_requirements PROPERTIES FAIL_REGULAR_EXPRESSION "\\[warning\\];\\[error\\];\\[critical\\]")

endif()


## Camera output
add_python_example(camera_output Camera/camera_output.py)
dai_set_example_test_labels(camera_output ondevice rvc2_all rvc4 ci)

add_python_example(camera_all Camera/camera_all.py)
dai_set_example_test_labels(camera_all ondevice rvc2_all rvc4 ci)
