# Helper to create a pybind11_mkdoc target which takes
include(target-public-headers)

# Usage:
# target_pybind11_mkdoc_setup([path/to/output/docstring.hpp] [Library for which to generate: target-name] [Enforce pybind11_mkdoc existing ON/OFF])
function(target_pybind11_mkdoc_setup_combined output_file_base enforce targets)

    set(all_filtered_header_files)
    set(include_directories)
    set(compile_definitions)

    foreach(target ${targets})
        # Get unaliased target if alias target passed in
        get_target_property(original_target ${target} ALIASED_TARGET)
        if(original_target)
            set(target ${original_target})
        endif()

        # Check if target is valid
        if(NOT TARGET ${target})
            message(FATAL_ERROR "Target ${target} does not exist.")
        endif()

        # Get target public headers
        get_target_public_headers(${target} header_files)
        foreach(header ${header_files})
            if(NOT header MATCHES "^/usr/include")
                list(APPEND all_filtered_header_files ${header})
            endif()
        endforeach()

        # Collect include directories and compile definitions
        get_target_property(target_include_dirs ${target} INTERFACE_INCLUDE_DIRECTORIES)
        get_target_property(target_compile_defs ${target} INTERFACE_COMPILE_DEFINITIONS)

        if(target_include_dirs)
            list(APPEND include_directories ${target_include_dirs})
        endif()

        if(target_compile_defs)
            list(APPEND compile_definitions ${target_compile_defs})
        endif()
    endforeach()

    # Construct a unique output file path for the combined targets
    set(output_file "${output_file_base}")

    # Setup mkdoc target for the combined headers
    pybind11_mkdoc_setup_internal_combined("${output_file}" "${all_filtered_header_files}" "${include_directories}" "${compile_definitions}" ${enforce})
endfunction()

# Internal helper, sets up pybind11_mkdoc target for combined headers
function(pybind11_mkdoc_setup_internal_combined output_path mkdoc_headers include_dirs compile_defs enforce)

    # Constants
    set(PYBIND11_MKDOC_MODULE_NAME "pybind11_mkdoc")
    set(PYBIND11_MKDOC_TARGET_NAME "pybind11_mkdoc")

    # Execute module pybind11_mkdoc to check if present
    message(STATUS "Checking for pybind11_mkdoc")
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -m ${PYBIND11_MKDOC_MODULE_NAME} --help RESULT_VARIABLE error OUTPUT_QUIET ERROR_QUIET)
    if(error)
        set(message "Checking for pybind11_mkdoc - not found, docstrings not available")
        if(NOT enforce)
            message(STATUS ${message})
        else()
            message(FATAL_ERROR ${message})
        endif()
        # Exit
        return()
    else()
        message(STATUS "Checking for pybind11_mkdoc - found, docstrings available")
    endif()

    # Prepare the output folder for the mkdoc
    get_filename_component(output_directory "${output_path}" DIRECTORY)
    # Create the command
    add_custom_command(
        OUTPUT "${output_path}"
        # Create directory first (if it doesn't exist)
        COMMAND ${CMAKE_COMMAND} -E make_directory "${output_directory}"
        # Execute mkdoc
        COMMAND
            ${PYTHON_EXECUTABLE}
            -m ${PYBIND11_MKDOC_MODULE_NAME}
            # Docstring wrap width
            -w 80
            -o "${output_path}"
            # C++ standard
            -std=c++17
            # List of include directories
            "-I$<JOIN:${include_dirs},;-I>"
            # List of compiler definitions
            "-D$<JOIN:${compile_defs},;-D>"
            # List of headers for which to generate docstrings
            ${mkdoc_headers}
            # Redirect stderr to not spam output
            # 2> /dev/null
        DEPENDS ${mkdoc_headers} #${target}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Creating docstrings with ${PYTHON_EXECUTABLE} -m ${PYBIND11_MKDOC_MODULE_NAME} ..."
        VERBATIM
        COMMAND_EXPAND_LISTS
    )

    # Create a target
    add_custom_target(
        ${PYBIND11_MKDOC_TARGET_NAME}
        DEPENDS "${output_path}"
    )

    # Force target build
    file(TOUCH_NOCREATE ${mkdoc_headers})

endfunction()
