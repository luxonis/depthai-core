# Extracts LICENSE and THIRD_PARTY_NOTICES.md from the bundled depthai-visualizer
# archive, combines them into a single file and verifies it against the committed
# notices/depthai-visualizer-LICENSE.
#
# Build integration (CMakeLists.txt):
#   DepthaiVisualizerLicense(<tarball> <committed-notice>)
#   Fails the configure step if the combined license differs from the committed one.
#   Pass -DDEPTHAI_UPDATE_VISUALIZER_LICENSE=ON to overwrite the committed notice instead
#   (one-shot: the option resets itself to OFF after the update).
#
# Standalone update (no configured build needed):
#   cmake -DTARBALL=build/resources/depthai-visualizer-<version>.tar.xz \
#         -P cmake/DepthaiVisualizerLicense.cmake
#   Optional: -DOUTPUT=<path> to write somewhere else than notices/depthai-visualizer-LICENSE.

set(_DEPTHAI_VISUALIZER_LICENSE_FILES "index/LICENSE" "index/THIRD_PARTY_NOTICES.md")

# Extracts the license files from `tarball` into `work_dir` and writes the combined
# license to `out_file`.
function(DepthaiVisualizerCombineLicense tarball work_dir out_file)
    if(NOT EXISTS "${tarball}")
        message(FATAL_ERROR "depthai-visualizer archive does not exist: ${tarball}")
    endif()

    file(REMOVE_RECURSE "${work_dir}")
    file(MAKE_DIRECTORY "${work_dir}")
    file(ARCHIVE_EXTRACT
        INPUT "${tarball}"
        DESTINATION "${work_dir}"
        PATTERNS ${_DEPTHAI_VISUALIZER_LICENSE_FILES}
    )

    set(combined "")
    set(first TRUE)
    foreach(entry IN LISTS _DEPTHAI_VISUALIZER_LICENSE_FILES)
        set(path "${work_dir}/${entry}")
        if(NOT EXISTS "${path}")
            message(FATAL_ERROR "depthai-visualizer archive is missing ${entry}: ${tarball}")
        endif()
        file(READ "${path}" content)
        string(REPLACE "\r" "" content "${content}")
        if(NOT content MATCHES "\n$")
            string(APPEND content "\n")
        endif()
        get_filename_component(name "${entry}" NAME)
        if(NOT first)
            string(APPEND combined "\n")
        endif()
        string(APPEND combined "================================================================================\n")
        string(APPEND combined "${name}\n")
        string(APPEND combined "================================================================================\n\n")
        string(APPEND combined "${content}")
        set(first FALSE)
    endforeach()

    file(WRITE "${out_file}" "${combined}")
endfunction()

# Combines the licenses from `tarball` and checks them against `expected`.
# With DEPTHAI_UPDATE_VISUALIZER_LICENSE=ON, `expected` is overwritten instead.
function(DepthaiVisualizerLicense tarball expected)
    get_filename_component(stem "${tarball}" NAME)
    string(REGEX REPLACE "\\.tar(\\.[a-z0-9]+)?$" "" stem "${stem}")
    set(work_dir "${CMAKE_CURRENT_BINARY_DIR}/visualizer-license/${stem}")
    set(generated "${work_dir}/depthai-visualizer-LICENSE")
    DepthaiVisualizerCombineLicense("${tarball}" "${work_dir}" "${generated}")

    if(DEPTHAI_UPDATE_VISUALIZER_LICENSE)
        file(COPY_FILE "${generated}" "${expected}")
        message(STATUS "Updated depthai-visualizer license notice: ${expected}")
        # One-shot: reset so later configures verify again instead of silently overwriting
        set(DEPTHAI_UPDATE_VISUALIZER_LICENSE OFF CACHE BOOL "Overwrite notices/depthai-visualizer-LICENSE with the license extracted from the bundled visualizer instead of failing on mismatch" FORCE)
        return()
    endif()

    if(NOT EXISTS "${expected}")
        message(FATAL_ERROR
            "You need to create the license notice for the bundled depthai-visualizer.\n"
            "Missing file: ${expected}\n"
            "Run this command, then commit the file:\n"
            "  cmake -DDEPTHAI_UPDATE_VISUALIZER_LICENSE=ON ${CMAKE_BINARY_DIR}"
        )
    endif()

    file(READ "${generated}" generated_content)
    file(READ "${expected}" expected_content)
    string(REPLACE "\r" "" expected_content "${expected_content}")

    if(NOT generated_content STREQUAL expected_content)
        message(FATAL_ERROR
            "You need to update the license notice for the bundled depthai-visualizer.\n"
            "The LICENSE + THIRD_PARTY_NOTICES.md inside ${tarball}\n"
            "differ from the committed ${expected}\n"
            "Run this command, then review and commit the updated file:\n"
            "  cmake -DDEPTHAI_UPDATE_VISUALIZER_LICENSE=ON ${CMAKE_BINARY_DIR}\n"
            "(The option resets itself to OFF after the update; the next configure verifies again.)"
        )
    endif()
    message(STATUS "depthai-visualizer license matches committed notice.")
endfunction()

# Script mode: cmake -DTARBALL=<archive> [-DOUTPUT=<file>] -P DepthaiVisualizerLicense.cmake
if(CMAKE_SCRIPT_MODE_FILE AND CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
    if(NOT TARBALL)
        message(FATAL_ERROR "Usage: cmake -DTARBALL=<depthai-visualizer-*.tar.xz> [-DOUTPUT=<file>] -P ${CMAKE_CURRENT_LIST_FILE}")
    endif()
    if(NOT OUTPUT)
        set(OUTPUT "${CMAKE_CURRENT_LIST_DIR}/../notices/depthai-visualizer-LICENSE")
    endif()
    get_filename_component(OUTPUT "${OUTPUT}" ABSOLUTE)
    get_filename_component(TARBALL "${TARBALL}" ABSOLUTE)
    get_filename_component(_stem "${TARBALL}" NAME)
    string(REGEX REPLACE "\\.tar(\\.[a-z0-9]+)?$" "" _stem "${_stem}")
    set(_work_dir "${CMAKE_CURRENT_LIST_DIR}/../build/visualizer-license/${_stem}")
    DepthaiVisualizerCombineLicense("${TARBALL}" "${_work_dir}" "${OUTPUT}")
    message(STATUS "Wrote ${OUTPUT}")
endif()
