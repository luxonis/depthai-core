# Check of a prebuilt private module (cmake -P, from DepthaiDeviceRvc4VendorPrebuilt.cmake). Fails when a file in one of the
# header lists, or an interface define of this build, differs from the ones that the archive was compiled against.
#
# Input (-D):
#   PREBUILT_DIR, VERSION                  for the message
#   HEADER_LISTS                           ;-list of files with lines "<sha256>  <source|core|build>/<path>"
#   SOURCE_DIR, CORE_DIR, BINARY_DIR       the roots of source/, core/ and build/
#   EXPECTED_DEFINES, ACTUAL_DEFINES       optional: one define per line
#   CONFIGURE                              ON: the configure step; files of build/ that do not exist yet are skipped
#   STAMP                                  optional: touched when the check passes

cmake_minimum_required(VERSION 3.13)

set(_report "")
foreach(_list IN LISTS HEADER_LISTS)
    file(STRINGS "${_list}" _lines)
    foreach(_line IN LISTS _lines)
        if(NOT _line MATCHES "^([0-9a-f]+)  (source|core|build)/(.+)$")
            message(FATAL_ERROR "Malformed line in ${_list}: '${_line}'")
        endif()
        set(_expected "${CMAKE_MATCH_1}")
        set(_key "${CMAKE_MATCH_2}/${CMAKE_MATCH_3}")
        if(CMAKE_MATCH_2 STREQUAL "source")
            set(_path "${SOURCE_DIR}/${CMAKE_MATCH_3}")
        elseif(CMAKE_MATCH_2 STREQUAL "core")
            set(_path "${CORE_DIR}/${CMAKE_MATCH_3}")
        else()
            set(_path "${BINARY_DIR}/${CMAKE_MATCH_3}")
        endif()
        if(NOT EXISTS "${_path}")
            if(NOT (CONFIGURE AND CMAKE_MATCH_2 STREQUAL "build"))
                string(APPEND _report "\n  missing: ${_key}")
            endif()
        else()
            file(SHA256 "${_path}" _actual)
            if(NOT _actual STREQUAL _expected)
                string(APPEND _report "\n  changed: ${_key}")
            endif()
        endif()
    endforeach()
endforeach()

if(ACTUAL_DEFINES)
    file(STRINGS "${EXPECTED_DEFINES}" _expected_defines)
    file(STRINGS "${ACTUAL_DEFINES}" _actual_defines)
    foreach(_defines _expected_defines _actual_defines)
        list(REMOVE_DUPLICATES ${_defines})
    endforeach()
    foreach(_define IN LISTS _expected_defines)
        if(NOT _define IN_LIST _actual_defines)
            string(APPEND _report "\n  define only in the archive's build: ${_define}")
        endif()
    endforeach()
    foreach(_define IN LISTS _actual_defines)
        if(NOT _define IN_LIST _expected_defines)
            string(APPEND _report "\n  define only in this build: ${_define}")
        endif()
    endforeach()
endif()

if(_report)
    message(FATAL_ERROR
        "The prebuilt private module ${VERSION} (${PREBUILT_DIR}) was compiled against other public headers or "
        "defines than this build:${_report}\n"
        "A changed header or define can change the binary interface between the firmware and the archive, so this "
        "build cannot use the archive. Either\n"
        "  * revert these changes, or\n"
        "  * build a commit whose DEPTHAI_DEVICE_RVC4_VENDOR_VERSION (cmake/Depthai/DepthaiDeviceRVC4Config.cmake) was built "
        "with these changes, or set DEPTHAI_DEVICE_RVC4_VENDOR to a vendor archive built from them, or\n"
        "  * build the private module from its sources (DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR; Luxonis licensees only).")
endif()
if(STAMP)
    file(TOUCH "${STAMP}")
endif()
