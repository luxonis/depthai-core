# DepthaiVerifyDownloadedLicense(<license file> <committed notice file> [<origin>])
#   Stops when <license file> is not byte-identical to the committed notice file (notices/ of this repository, which
#   the wheel and the install ship). <origin> names where the license file comes from in the messages: Downloaded (the
#   default) or Built.
#
# Script mode, for the same check at build time (for example after a build of the RVC4 firmware from device/rvc4):
#   cmake -D LICENSE_FILE=<license file> -D EXPECTED=<committed notice file> [-D ORIGIN=Built] -P cmake/VerifyLicense.cmake
function(DepthaiVerifyDownloadedLicense downloaded expected)
    set(origin "Downloaded")
    if(ARGC GREATER 2 AND NOT "${ARGV2}" STREQUAL "")
        set(origin "${ARGV2}")
    endif()
    string(TOLOWER "${origin}" origin_lower)

    if(NOT EXISTS "${downloaded}")
        message(FATAL_ERROR "${origin} license file does not exist: ${downloaded}")
    endif()

    if(NOT EXISTS "${expected}")
        message(FATAL_ERROR "Committed license file does not exist: ${expected}")
    endif()

    file(SHA256 "${downloaded}" downloaded_sha)
    file(SHA256 "${expected}" expected_sha)

    if(NOT downloaded_sha STREQUAL expected_sha)
        message(FATAL_ERROR
            "${origin} license does not match committed source license.\n"
            "${origin}: ${downloaded}\n"
            "Committed:  ${expected}\n"
            "${origin} SHA256: ${downloaded_sha}\n"
            "Committed SHA256:  ${expected_sha}\n"
            "Update the committed notice file if the third-party licenses changed intentionally.\n"
            "To update it, copy the ${origin_lower} license over the committed one, then review and commit it:\n"
            "  cmake -E copy \"${downloaded}\" \"${expected}\""
        )
    else()
        message(STATUS "${origin} license matches committed source license.")
    endif()
endfunction()

if(CMAKE_SCRIPT_MODE_FILE AND CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
    if(NOT DEFINED LICENSE_FILE OR NOT DEFINED EXPECTED)
        message(FATAL_ERROR "Usage: cmake -D LICENSE_FILE=<file> -D EXPECTED=<committed notice file> [-D ORIGIN=<origin>] -P ${CMAKE_CURRENT_LIST_FILE}")
    endif()
    DepthaiVerifyDownloadedLicense("${LICENSE_FILE}" "${EXPECTED}" "${ORIGIN}")
endif()
