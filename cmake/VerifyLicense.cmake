function(DepthaiVerifyDownloadedLicense downloaded expected)
    if(NOT EXISTS "${downloaded}")
        message(FATAL_ERROR "Downloaded license file does not exist: ${downloaded}")
    endif()

    if(NOT EXISTS "${expected}")
        message(FATAL_ERROR "Committed license file does not exist: ${expected}")
    endif()

    file(SHA256 "${downloaded}" downloaded_sha)
    file(SHA256 "${expected}" expected_sha)

    if(NOT downloaded_sha STREQUAL expected_sha)
        message(FATAL_ERROR
            "Downloaded license does not match committed source license.\n"
            "Downloaded: ${downloaded}\n"
            "Committed:  ${expected}\n"
            "Downloaded SHA256: ${downloaded_sha}\n"
            "Committed SHA256:  ${expected_sha}\n"
            "Update the committed notice file if the third-party licenses changed intentionally."
        )
    else()
        message(STATUS "Downloaded license matches committed source license.")
    endif()
endfunction()
