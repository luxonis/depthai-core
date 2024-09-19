    # Download files
    function(DownloadAndChecksum url url_checksum output status_var)

        # Check if file already downloaded (in resources)
        if(EXISTS "${output}")
            get_filename_component(_filename "${output}" NAME)
            message(STATUS "File already downloaded (resources): ${_filename}")
            set("${status_var}" "0" PARENT_SCOPE)
            return()
        endif()

        # Retry again if failed
        set(_num_retries_left ${DOWNLOADER_RETRY_NUM})
        # Set error by default
        set("${status_var}" "1" PARENT_SCOPE)

        while(NOT ${_num_retries_left} EQUAL 0)
            math(EXPR _num_retries_left "${_num_retries_left} - 1")

            # Download checksum first
            file(DOWNLOAD "${url_checksum}" "${output}.checksum" STATUS _status TIMEOUT ${DOWNLOADER_TIMEOUT_S})
            # Read checksum to file
            file(READ "${output}.checksum" _file_checksum)
            string(REGEX REPLACE "\n$" "" _file_checksum "${_file_checksum}")
            # Remove checksum file
            file(REMOVE "${output}.checksum")
            #CHECKS
            list(GET _status 0 _status_num)
            if(${_status_num})
                message(STATUS "Status error: ${_status}")
                set("${status_var}" "${_status_num}" PARENT_SCOPE)
                continue()
            endif()


            # Download file and validate checksum
            file(DOWNLOAD "${url}" "${output}" INACTIVITY_TIMEOUT ${DOWNLOADER_INACTIVE_TIMEOUT_S} STATUS _status TIMEOUT ${DOWNLOADER_TIMEOUT_S} SHOW_PROGRESS)

            #CHECKS
            list(GET _status 0 _status_num)
            if(${_status_num})
                message(STATUS "Status error: ${_status}")
                set("${status_var}" "${_status_num}" PARENT_SCOPE)
                continue()
            endif()

            # Now check if hash matches (if both files were downloaded without timeouts)
            file(SHA256 ${output} _downloaded_checksum)

            # if hashes don't match
            if(NOT (_downloaded_checksum STREQUAL _file_checksum))
                message(STATUS "Downloaded file checksum mismatch: ${_downloaded_checksum} != {_file_checksum}")
                set("${status_var}" "99" PARENT_SCOPE)
                continue()
            endif()

            # If no errors happened, set status to 0
            set("${status_var}" "0" PARENT_SCOPE)
            # And break the loop
            break()

        endwhile()

    endfunction()