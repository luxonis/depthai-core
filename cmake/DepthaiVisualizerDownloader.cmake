# This script downloads depthai device side artifacts for different device types
include(DownloadAndChecksum)
function(DepthaiVisualizerDownloader)

    ### VARIABLES
    # Artifactory
    set(DOWNLOADER_BASE_URL "https://artifacts.luxonis.com/artifactory")

    # Errors and retry count
    set(DOWNLOADER_TIMEOUT_S 300)
    set(DOWNLOADER_INACTIVE_TIMEOUT_S 60)
    set(DOWNLOADER_RETRY_NUM 5)
    ### END VARIABLES

    # PARSE ARGUMENTS
    set(repo "luxonis-depthai-visualizer-local")
    set(visualizer_hash ${ARGV0})
    set(folder "${ARGV1}")
    set(output_list_var "${ARGV2}")
    # END PARSE ARGUMENTS

    string(CONFIGURE "${DOWNLOADER_BASE_URL}/${repo}" _download_directory_url)
    message(STATUS "Downloading ${repo}-${visualizer_hash}")

    # Prints error message
    macro(PrintErrorMessage status)
        if(${status} EQUAL 22)
            message(STATUS "Resource not found, check if commit hash is correctly specified.\n")
        elseif(${status} EQUAL 28)
            message(STATUS "Timeout.\n")
        elseif(${status} EQUAL 99)
            message(STATUS "Couldn't retrieve files correctly, checksum mismatch.")
        else()
            message(STATUS "Unknown error.\n")
        endif()
    endmacro()


    # Download firmware package
    message(STATUS "Downloading and checking ${device_type}-fwp.tar.xz")
    DownloadAndChecksum(
        "${_download_directory_url}/${visualizer_hash}/depthai-visualizer-${visualizer_hash}.tar.xz" # File
        "${_download_directory_url}/${visualizer_hash}/depthai-visualizer-${visualizer_hash}.tar.xz.sha256" # File checksum
        "${folder}/depthai-visualizer-${visualizer_hash}.tar.xz"
        status
    )
    if(${status})
        message(STATUS "\nCouldn't download depthai-visualizer static files\n")
        PrintErrorMessage(${status})
        message(FATAL_ERROR "Aborting.\n")
    endif()
    # add depthai-device-kb-fwp.tar.xz to list
    list(APPEND "${output_list_var}" "${folder}/depthai-visualizer-${visualizer_hash}.tar.xz")


    # Set list of files as output
    set("${output_list_var}" "${${output_list_var}}" PARENT_SCOPE)

endfunction()
