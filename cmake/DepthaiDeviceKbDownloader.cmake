# This script downloads depthai device side artifacts for different device types
include(DownloadAndChecksum)
function(DepthaiDeviceDownloader)

    ### VARIABLES
    # Artifactory
    set(DOWNLOADER_BASE_URL "https://artifacts.luxonis.com/artifactory")

    
    # Errors and retry count
    set(DOWNLOADER_TIMEOUT_S 300)
    set(DOWNLOADER_INACTIVE_TIMEOUT_S 60)
    set(DOWNLOADER_RETRY_NUM 5)
    ### END VARIABLES
    
    
    # PARSE ARGUMENTS
    set(device_type ${ARGV0})
    set(repo_snapshot ${ARGV1})
    set(repo_release ${ARGV2})
    set(_depthai_shared_commit ${ARGV3})
    set(_enforce_depthai_shared_commit ${ARGV4})
    set(folder "${ARGV5}")
    set(output_list_var "${ARGV6}")
    set(maturity "${ARGV7}")
    set(commit_version_arg "${ARGV8}")
    # END PARSE ARGUMENTS
    
    message(STATUS "Downloading ${device_type}")

    # Repositories (dynamic based on parameters)
    set(DOWNLOADER_REPO_SNAPSHOT "${repo_snapshot}")
    set(DOWNLOADER_REPO_RELEASE "${repo_release}")

    # Prefix (dynamic based on device_type)
    set(DOWNLOADER_ARTIFACT_PREFIX "${device_type}")


    #DEBUG
    message(STATUS "folder: ${folder}")
    message(STATUS "maturity: ${maturity}")
    message(STATUS "commit_version_arg: ${commit_version_arg}")

    string(TOLOWER "${maturity}" maturity_lower)

    # Switch between maturity
    if("${maturity_lower}" STREQUAL "snapshot")
        set(_selected_repo "${DOWNLOADER_REPO_SNAPSHOT}")
        set(commit ${commit_version_arg})
        string(REPLACE "-asan" "" commit "${commit}")
        string(REPLACE "-ubsan" "" commit "${commit}")
        string(REPLACE "-tsan" "" commit "${commit}")

        # Create download directory string
        string(CONFIGURE "@DOWNLOADER_BASE_URL@/@DOWNLOADER_REPO_SNAPSHOT@/@DOWNLOADER_ARTIFACT_PREFIX@/@commit@" _download_directory_url)

        # Create _version_commit_identifier
        set(_version_commit_identifier "${commit_version_arg}")

    elseif("${maturity_lower}" STREQUAL "release")
        set(_selected_repo "${DOWNLOADER_REPO_RELEASE}")
        set(version ${commit_version_arg})

        # Create download directory string
        string(CONFIGURE "@DOWNLOADER_BASE_URL@/@DOWNLOADER_REPO_RELEASE@/@DOWNLOADER_ARTIFACT_PREFIX@/@version@" _download_directory_url)

        # Create _version_commit_identifier
        set(_version_commit_identifier "${version}")
    else()
        # Not a recognized maturity level
        message(FATAL_ERROR "Cannot download ${device_type} binaries. Maturity level not recognized (${maturity_lower})")
        return()
    endif()

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

    # Check if shared commit matches
    if(_depthai_shared_commit)
        message(STATUS "${device_type} shared commit: ${_depthai_shared_commit}")
        DownloadAndChecksum(
            "${_download_directory_url}/depthai-shared-commit-hash-${_version_commit_identifier}.txt" # File
            "${_download_directory_url}/depthai-shared-commit-hash-${_version_commit_identifier}.txt.sha256" # File checksum
            "${folder}/depthai-shared-commit-hash-${_version_commit_identifier}.txt"
            status
        )
        if(${status})

            message(STATUS "Couldn't check if depthai-device-kb-shared codebase matches between device and host")
            if(${_enforce_depthai_shared_commit})
                message(FATAL_ERROR "Aborting.\n")
            endif()

        else()

            set(_message_mode WARNING)
            if(${_enforce_depthai_shared_commit})
                set(_message_mode FATAL_ERROR)
            endif()

            # Read commit hash file
            file(READ "${folder}/depthai-shared-commit-hash-${_version_commit_identifier}.txt" _device_depthai_shared_commit_hash)
            string(REGEX REPLACE "\n$" "" _device_depthai_shared_commit_hash "${_device_depthai_shared_commit_hash}")
            string(REGEX REPLACE "\n$" "" _depthai_shared_commit "${_depthai_shared_commit}")
            string(COMPARE EQUAL "${_device_depthai_shared_commit_hash}" "${_depthai_shared_commit}" _is_same)

            # If commits dont match
            if(NOT ${_is_same})
                message(${_message_mode} "${device_type}-shared codebases differ between device and host. Enforce (CI): ${_enforce_depthai_shared_commit} (device: ${_device_depthai_shared_commit_hash}, host: ${_depthai_shared_commit}")
            else()
                message(STATUS "${device_type}-shared between device and host MATCH!. (device: ${_device_depthai_shared_commit_hash}, host: ${_depthai_shared_commit}")
            endif()

        endif()

    endif()

    # Download firmware package
    message(STATUS "Downloading and checking ${device_type}-fwp.tar.xz")
    message(STATUS "Download URL: ${_download_directory_url}/${device_type}-fwp-${_version_commit_identifier}.tar.xz")
    DownloadAndChecksum(
        "${_download_directory_url}/${device_type}-fwp-${_version_commit_identifier}.tar.xz" # File
        "${_download_directory_url}/${device_type}-fwp-${_version_commit_identifier}.tar.xz.sha256" # File checksum
        "${folder}/${device_type}-fwp-${_version_commit_identifier}.tar.xz"
        status
    )
    if(${status})
        message(STATUS "\nCouldn't download ${device_type}-fwp.tar.xz\n")
        PrintErrorMessage(${status})
        message(FATAL_ERROR "Aborting.\n")
    endif()
    # add depthai-device-kb-fwp.tar.xz to list
    list(APPEND "${output_list_var}" "${folder}/${device_type}-fwp-${_version_commit_identifier}.tar.xz")


    # Set list of files as output
    set("${output_list_var}" "${${output_list_var}}" PARENT_SCOPE)

endfunction()
