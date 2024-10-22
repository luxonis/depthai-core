# This script downloads depthai device side artifacts
include(DownloadAndChecksum)
function(DepthaiDownload)

    ### VARIABLES
    # Artifactory 
    set(DEPTHAI_BASE_URL "https://artifacts.luxonis.com/artifactory")

    # Repositories
    set(DEPTHAI_REPO_SNAPSHOT "luxonis-myriad-snapshot-local")
    set(DEPTHAI_REPO_RELEASE "luxonis-myriad-release-local")

    # Prefix
    set(DEPTHAI_ARTIFACT_PREFIX "depthai-device-side")

    set(DOWNLOADER_TIMEOUT_S 300)
    set(DOWNLOADER_INACTIVE_TIMEOUT_S 60)
    set(DOWNLOADER_RETRY_NUM 5)


    # PARSE ARGUMENTS
    set(_download_patch_only OFF)

    #if first argument PATCH_ONLY, second must be either ON/OFF
    if("${ARGV2}" STREQUAL "PATCH_ONLY")
        set(_depthai_shared_commit ${ARGV0})
        set(_enforce_depthai_shared_commit ${ARGV1})
            
        if( ${ARGV3} )
            set(_download_patch_only ON)
        else()
            set(_download_patch_only OFF)
        endif()

        set(folder "${ARGV4}")
        set(output_list_var "${ARGV5}")
        set(maturity "${ARGV6}")
        set(commit "${ARGV7}")
        set(version "${ARGV8}") #optional

    else()
        
        set(folder "${ARGV0}")
        set(output_list_var "${ARGV1}")
        set(maturity "${ARGV2}")
        set(commit "${ARGV3}")
        set(version "${ARGV4}") #optional

    endif("${ARGV2}" STREQUAL "PATCH_ONLY")

    if(_download_patch_only)
        message(STATUS "Downloading depthai and patch")
    else()
        message(STATUS "Downloading depthai, depthai-usb2 and patch")
    endif()

    # END PARSE ARGUMENTS

    #DEBUG
    #message(STATUS "folder ${folder}")
    #message(STATUS "maturity ${maturity}")
    #message(STATUS "commit ${commit}")
    #message(STATUS "version ${version}") #optional

    string(TOLOWER "${maturity}" maturity_lower)

    # Switch between maturity
    if(${maturity_lower} STREQUAL "snapshot")
        set(_selected_repo "${DEPTHAI_REPO_SNAPSHOT}")

        # Create download directory string
        string(CONFIGURE "@DEPTHAI_BASE_URL@/@DEPTHAI_REPO_SNAPSHOT@/@DEPTHAI_ARTIFACT_PREFIX@/@commit@" _download_directory_url)

        # Create _version_commit_identifier
        set(_version_commit_identifier "${commit}")

    elseif(${maturity_lower} STREQUAL "release")
        set(_selected_repo "${DEPTHAI_REPO_RELEASE}")

        # TODO
        # Create download directory string
        #string(CONFIGURE "@DEPTHAI_BASE_URL@/@DEPTHAI_REPO_SNAPSHOT@/@DEPTHAI_ARTIFACT_PREFIX@/@_commit_hash@" _download_directory_url)
        
    else()
        # Not a recognized maturity level
        message(FATAL_ERROR "Cannot download DepthAI Device Side binaries. Maturity level not recognized (${maturity_lower})")
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

    # Check if depthai-shared matches
    message(STATUS "commit: ${_depthai_shared_commit}")
    if(_depthai_shared_commit)
        DownloadAndChecksum(
            "${_download_directory_url}/depthai-shared-commit-hash-${_version_commit_identifier}.txt" # File
            "${_download_directory_url}/depthai-shared-commit-hash-${_version_commit_identifier}.sha256.checksum" # File checksum
            "${folder}/depthai-shared-commit-hash-${_version_commit_identifier}.txt"
            status
        )
        if(${status})
            message(STATUS "Couldn't check if depthai-shared codebase matches between device and host")
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
                message(${_message_mode} "depthai-shared codebases differ between device and host. Enforce (CI): ${_enforce_depthai_shared_commit} (device: ${_device_depthai_shared_commit_hash}, host: ${_depthai_shared_commit}")
            else()
                message(STATUS "depthai-shared between device and host MATCH!. (device: ${_device_depthai_shared_commit_hash}, host: ${_depthai_shared_commit}")
            endif()
            
        endif()


    endif()


    # Download depthai-device firmware package
    message(STATUS "Downloading and checking depthai-device-fwp.tar.xz")
    DownloadAndChecksum(
        "${_download_directory_url}/depthai-device-fwp-${_version_commit_identifier}.tar.xz" # File
        "${_download_directory_url}/depthai-device-fwp-${_version_commit_identifier}.tar.xz.sha256" # File checksum
        "${folder}/depthai-device-fwp-${_version_commit_identifier}.tar.xz"
        status
    )
    if(${status})
        message(STATUS "\nCouldn't download depthai-device-fwp.tar.xz\n")
        PrintErrorMessage(${status})
        message(FATAL_ERROR "Aborting.\n")
    endif()
    # add depthai-device-fwp.tar.xz to list
    list(APPEND "${output_list_var}" "${folder}/depthai-device-fwp-${_version_commit_identifier}.tar.xz")
    


    ## # depthai.cmd
    ## message(STATUS "Downloading and checking depthai.cmd")
    ## DownloadAndChecksum(
    ##     "${_download_directory_url}/depthai-${_version_commit_identifier}.cmd" # File
    ##     "${_download_directory_url}/depthai-${_version_commit_identifier}.sha256.checksum" # File checksum
    ##     "${folder}/depthai-${_version_commit_identifier}.cmd"
    ##     status
    ## )
    ## if(${status})
    ##     message(STATUS "\nCouldn't download depthai.cmd\n")
    ##     PrintErrorMessage(${status})
    ##     message(FATAL_ERROR "Aborting.\n")
    ## endif()
    ## # add depthai.cmd to list
    ## list(APPEND "${output_list_var}" "${folder}/depthai-${_version_commit_identifier}.cmd")
    ## 

    ## if(NOT _download_patch_only)
    ##     # depthai-usb2.cmd
    ##     message(STATUS "Downloading and checking depthai-usb2.cmd")
    ##     DownloadAndChecksum(
    ##         "${_download_directory_url}/depthai-usb2-${_version_commit_identifier}.cmd" # File
    ##         "${_download_directory_url}/depthai-usb2-${_version_commit_identifier}.sha256.checksum" # File checksum
    ##         "${folder}/depthai-usb2-${_version_commit_identifier}.cmd"
    ##         status
    ##     )

    ##     if(${status})
    ##         message(STATUS "\nCouldn't download depthai-usb2.cmd.\n")
    ##         PrintErrorMessage(${status})
    ##         message(FATAL_ERROR "Aborting.\n")
    ##     endif()

    ##     # add depthai-usb2.cmd to list
    ##     list(APPEND "${output_list_var}" "${folder}/depthai-usb2-${_version_commit_identifier}.cmd")

    ## endif(NOT _download_patch_only)

    ## # depthai-usb2-patch.patch
    ## message(STATUS "Downloading and checking depthai-usb2-patch.patch")
    ## DownloadAndChecksum(
    ##     "${_download_directory_url}/depthai-usb2-patch-${_version_commit_identifier}.patch" # File
    ##     "${_download_directory_url}/depthai-usb2-patch-${_version_commit_identifier}.sha256.checksum" # File checksum
    ##     "${folder}/depthai-usb2-patch-${_version_commit_identifier}.patch"
    ##     status
    ## )
    ## if(${status})
    ##     message(STATUS "\nCouldn't download depthai-usb2-patch.patch.\n")
    ##     PrintErrorMessage(${status})
    ##     message(FATAL_ERROR "Aborting.\n")
    ## endif()    

    ## # add depthai-usb2.cmd to list
    ## list(APPEND "${output_list_var}" "${folder}/depthai-usb2-patch-${_version_commit_identifier}.patch")
    
    
    # Propagate the variable to parent
    set("${output_list_var}" "${${output_list_var}}" PARENT_SCOPE)

endfunction()

function(DepthaiLocal patch_only patch_only_on_off output_dir output_list_var path1 path2 path3)

    #debug
    message(STATUS "patch only: ${patch_only}\non/off${patch_only_on_off}\noutput dir: ${output_dir}\nout_var: ${output_list_var}\npath1: ${path1}\npath2: ${path2}\npath3: ${path3}")

    # If patch only
    if(patch_only_on_off)
        if(NOT path1 OR NOT path3)
            message(FATAL_ERROR "Both depthai binary and depthai usb2 patch files must be specified")
        else()
            configure_file("${path1}" "${output_dir}/depthai.cmd" COPYONLY)
            configure_file("${path3}" "${output_dir}/depthai-usb2-patch.patch" COPYONLY)
            
            # Set output list
            list(APPEND _resource_list "${output_dir}/depthai.cmd" "${output_dir}/depthai-usb2-patch.patch")

        endif()
    else()
        if(NOT path1 OR NOT path2)
            message(FATAL_ERROR "Both depthai and depthai usb2 binary files must be specified")
        else()
            configure_file("${path1}" "${output_dir}/depthai.cmd" COPYONLY)
            configure_file("${path2}" "${output_dir}/depthai-usb2.cmd" COPYONLY)

            # Set output list
            list(APPEND _resource_list "${output_dir}/depthai.cmd" "${output_dir}/depthai-usb2.cmd")

        endif()
    endif()

    set("${output_list_var}" "${_resource_list}" PARENT_SCOPE)

endfunction()