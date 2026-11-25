# This script downloads depthai device side model artifacts.
#
# The models are served by Artifact Keeper and read anonymously. A token is only
# needed for a repository that is not public. Supply one as the CMake variable
# DEPTHAI_ARTIFACTORY_TOKEN or as the environment variable AK_TOKEN.
#
# Models reached through the DeviceModelZoo table are XOR'd on read
# (see NeuralNetworkRvc4.cpp, xor_file_with_seed). Upload the XOR'd file.

include_guard(GLOBAL)

### VARIABLES
set(_DEPTHAI_MODELS_KB_BASE_URL "https://artifacts.luxonis.com/api/v1/repositories")

# Repositories
set(_DEPTHAI_MODELS_KB_REPO_SNAPSHOT "luxonis-rvc4-snapshot-local")
set(_DEPTHAI_MODELS_KB_REPO_RELEASE "luxonis-rvc4-release-local")

# Errors and retry count
set(_DEPTHAI_MODELS_KB_TIMEOUT_S 300)
set(_DEPTHAI_MODELS_KB_INACTIVITY_TIMEOUT_S 60)
set(_DEPTHAI_MODELS_KB_RETRY_NUM 5)
### END VARIABLES


# Prints an explanation for a file(DOWNLOAD) status code.
function(_DepthaiModelsKbPrintError status)
    if(status EQUAL 22)
        message(STATUS "HTTP error. Check the path, and the token if the repository is not public.")
    elseif(status EQUAL 28)
        message(STATUS "Timeout.")
    elseif(status EQUAL 99)
        message(STATUS "Checksum mismatch.")
    else()
        message(STATUS "Unknown error.")
    endif()
endfunction()


# Returns the Artifactory token in out_var, or an empty string.
# Never print this value. It leaks into the build log.
function(_DepthaiModelsKbToken out_var)
    if(DEPTHAI_ARTIFACTORY_TOKEN)
        set("${out_var}" "${DEPTHAI_ARTIFACTORY_TOKEN}" PARENT_SCOPE)
    else()
        set("${out_var}" "$ENV{AK_TOKEN}" PARENT_SCOPE)
    endif()
endfunction()


# Downloads one URL to output. Removes the output file if the download fails,
# so that a broken body is never mistaken for a cached artifact.
function(_DepthaiModelsKbFetch url output token status_var)
    set(_header_args "")
    if(NOT "${token}" STREQUAL "")
        # Use "X-JFrog-Art-Api: ${token}" instead if you use a legacy API key.
        set(_header_args HTTPHEADER "Authorization: Bearer ${token}")
    endif()

    file(DOWNLOAD "${url}" "${output}"
        ${_header_args}
        STATUS _status
        TIMEOUT ${_DEPTHAI_MODELS_KB_TIMEOUT_S}
        INACTIVITY_TIMEOUT ${_DEPTHAI_MODELS_KB_INACTIVITY_TIMEOUT_S}
    )

    list(GET _status 0 _status_num)
    if(NOT _status_num EQUAL 0)
        file(REMOVE "${output}")
        message(STATUS "Status error: ${_status} (${url})")
    endif()
    set("${status_var}" "${_status_num}" PARENT_SCOPE)
endfunction()


# Downloads one model and appends its path to the caller's output list.
#
# DepthaiModelsKbDownload(
#     FOLDER      <folder that receives the file>
#     OUTPUT_LIST <name of the list variable to append to>
#     MATURITY    <"snapshot" or "release">
#     VERSION     <version or commit directory, may be empty>
#     PATH_PREFIX <path under the repository, without a leading slash>
#     NAME        <file name of the model>
#     [SHA256     <expected checksum, pins the artifact and allows offline builds>]
# )
#
# The download URL becomes:
#     <base>/<repo>/download/<PATH_PREFIX>/<VERSION>/<NAME>
# and the checksum URL is that URL plus ".sha256".
function(DepthaiModelsKbDownload)

    cmake_parse_arguments(ARG "" "FOLDER;OUTPUT_LIST;MATURITY;VERSION;PATH_PREFIX;NAME;SHA256" "" ${ARGN})

    foreach(_required FOLDER OUTPUT_LIST MATURITY PATH_PREFIX NAME)
        if(NOT ARG_${_required})
            message(FATAL_ERROR "DepthaiModelsKbDownload: ${_required} is required")
        endif()
    endforeach()
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "DepthaiModelsKbDownload: unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    string(TOLOWER "${ARG_MATURITY}" _maturity)
    if("${_maturity}" STREQUAL "snapshot")
        set(_repo "${_DEPTHAI_MODELS_KB_REPO_SNAPSHOT}")
    elseif("${_maturity}" STREQUAL "release")
        set(_repo "${_DEPTHAI_MODELS_KB_REPO_RELEASE}")
    else()
        message(FATAL_ERROR "Cannot download depthai models. Maturity level not recognized (${_maturity})")
    endif()

    # Build the directory URL. An empty VERSION adds no directory level.
    set(_directory_url "${_DEPTHAI_MODELS_KB_BASE_URL}/${_repo}/download/${ARG_PATH_PREFIX}")
    if(NOT "${ARG_VERSION}" STREQUAL "")
        set(_directory_url "${_directory_url}/${ARG_VERSION}")
    endif()

    set(_url "${_directory_url}/${ARG_NAME}")
    set(_url_checksum "${_url}.sha256")
    set(_output "${ARG_FOLDER}/${ARG_NAME}")

    string(TOLOWER "${ARG_SHA256}" _expected)

    # A cached file with the expected checksum needs no network access.
    if(EXISTS "${_output}" AND NOT "${_expected}" STREQUAL "")
        file(SHA256 "${_output}" _cached_checksum)
        string(TOLOWER "${_cached_checksum}" _cached_checksum)
        if("${_cached_checksum}" STREQUAL "${_expected}")
            message(STATUS "Model already downloaded: ${ARG_NAME}")
            list(APPEND "${ARG_OUTPUT_LIST}" "${_output}")
            set("${ARG_OUTPUT_LIST}" "${${ARG_OUTPUT_LIST}}" PARENT_SCOPE)
            return()
        endif()
        message(STATUS "Cached model does not match the expected checksum, downloading again: ${ARG_NAME}")
    endif()

    # Public repositories need no token. A token is sent only when one is set.
    _DepthaiModelsKbToken(_token)

    message(STATUS "Downloading and checking ${ARG_NAME}")
    file(MAKE_DIRECTORY "${ARG_FOLDER}")

    set(_retries_left ${_DEPTHAI_MODELS_KB_RETRY_NUM})
    set(_status 1)
    while(NOT _retries_left EQUAL 0)
        math(EXPR _retries_left "${_retries_left} - 1")

        # Download the checksum first. Read it only after the status is good,
        # because file(READ) on a missing file aborts the configure step.
        _DepthaiModelsKbFetch("${_url_checksum}" "${_output}.checksum" "${_token}" _status)
        if(NOT _status EQUAL 0)
            continue()
        endif()
        file(READ "${_output}.checksum" _remote_checksum)
        file(REMOVE "${_output}.checksum")
        string(STRIP "${_remote_checksum}" _remote_checksum)
        string(TOLOWER "${_remote_checksum}" _remote_checksum)

        # A pinned checksum must agree with the published one. If it does not,
        # the artifact changed under a name that is supposed to be stable.
        if(NOT "${_expected}" STREQUAL "" AND NOT "${_remote_checksum}" STREQUAL "${_expected}")
            message(FATAL_ERROR
                "Published checksum of ${ARG_NAME} does not match the pinned value.\n"
                "  published: ${_remote_checksum}\n"
                "  pinned:    ${_expected}\n"
                "Bump the version, or correct the pinned checksum.")
        endif()

        _DepthaiModelsKbFetch("${_url}" "${_output}" "${_token}" _status)
        if(NOT _status EQUAL 0)
            continue()
        endif()

        file(SHA256 "${_output}" _downloaded_checksum)
        string(TOLOWER "${_downloaded_checksum}" _downloaded_checksum)
        if(NOT "${_downloaded_checksum}" STREQUAL "${_remote_checksum}")
            message(STATUS "Downloaded file checksum mismatch: ${_downloaded_checksum} != ${_remote_checksum}")
            file(REMOVE "${_output}")
            set(_status 99)
            continue()
        endif()

        set(_status 0)
        break()
    endwhile()

    if(NOT _status EQUAL 0)
        message(STATUS "Couldn't download ${ARG_NAME}")
        _DepthaiModelsKbPrintError(${_status})
        message(FATAL_ERROR "Aborting.")
    endif()

    list(APPEND "${ARG_OUTPUT_LIST}" "${_output}")
    set("${ARG_OUTPUT_LIST}" "${${ARG_OUTPUT_LIST}}" PARENT_SCOPE)

endfunction()
