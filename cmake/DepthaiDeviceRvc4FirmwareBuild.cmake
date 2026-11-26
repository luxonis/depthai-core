# Builds the RVC4 firmware package from device/rvc4 (in Docker) and embeds it in the library.
# Used when DEPTHAI_DEVICE_RVC4_VERSION and DEPTHAI_DEVICE_RVC4_FWP_PATH are not set.
#
# - Configure step: configures the firmware build too (-D DEPTHAI_DEVICE_RVC4_FWP_CONFIGURE=OFF skips it).
# - Build step: incremental; the package is copied only when it changed.
# - The cache entries below go to create_deployment_artifacts.sh as environment variables.
# - The license bundle must be notices/depthai-device-RVC4-LICENSE. After an intended change, copy it over.
# - Version: "<firmware version>+local.<12-digit commit>".
#
# DepthaiDeviceRvc4FirmwareBuild(<output folder> <version var> <resource list var> <target var>)
#   out: package version, files to embed, target that builds the package

function(DepthaiDeviceRvc4FirmwareBuild output_dir version_var resource_list_var target_var)
    set(_source_dir "${CMAKE_SOURCE_DIR}/device/rvc4")
    if(DEPTHAI_SANITIZE AND SANITIZE_THREAD)
        set(_target "rvc4-tsan")
    elseif(DEPTHAI_SANITIZE)
        set(_target "rvc4-asan-ubsan")
    else()
        set(_target "rvc4")
    endif()

    find_program(DEPTHAI_DEVICE_RVC4_BASH bash)
    find_program(DEPTHAI_DEVICE_RVC4_DOCKER docker)
    string(CONCAT _alternatives
           "  * download a firmware package: -D DEPTHAI_DEVICE_RVC4_VERSION=<version>, or\n"
           "  * embed a package that you have: -D DEPTHAI_DEVICE_RVC4_FWP_PATH=<path>, or\n"
           "  * build without the RVC4 firmware: -D DEPTHAI_ENABLE_DEVICE_RVC4_FW=OFF.")
    if(NOT EXISTS "${_source_dir}/create_deployment_artifacts.sh" OR NOT DEPTHAI_DEVICE_RVC4_BASH OR NOT DEPTHAI_DEVICE_RVC4_DOCKER)
        message(FATAL_ERROR "No DEPTHAI_DEVICE_RVC4_VERSION is set, so the build builds the RVC4 firmware from device/rvc4, "
                            "and that needs bash and Docker (found bash: '${DEPTHAI_DEVICE_RVC4_BASH}', docker: "
                            "'${DEPTHAI_DEVICE_RVC4_DOCKER}'). Either\n"
                            "  * install them (device/rvc4/README.md, Building), or\n"
                            "${_alternatives}")
    endif()
    # The docker command is only the client (for example the docker formula of Homebrew): the build also needs a running
    # Docker engine
    execute_process(COMMAND "${DEPTHAI_DEVICE_RVC4_DOCKER}" info --format "{{.ServerVersion}}"
                    RESULT_VARIABLE _docker_result OUTPUT_QUIET ERROR_VARIABLE _docker_error
                    ERROR_STRIP_TRAILING_WHITESPACE TIMEOUT 60)
    if(NOT _docker_result EQUAL 0)
        message(FATAL_ERROR "No DEPTHAI_DEVICE_RVC4_VERSION is set, so the build builds the RVC4 firmware from device/rvc4 "
                            "in a Docker container, and the Docker engine does not answer: '${DEPTHAI_DEVICE_RVC4_DOCKER} "
                            "info' failed (${_docker_result}): ${_docker_error}. Either\n"
                            "  * start the Docker engine (device/rvc4/README.md, Building), or\n"
                            "${_alternatives}")
    endif()

    file(STRINGS "${_source_dir}/version.txt" _firmware_version LIMIT_COUNT 1)
    string(STRIP "${_firmware_version}" _firmware_version)
    find_package(Git QUIET)
    set(_commit "unknown")
    if(GIT_FOUND)
        execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short=12 HEAD WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                        OUTPUT_VARIABLE _git_commit OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _git_result ERROR_QUIET)
        if(_git_result EQUAL 0 AND _git_commit)
            set(_commit "${_git_commit}")
        endif()
    endif()
    set(_version "${_firmware_version}+local.${_commit}")

    option(DEPTHAI_DEVICE_RVC4_FWP_CONFIGURE "Configure the RVC4 firmware build (device/rvc4) in its build container at each configure step" ON)
    set(DEPTHAI_ACCEPT_VENDOR_EULA "" CACHE STRING
        "yes: accept the Luxonis DepthAI Vendor Module End User License Agreement for the firmware build from device/rvc4 (device/rvc4/README.md, Building). Empty: the environment variable applies")
    set(DEPTHAI_ACCEPT_QUALCOMM_LICENSE "" CACHE STRING
        "yes: accept the license terms of the Qualcomm AI Runtime SDK for the first firmware build with a public image (device/rvc4/README.md). Empty: the environment variable applies")
    set(DEPTHAI_DEVICE_SDK_IMAGE "" CACHE STRING
        "Build image of the RVC4 firmware build. Empty: the environment variable applies, else the pinned public image (DEPTHAI_DEVICE_RVC4_SDK_IMAGE)")
    set(DEPTHAI_DEVICE_RVC4_VENDOR "" CACHE FILEPATH
        "Local prebuilt vendor archive for the RVC4 firmware build. Empty: the environment variable applies, else the download of DEPTHAI_DEVICE_RVC4_VENDOR_VERSION")
    set(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR "" CACHE PATH
        "Sources of the vendor module for the RVC4 firmware build (Luxonis licensees). Empty: the environment variable applies, else the prebuilt archive")
    set(_script_env "")
    foreach(_setting DEPTHAI_ACCEPT_VENDOR_EULA DEPTHAI_ACCEPT_QUALCOMM_LICENSE DEPTHAI_DEVICE_SDK_IMAGE
                     DEPTHAI_DEVICE_RVC4_VENDOR DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR)
        if(NOT "${${_setting}}" STREQUAL "")
            list(APPEND _script_env "${_setting}=${${_setting}}")
        endif()
    endforeach()

    if(DEPTHAI_DEVICE_RVC4_FWP_CONFIGURE)
        message(STATUS "RVC4 firmware: configuring device/rvc4 (target ${_target}) in its build container")
        execute_process(
            COMMAND "${CMAKE_COMMAND}" -E env ${_script_env} DEPTHAI_DEVICE_CONFIGURE_ONLY=1
                    "${DEPTHAI_DEVICE_RVC4_BASH}" "${_source_dir}/create_deployment_artifacts.sh" "${_target}"
            WORKING_DIRECTORY "${_source_dir}"
            RESULT_VARIABLE _configure_result)
        if(NOT _configure_result EQUAL 0)
            message(FATAL_ERROR "The configure step of the RVC4 firmware build (device/rvc4, target ${_target}) failed "
                                "(${_configure_result}); its output is above. The cache entries DEPTHAI_ACCEPT_VENDOR_EULA, "
                                "DEPTHAI_ACCEPT_QUALCOMM_LICENSE, DEPTHAI_DEVICE_SDK_IMAGE, DEPTHAI_DEVICE_RVC4_VENDOR and "
                                "DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR of the library are given to it as environment "
                                "variables. -D DEPTHAI_DEVICE_RVC4_FWP_CONFIGURE=OFF leaves the firmware configuration to "
                                "the build.")
        endif()
    endif()

    set(_package_dir "${_source_dir}/build_docker_arm64_${_target}/RelWithDebInfo")
    set(_embedded_fwp "${output_dir}/depthai-device-rvc4-fwp-${_version}.tar.xz")
    set(_embedded_license "${output_dir}/depthai-device-rvc4-fwp-${_version}-LICENSE")
    set(_committed_notice "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../notices/depthai-device-RVC4-LICENSE")
    add_custom_target(depthai-device-rvc4-fwp ALL
        COMMAND "${CMAKE_COMMAND}" -E env --unset=DEPTHAI_DEVICE_CONFIGURE_ONLY ${_script_env}
                "${DEPTHAI_DEVICE_RVC4_BASH}" "${_source_dir}/create_deployment_artifacts.sh" "${_target}"
        # The license bundle must be the committed notice file, which the wheel and the install ship
        COMMAND "${CMAKE_COMMAND}" -D "LICENSE_FILE=${_package_dir}/depthai-device-rvc4-fwp-LICENSE"
                -D "EXPECTED=${_committed_notice}" -D ORIGIN=Built -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/VerifyLicense.cmake"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_package_dir}/depthai-device-rvc4-fwp.tar.xz" "${_embedded_fwp}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_package_dir}/depthai-device-rvc4-fwp-LICENSE" "${_embedded_license}"
        BYPRODUCTS "${_embedded_fwp}" "${_embedded_license}"
        WORKING_DIRECTORY "${_source_dir}"
        COMMENT "Building the RVC4 firmware package from device/rvc4 (target ${_target})"
        USES_TERMINAL
        VERBATIM)

    message(STATUS "RVC4 firmware: built from device/rvc4 (target ${_target}) at each build, version ${_version}")
    set(${version_var} "${_version}" PARENT_SCOPE)
    set(${resource_list_var} "${_embedded_fwp};${_embedded_license}" PARENT_SCOPE)
    set(${target_var} depthai-device-rvc4-fwp PARENT_SCOPE)
endfunction()
