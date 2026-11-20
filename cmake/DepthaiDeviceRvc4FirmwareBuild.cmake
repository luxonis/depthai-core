# Builds the RVC4 firmware package from device/rvc4 of this checkout and embeds it in the library. The host build uses
# this when cmake/Depthai/DepthaiDeviceRVC4Config.cmake sets no DEPTHAI_DEVICE_RVC4_VERSION (the default) and
# DEPTHAI_DEVICE_RVC4_FWP_PATH is not set. A version downloads that package from the artifact server instead.
#
# Each build of the library runs device/rvc4/create_deployment_artifacts.sh in its build container (Docker): the
# target rvc4, or rvc4-tsan / rvc4-asan-ubsan for a sanitizer build of the library. The firmware build is incremental,
# and the package and its license bundle are copied to the resources only when they changed, so an unchanged firmware
# does not compile the library again. The environment variables of create_deployment_artifacts.sh apply (for example
# DEPTHAI_ACCEPT_QUALCOMM_LICENSE, DEPTHAI_ACCEPT_VENDOR_EULA, DEPTHAI_BUILD_JOBS, DEPTHAI_DEVICE_RVC4_VENDOR).
# The cache entries DEPTHAI_ACCEPT_VENDOR_EULA, DEPTHAI_ACCEPT_QUALCOMM_LICENSE, DEPTHAI_DEVICE_SDK_IMAGE,
# DEPTHAI_DEVICE_RVC4_VENDOR and DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR of the library (-D, or a preset of
# CMakePresets.json or CMakeUserPresets.json) are given to the script as these environment variables; empty, the
# environment applies.
#
# Each configure step of the library also configures the firmware build in its build container
# (DEPTHAI_DEVICE_CONFIGURE_ONLY=1 of create_deployment_artifacts.sh): an error of the firmware configuration, for
# example a license agreement that is not accepted, stops the configure step and not the build. The step also writes
# device/rvc4/build_docker_arm64_<target>/compile_commands.json. -D DEPTHAI_DEVICE_RVC4_FWP_CONFIGURE=OFF leaves the
# firmware configuration to the build.
#
# The build stops when the license bundle of the package is not the committed notices/depthai-device-RVC4-LICENSE, as a
# download stops (cmake/VerifyLicense.cmake): the wheel and the install ship that notice file. After an intended change
# of the bundle (third-party components, the legal files of the vendor module), copy the bundle over the notice file.
#
# The version string of the embedded package is "<firmware version>+local.<first 12 hex digits of the commit>".
# At run time, the environment variable DEPTHAI_DEVICE_RVC4_FWP still selects another package.
#
# DepthaiDeviceRvc4FirmwareBuild(<output folder> <version variable> <resource list variable> <target variable>)
#   <version variable>        out: version of the embedded package
#   <resource list variable>  out: files to embed
#   <target variable>         out: target that builds the package; the resource library depends on it
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
    if(NOT EXISTS "${_source_dir}/create_deployment_artifacts.sh" OR NOT DEPTHAI_DEVICE_RVC4_BASH OR NOT DEPTHAI_DEVICE_RVC4_DOCKER)
        message(FATAL_ERROR "No DEPTHAI_DEVICE_RVC4_VERSION is set, so the build builds the RVC4 firmware from device/rvc4, "
                            "and that needs bash and Docker (found bash: '${DEPTHAI_DEVICE_RVC4_BASH}', docker: "
                            "'${DEPTHAI_DEVICE_RVC4_DOCKER}'). Either\n"
                            "  * install them (device/rvc4/README.md, Building), or\n"
                            "  * download a firmware package: -D DEPTHAI_DEVICE_RVC4_VERSION=<version>, or\n"
                            "  * embed a package that you have: -D DEPTHAI_DEVICE_RVC4_FWP_PATH=<path>, or\n"
                            "  * build without the RVC4 firmware: -D DEPTHAI_ENABLE_DEVICE_RVC4_FW=OFF.")
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
