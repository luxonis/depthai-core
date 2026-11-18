# Builds the RVC4 firmware package from device/rvc4 of this checkout and embeds it in the library. The host build uses
# this when cmake/Depthai/DepthaiDeviceRVC4Config.cmake sets no DEPTHAI_DEVICE_RVC4_VERSION (the default) and
# DEPTHAI_DEVICE_RVC4_FWP_PATH is not set. A version downloads that package from the artifact server instead.
#
# Each build of the library runs device/rvc4/create_deployment_artifacts.sh in its build container (Docker): the
# target rvc4, or rvc4-tsan / rvc4-asan-ubsan for a sanitizer build of the library. The firmware build is incremental,
# and the package and its license bundle are copied to the resources only when they changed, so an unchanged firmware
# does not compile the library again. The environment variables of create_deployment_artifacts.sh apply (for example
# DEPTHAI_ACCEPT_QUALCOMM_LICENSE, DEPTHAI_BUILD_JOBS, DEPTHAI_DEVICE_RVC4_VENDOR).
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

    set(_package_dir "${_source_dir}/build_docker_arm64_${_target}/RelWithDebInfo")
    set(_embedded_fwp "${output_dir}/depthai-device-rvc4-fwp-${_version}.tar.xz")
    set(_embedded_license "${output_dir}/depthai-device-rvc4-fwp-${_version}-LICENSE")
    add_custom_target(depthai-device-rvc4-fwp ALL
        COMMAND "${DEPTHAI_DEVICE_RVC4_BASH}" "${_source_dir}/create_deployment_artifacts.sh" "${_target}"
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
