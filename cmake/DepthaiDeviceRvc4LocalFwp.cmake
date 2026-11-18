# Embeds a local RVC4 firmware package in the library, in place of the firmware that the build builds from
# device/rvc4 or downloads (DEPTHAI_DEVICE_RVC4_VERSION):
#
#   cmake -S . -B build -D DEPTHAI_DEVICE_RVC4_FWP_PATH=<path>/depthai-device-rvc4-fwp.tar.xz
#
# for example a package that you built from device/rvc4. Nothing is downloaded for the RVC4 firmware.
#
# The version string of the embedded package is "<base version>+local.<first 12 hex digits of its sha256>". Logs
# then show that the firmware is a local build, and a different package gives a different version.
# At run time, the environment variable DEPTHAI_DEVICE_RVC4_FWP still selects another package. Use that variable
# while you iterate on the firmware: a new embedded package changes the version, and the library is compiled again.
#
# DepthaiDeviceRvc4LocalFwp(<package path> <output folder> <version variable> <resource list variable>)
#   <version variable>        in: DEPTHAI_DEVICE_RVC4_VERSION (gives the base version), out: version of the local package
#   <resource list variable>  out: files to embed
function(DepthaiDeviceRvc4LocalFwp fwp_path output_dir version_var resource_list_var)
    # A relative path is relative to the source directory
    get_filename_component(_fwp "${fwp_path}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
    if(NOT EXISTS "${_fwp}" OR IS_DIRECTORY "${_fwp}")
        message(FATAL_ERROR "DEPTHAI_DEVICE_RVC4_FWP_PATH: '${_fwp}' is not a file")
    endif()

    file(SHA256 "${_fwp}" _sha256)
    string(SUBSTRING "${_sha256}" 0 12 _short_sha256)
    # "0.0.1+<commit>" or "0.0.1" -> "0.0.1"; no version: the version of device/rvc4
    string(REGEX REPLACE "\\+.*$" "" _base_version "${${version_var}}")
    if(_base_version STREQUAL "" AND EXISTS "${CMAKE_SOURCE_DIR}/device/rvc4/version.txt")
        file(STRINGS "${CMAKE_SOURCE_DIR}/device/rvc4/version.txt" _base_version LIMIT_COUNT 1)
        string(STRIP "${_base_version}" _base_version)
    endif()
    set(_version "${_base_version}+local.${_short_sha256}")

    # configure_file makes the configure step run again when the package changes
    set(_embedded_fwp "${output_dir}/depthai-device-rvc4-fwp-${_version}.tar.xz")
    configure_file("${_fwp}" "${_embedded_fwp}" COPYONLY)
    set(_resources "${_embedded_fwp}")

    # License bundle that the firmware build writes next to the package (depthai-device-rvc4-fwp-LICENSE)
    get_filename_component(_fwp_dir "${_fwp}" DIRECTORY)
    get_filename_component(_fwp_name "${_fwp}" NAME)
    string(REGEX REPLACE "\\.tar\\.xz$" "" _fwp_stem "${_fwp_name}")
    set(_license "${_fwp_dir}/${_fwp_stem}-LICENSE")
    if(EXISTS "${_license}")
        set(_embedded_license "${output_dir}/depthai-device-rvc4-fwp-${_version}-LICENSE")
        configure_file("${_license}" "${_embedded_license}" COPYONLY)
        list(APPEND _resources "${_embedded_license}")
    else()
        message(STATUS "No license bundle next to the local RVC4 firmware package (${_fwp_stem}-LICENSE): none is embedded")
    endif()

    message(STATUS "RVC4 firmware: local package ${_fwp}, version ${_version}")
    set(${version_var} "${_version}" PARENT_SCOPE)
    set(${resource_list_var} "${_resources}" PARENT_SCOPE)
endfunction()
