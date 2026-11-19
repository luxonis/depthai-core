# Downloads the prebuilt vendor module of the RVC4 firmware for a firmware build of device/rvc4. The vendor module is
# the licensee-only Qualcomm and ST code behind the public interface headers. Its CI builds it and publishes the
# archive in the folder of the firmware package of the same version:
#   <base url>/<repository of the maturity>/depthai-device-rvc4/<version>/depthai-device-rvc4-vendor-<version><suffix>.tar.xz
# with its sha256 in <archive>.sha256. The version is DEPTHAI_DEVICE_RVC4_VENDOR_VERSION and the maturity
# DEPTHAI_DEVICE_RVC4_MATURITY of cmake/Depthai/DepthaiDeviceRVC4Config.cmake. The suffix is the sanitizer variant:
# empty, -asan-ubsan or -tsan. DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL (CMake or environment) sets another base URL. The
# environment variable, when it is set, replaces the cached value at each configure step (empty: the default URL).

include_guard(GLOBAL)
set(_DEPTHAI_DEVICE_RVC4_VENDOR_CONFIG "${CMAKE_CURRENT_LIST_DIR}/Depthai/DepthaiDeviceRVC4Config.cmake")

set(_DEPTHAI_DEVICE_RVC4_VENDOR_DEFAULT_URL "https://artifacts.luxonis.com/artifactory")
set(DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL "${_DEPTHAI_DEVICE_RVC4_VENDOR_DEFAULT_URL}" CACHE STRING "Base URL of the repositories with the prebuilt RVC4 vendor module")
if(DEFINED ENV{DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL})
    set(_base_url "$ENV{DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL}")
    if(_base_url STREQUAL "")
        set(_base_url "${_DEPTHAI_DEVICE_RVC4_VENDOR_DEFAULT_URL}")
    endif()
    set(DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL "${_base_url}" CACHE STRING "Base URL of the repositories with the prebuilt RVC4 vendor module" FORCE)
    unset(_base_url)
endif()

# DepthaiDeviceRvc4VendorDownloader(<suffix> <output dir> <out var>): sets <out var> to the archive in <output dir>.
# An archive that is there already and has the sha256 of the last download is used without a download.
function(DepthaiDeviceRvc4VendorDownloader suffix output_dir out_var)
    include("${_DEPTHAI_DEVICE_RVC4_VENDOR_CONFIG}")
    if("${DEPTHAI_DEVICE_RVC4_VENDOR_VERSION}" STREQUAL "")
        message(FATAL_ERROR "DEPTHAI_DEVICE_RVC4_VENDOR_VERSION is not set in ${_DEPTHAI_DEVICE_RVC4_VENDOR_CONFIG}. "
                            "Set DEPTHAI_DEVICE_RVC4_VENDOR to a local vendor archive.")
    endif()
    if(DEPTHAI_DEVICE_RVC4_MATURITY STREQUAL "release")
        set(_repository "luxonis-rvc4-release-local")
    else()
        set(_repository "luxonis-rvc4-snapshot-local")
    endif()
    set(_version "${DEPTHAI_DEVICE_RVC4_VENDOR_VERSION}")
    set(_name "depthai-device-rvc4-vendor-${_version}${suffix}.tar.xz")
    set(_url "${DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL}/${_repository}/depthai-device-rvc4/${_version}/${_name}")
    set(_archive "${output_dir}/${_name}")

    if(EXISTS "${_archive}" AND EXISTS "${_archive}.sha256")
        file(READ "${_archive}.sha256" _expected)
        file(SHA256 "${_archive}" _actual)
        if(_actual STREQUAL _expected)
            message(STATUS "RVC4 vendor module ${_version}${suffix}: ${_archive}")
            set(${out_var} "${_archive}" PARENT_SCOPE)
            return()
        endif()
    endif()

    file(MAKE_DIRECTORY "${output_dir}")
    file(REMOVE "${_archive}" "${_archive}.sha256")
    file(DOWNLOAD "${_url}.sha256" "${_archive}.sha256.download" TIMEOUT 60 STATUS _status)
    list(GET _status 0 _code)
    if(NOT _code EQUAL 0)
        list(GET _status 1 _message)
        file(REMOVE "${_archive}.sha256.download")
        message(FATAL_ERROR "No RVC4 vendor module ${_version}${suffix} at ${_url}.sha256 (${_message}). "
                            "Set DEPTHAI_DEVICE_RVC4_VENDOR to a local vendor archive.")
    endif()
    file(READ "${_archive}.sha256.download" _hash)
    file(REMOVE "${_archive}.sha256.download")
    string(REGEX MATCH "[0-9a-fA-F]+" _hash "${_hash}")
    string(TOLOWER "${_hash}" _hash)
    message(STATUS "RVC4 vendor module ${_version}${suffix}: downloading ${_url}")
    file(DOWNLOAD "${_url}" "${_archive}" EXPECTED_HASH "SHA256=${_hash}" TIMEOUT 600 INACTIVITY_TIMEOUT 60 STATUS _status)
    list(GET _status 0 _code)
    if(NOT _code EQUAL 0)
        list(GET _status 1 _message)
        file(REMOVE "${_archive}")
        message(FATAL_ERROR "The download of the RVC4 vendor module ${_url} failed: ${_message}")
    endif()
    file(WRITE "${_archive}.sha256" "${_hash}")
    set(${out_var} "${_archive}" PARENT_SCOPE)
endfunction()
