cmake_minimum_required(VERSION 3.13) # For Hunter

# ---------------------------------------------------------------------
# Cross toolchain and RVC4 sysroot
# ---------------------------------------------------------------------
#   oe   the OpenEmbedded SDK of the Luxonis OS SDK image. Its environment script sets CC, CXX, the flags and
#        OECORE_TARGET_SYSROOT (toolchains/OEToolchainConfig.cmake).
#   gnu  a GNU cross compiler (aarch64-linux-gnu) with a plain sysroot directory, as in the public build container docker/Dockerfile.sdk
#        (toolchains/rvc4-gnu.cmake).
#   auto oe when the OpenEmbedded SDK environment is sourced, else gnu.
set(DEPTHAI_DEVICE_RVC4_TOOLCHAIN "auto" CACHE STRING "Cross toolchain of the firmware: auto, oe or gnu")
set_property(CACHE DEPTHAI_DEVICE_RVC4_TOOLCHAIN PROPERTY STRINGS auto oe gnu)
set(DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED "${DEPTHAI_DEVICE_RVC4_TOOLCHAIN}")
if(DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED STREQUAL "auto")
    if(DEFINED ENV{OECORE_TARGET_SYSROOT})
        set(DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED "oe")
    else()
        set(DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED "gnu")
    endif()
endif()
if(DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED STREQUAL "oe")
    set(_rvc4_toolchain_file "${CMAKE_CURRENT_LIST_DIR}/toolchains/OEToolchainConfig.cmake")
    # The hard-coded location of the current SDK image is the fallback
    set(_rvc4_sysroot_default "$ENV{OECORE_TARGET_SYSROOT}")
    if(NOT _rvc4_sysroot_default AND EXISTS "/usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux")
        set(_rvc4_sysroot_default "/usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux")
    endif()
elseif(DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED STREQUAL "gnu")
    set(_rvc4_toolchain_file "${CMAKE_CURRENT_LIST_DIR}/toolchains/rvc4-gnu.cmake")
    set(_rvc4_sysroot_default "$ENV{DEPTHAI_DEVICE_RVC4_SYSROOT}")
else()
    message(FATAL_ERROR "DEPTHAI_DEVICE_RVC4_TOOLCHAIN must be auto, oe or gnu, not '${DEPTHAI_DEVICE_RVC4_TOOLCHAIN}'")
endif()
set(DEPTHAI_DEVICE_RVC4_SYSROOT "" CACHE PATH "RVC4 target sysroot (default: OECORE_TARGET_SYSROOT for oe, DEPTHAI_DEVICE_RVC4_SYSROOT for gnu)")
if(NOT DEPTHAI_DEVICE_RVC4_SYSROOT)
    set(DEPTHAI_DEVICE_RVC4_SYSROOT "${_rvc4_sysroot_default}" CACHE PATH "RVC4 target sysroot (default: OECORE_TARGET_SYSROOT for oe, DEPTHAI_DEVICE_RVC4_SYSROOT for gnu)" FORCE)
endif()
if(NOT EXISTS "${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/include/python3.10/")
    message(FATAL_ERROR "Could not find the RVC4 sysroot for the ${DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED} toolchain at '${DEPTHAI_DEVICE_RVC4_SYSROOT}'. "
                        "oe: source the SDK environment script. gnu: set DEPTHAI_DEVICE_RVC4_SYSROOT (the public build container sets it).")
endif()
message(STATUS "RVC4 toolchain: ${DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED} (${_rvc4_toolchain_file})")
# The vcpkg port builds read the triplets and the toolchain file in other processes, which see only the environment
set(ENV{DEPTHAI_DEVICE_RVC4_TOOLCHAIN} "${DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED}")
set(ENV{DEPTHAI_DEVICE_RVC4_SYSROOT} "${DEPTHAI_DEVICE_RVC4_SYSROOT}")
if(DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED STREQUAL "gnu")
    # The Python finders ask the host python3 for its headers when these are not set; the public image has none, and
    # the target headers come first on the include path anyway. FindPython of depthai-core reads Python_INCLUDE_DIR,
    # the FindPythonLibsNew of pybind11 in this directory reads PYTHON_INCLUDE_DIR (and overwrites it unless told not to).
    set(Python_INCLUDE_DIR "${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/include/python3.10" CACHE PATH "Python headers of the target")
    set(PYTHON_INCLUDE_DIR "${Python_INCLUDE_DIR}" CACHE PATH "Python headers of the target")
    set(PYBIND11_PYTHONLIBS_OVERWRITE OFF CACHE BOOL "Keep PYTHON_INCLUDE_DIR (RVC4 gnu toolchain)")
endif()
# pkg-config (libusb for XLink, GStreamer) searches the sysroot only
set(ENV{PKG_CONFIG_LIBDIR} "${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/lib/pkgconfig:${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${DEPTHAI_DEVICE_RVC4_SYSROOT}")
# QAIRT (SNPE) SDK inside the sysroot. The firmware is built against a fixed version; other
# installed versions are only used as a fallback.
set(DEPTHAI_DEVICE_QAIRT_VERSION "2.32.6.250402" CACHE STRING "QAIRT/SNPE SDK version under <sysroot>/opt/qcom/aistack/qairt to build against")
set(SNPE_ROOT "${DEPTHAI_DEVICE_RVC4_SYSROOT}/opt/qcom/aistack/qairt/${DEPTHAI_DEVICE_QAIRT_VERSION}")
if(NOT EXISTS "${SNPE_ROOT}/include/SNPE")
    file(GLOB _qairt_candidates LIST_DIRECTORIES true "${DEPTHAI_DEVICE_RVC4_SYSROOT}/opt/qcom/aistack/qairt/*")
    list(SORT _qairt_candidates COMPARE NATURAL)
    list(REVERSE _qairt_candidates)
    foreach(_c IN LISTS _qairt_candidates)
        if(EXISTS "${_c}/include/SNPE")
            set(SNPE_ROOT "${_c}")
            message(WARNING "QAIRT ${DEPTHAI_DEVICE_QAIRT_VERSION} not found in the sysroot, using ${_c}")
            break()
        endif()
    endforeach()
endif()
if(NOT EXISTS "${SNPE_ROOT}/include/SNPE")
    message(FATAL_ERROR "No QAIRT/SNPE SDK found under ${DEPTHAI_DEVICE_RVC4_SYSROOT}/opt/qcom/aistack/qairt. "
                        "create_deployment_artifacts.sh and start_dev_shell.sh mount it there (README.md, Public build container).")
endif()
message(STATUS "RVC4 sysroot: ${DEPTHAI_DEVICE_RVC4_SYSROOT}; QAIRT: ${SNPE_ROOT}")
# Work around the broken OpenCV CMake export in the SDK sysroot (no root needed).
include("${CMAKE_CURRENT_LIST_DIR}/cmake/SysrootOpenCV.cmake")
depthai_device_fix_sysroot_opencv("${DEPTHAI_DEVICE_RVC4_SYSROOT}" "${CMAKE_CURRENT_BINARY_DIR}/sysroot-fixes")


option(DEPTHAI_ASAN_UBSAN "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(DEPTHAI_TSAN "Enable ThreadSanitizer" OFF)

set(DEPTHAI_ASAN_UBSAN_FLAGS
    -fno-omit-frame-pointer
    -fno-optimize-sibling-calls
    -fsanitize=address,undefined,leak
    -fsanitize-address-use-after-scope
    -fsanitize=float-divide-by-zero
    -fno-sanitize=pointer-overflow
    -fno-sanitize-recover=all
)

# Project wide options
# Disable FW in core
set(VCPKG_OVERLAY_TRIPLETS "${CMAKE_CURRENT_LIST_DIR}/triplets" CACHE STRING "Path to vcpkg triplet files" FORCE)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${_rvc4_toolchain_file}" CACHE STRING "Toolchain file to overload" FORCE)
if(DEPTHAI_ASAN_UBSAN)
    message(STATUS "Enabling AddressSanitizer and UndefinedBehaviorSanitizer")
    set(VCPKG_TARGET_TRIPLET "rvc4-asan-ubsan" CACHE STRING "Vcpkg target triplet" FORCE)
    set(DEPTHAI_SANITIZE ON CACHE BOOL "Enable sanitizers in depthai-core" FORCE)

    set(SANITIZE_ADDRESS ON CACHE BOOL "Enable AddressSanitizer" FORCE)
    set(SANITIZE_UNDEFINED ON CACHE BOOL "Enable UndefinedSanitizer" FORCE)
    set(SANITIZE_THREAD OFF CACHE BOOL "Enable ThreadSanitizer" FORCE)
elseif(DEPTHAI_TSAN)
    message(STATUS "Enabling ThreadSanitizer")
    set(VCPKG_TARGET_TRIPLET "rvc4-tsan" CACHE STRING "Vcpkg target triplet" FORCE)
    set(DEPTHAI_SANITIZE ON CACHE BOOL "Enable sanitizers in depthai-core" FORCE)

    set(SANITIZE_ADDRESS OFF CACHE BOOL "Enable AddressSanitizer" FORCE)
    set(SANITIZE_UNDEFINED OFF CACHE BOOL "Enable UndefinedSanitizer" FORCE)
    set(SANITIZE_THREAD ON CACHE BOOL "Enable ThreadSanitizer" FORCE)
else()
    set(VCPKG_TARGET_TRIPLET "rvc4" CACHE STRING "Vcpkg target triplet" FORCE)
endif()
# vcpkg: a checkout of the pinned commit, by default in <this directory>/vcpkg. It is not a submodule of
# depthai-core: the configure step fetches it when it is missing.
set(DEPTHAI_DEVICE_VCPKG_COMMIT "b2cb0da531c2f1f740045bfe7c4dac59f0b2b69c" CACHE STRING "vcpkg commit that the firmware build uses")
set(DEPTHAI_DEVICE_VCPKG_ROOT "${CMAKE_CURRENT_LIST_DIR}/vcpkg" CACHE PATH "vcpkg checkout used by the build (toolchain, license and SBOM generation)")
if(NOT EXISTS "${DEPTHAI_DEVICE_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    find_package(Git REQUIRED)
    message(STATUS "Fetching vcpkg ${DEPTHAI_DEVICE_VCPKG_COMMIT} into ${DEPTHAI_DEVICE_VCPKG_ROOT}")
    file(MAKE_DIRECTORY "${DEPTHAI_DEVICE_VCPKG_ROOT}")
    foreach(_vcpkg_git_args
            "init;-q"
            "fetch;-q;--depth;1;https://github.com/microsoft/vcpkg.git;${DEPTHAI_DEVICE_VCPKG_COMMIT}"
            "checkout;-q;FETCH_HEAD")
        execute_process(COMMAND "${GIT_EXECUTABLE}" ${_vcpkg_git_args} WORKING_DIRECTORY "${DEPTHAI_DEVICE_VCPKG_ROOT}" RESULT_VARIABLE _vcpkg_git_result)
        if(NOT _vcpkg_git_result EQUAL 0)
            message(FATAL_ERROR "Could not fetch vcpkg (git ${_vcpkg_git_args}). Put a checkout of vcpkg commit ${DEPTHAI_DEVICE_VCPKG_COMMIT} "
                                "at ${DEPTHAI_DEVICE_VCPKG_ROOT}, or set DEPTHAI_DEVICE_VCPKG_ROOT to one.")
        endif()
    endforeach()
endif()
set(CMAKE_TOOLCHAIN_FILE "${DEPTHAI_DEVICE_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Toolchain file to overload" FORCE)

set(DEPTHAI_ENABLE_DEVICE_FW OFF CACHE BOOL "" FORCE)
set(DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW OFF CACHE BOOL "" FORCE)
set(DEPTHAI_ENABLE_DEVICE_RVC3_FW OFF CACHE BOOL "" FORCE)
set(DEPTHAI_ENABLE_DEVICE_RVC4_FW OFF CACHE BOOL "" FORCE)
# DynamicCalibration runs on the device: libdynamic_calibration.so is shipped in the fwp next to the executable
set(DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT ON CACHE BOOL "" FORCE)
set(XLINK_LIBUSB_SYSTEM ON CACHE BOOL "Use target sysroot libusb for XLink" FORCE)

# Set compiler features (c++17), and disables extensions (g++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add core: this directory is device/rvc4 of the depthai-core repository, so the firmware compiles the node
# definitions and message types of the same commit as the host library.
get_filename_component(_depthai_core_default_path "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(DEPTHAI_DEVICE_KB_CORE_PATH "${_depthai_core_default_path}" CACHE PATH "Path to the depthai-core source tree")
set(DEPTHAI_PYTHON_EMBEDDED_MODULE ON CACHE BOOL "Set depthai python as embedded module" FORCE)
set(DEPTHAI_BUILD_PYTHON ON CACHE BOOL "Build depthai python module" FORCE)
set(DEPTHAI_ENABLE_CURL OFF CACHE BOOL "Enable curl" FORCE)
set(DEPTHAI_ENABLE_PROTOBUF OFF CACHE BOOL "Enable protobuf" FORCE)
set(DEPTHAI_ENABLE_REMOTE_CONNECTION OFF CACHE BOOL "Enable remote connection" FORCE)
set(DEPTHAI_ENABLE_EVENTS_MANAGER OFF CACHE BOOL "Enable events manager" FORCE)
set(DEPTHAI_BOOTSTRAP_VCPKG OFF CACHE BOOL "Enable vcpkg" FORCE)
set(DEPTHAI_NEW_FIND_PYTHON OFF CACHE BOOL "Enable new find python" FORCE)
set(DEPTHAI_XTENSOR_SUPPORT ON CACHE BOOL "Enable xtensor support" FORCE)
set(DEPTHAI_VCPKG_INTERNAL_ONLY OFF CACHE BOOL "Enable vcpkg internal only" FORCE)
set(DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4 ON CACHE BOOL "Enable rvc4 define in depthai-core" FORCE)
if(DEFINED ENV{CI})
    set(DEPTHAI_BUILD_BETA ON CACHE BOOL "Build beta namespace APIs" FORCE)
endif()
# Create depthai project
file(READ ${CMAKE_CURRENT_LIST_DIR}/version.txt version)
string(STRIP "${version}" version)

project(depthai-device-rvc4 VERSION ${version} LANGUAGES CXX C ASM)

# The release image of the public build has no sanitizer libraries; its dev image has them. Stop here with the reason,
# and not at the link step.
if((DEPTHAI_ASAN_UBSAN OR DEPTHAI_TSAN) AND DEPTHAI_DEVICE_RVC4_TOOLCHAIN_RESOLVED STREQUAL "gnu")
    if(DEPTHAI_TSAN)
        set(_rvc4_sanitizer_library libtsan.so)
    else()
        set(_rvc4_sanitizer_library libasan.so)
    endif()
    execute_process(COMMAND "${CMAKE_CXX_COMPILER}" -print-file-name=${_rvc4_sanitizer_library}
                    OUTPUT_VARIABLE _rvc4_sanitizer_library_path OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT IS_ABSOLUTE "${_rvc4_sanitizer_library_path}")
        message(FATAL_ERROR "The toolchain of this build image has no ${_rvc4_sanitizer_library}. The sanitizer targets need the "
                            "dev image (the pinned image of DEPTHAI_DEVICE_RVC4_SDK_IMAGE with -dev). "
                            "create_deployment_artifacts.sh takes it when DEPTHAI_DEVICE_SDK_IMAGE is not set.")
    endif()
endif()

# Location of this tree and of its build directory. They differ from CMAKE_SOURCE_DIR /
# CMAKE_BINARY_DIR when the firmware is built as a subdirectory of depthai-core.
set(DEPTHAI_DEVICE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(DEPTHAI_DEVICE_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Debug data in the object files. Without it the firmware in the package is the same, because the package has the
# stripped binary. The build is then faster and its build folder is 10 times smaller. Debug data is necessary for a
# debugger and for the file and line of an address in a crash report.
option(DEPTHAI_DEVICE_DEBUG_INFO "Compile with debug data (-g)" ON)
if(NOT DEPTHAI_DEVICE_DEBUG_INFO)
    add_compile_options(-g0)
endif()

# GCC on AArch64 tells about each function that passes a type such as std::pair<float, float> by value: GCC 10.1
# changed the parameter passing of these types for C++17. The change is only a problem when the program links code
# from a GCC older than 10.1. All objects in the firmware come from GCC 11.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:-Wno-psabi>)
endif()

find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")

    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()

# include(CTest) of depthai-core would default BUILD_TESTING to ON, and the subprojects (cpp-subprocess) would then
# build their tests.
set(BUILD_TESTING OFF CACHE BOOL "Build the tests of the subprojects")

# depthai-core is not below this directory, so it needs an explicit binary directory
add_subdirectory("${DEPTHAI_DEVICE_KB_CORE_PATH}" "${CMAKE_BINARY_DIR}/depthai-core" EXCLUDE_FROM_ALL)
message(STATUS "Project version ${PROJECT_VERSION}")
target_compile_definitions(depthai PRIVATE DEPTHAI_SCRIPT_NODE_ADD_IO)
target_compile_definitions(depthai PRIVATE DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4)

# Generate build/version.hpp (global build info)
file(MAKE_DIRECTORY "${DEPTHAI_DEVICE_BINARY_DIR}/build")

configure_file(
    "${DEPTHAI_DEVICE_SOURCE_DIR}/cmake/version.hpp.in"
    "${DEPTHAI_DEVICE_BINARY_DIR}/build/version.hpp"
)

# Make sure all targets can find it
include_directories(BEFORE
    "${DEPTHAI_DEVICE_BINARY_DIR}"
)

# Vendor module: the licensee-only Qualcomm/ST code (not part of this repository) behind the C interface
# src/vendor/dai_device_*.h. The build links its prebuilt archive (cmake/DepthaiDeviceRvc4VendorPrebuilt.cmake), the first of:
#   DEPTHAI_DEVICE_RVC4_VENDOR  a local archive depthai-device-rvc4-vendor-<version>[-asan-ubsan|-tsan].tar.xz, in place of
#                               the download (as DEPTHAI_DEVICE_RVC4_FWP is a local firmware package for depthai-core)
#   the archive of DEPTHAI_DEVICE_RVC4_VENDOR_VERSION in cmake/Depthai/DepthaiDeviceRVC4Config.cmake of depthai-core,
#   downloaded (cmake/DepthaiDeviceRvc4VendorDownloader.cmake of depthai-core)
# The private CI of the vendor module builds it from its sources instead: DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR (Luxonis
# licensees only). A build with the prebuilt archive needs DEPTHAI_ACCEPT_VENDOR_EULA=yes: the archive is proprietary
# software of Luxonis and its licensors under the Luxonis DepthAI Vendor Module End User License Agreement
# (legal/DEPTHAI-EULA.txt in the archive, in each firmware package and in its license bundle; the CI of the module also
# publishes it next to the archive, and without the acceptance the build downloads it and stops). An environment
# variable of the same name as one of these settings, when it is set (also when it is empty), replaces the cached value
# at each configure step: the build scripts set both, so a build folder follows the last call.
include("${CMAKE_CURRENT_LIST_DIR}/cmake/DepthaiDeviceRvc4VendorPrebuilt.cmake")
include("${DEPTHAI_DEVICE_KB_CORE_PATH}/cmake/DepthaiDeviceRvc4VendorDownloader.cmake")
set(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR "" CACHE PATH "Sources of the vendor module (its private CI and Luxonis licensees)")
set(DEPTHAI_DEVICE_RVC4_VENDOR "" CACHE FILEPATH "Local prebuilt vendor archive (depthai-device-rvc4-vendor-<version>.tar.xz) in place of the download")
set(DEPTHAI_ACCEPT_VENDOR_EULA "" CACHE STRING "yes: accept the Luxonis DepthAI Vendor Module End User License Agreement, which covers the prebuilt vendor archive")
foreach(_vendor_setting DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR DEPTHAI_DEVICE_RVC4_VENDOR DEPTHAI_ACCEPT_VENDOR_EULA)
    if(DEFINED ENV{${_vendor_setting}})
        get_property(_vendor_help CACHE ${_vendor_setting} PROPERTY HELPSTRING)
        get_property(_vendor_type CACHE ${_vendor_setting} PROPERTY TYPE)
        set(${_vendor_setting} "$ENV{${_vendor_setting}}" CACHE ${_vendor_type} "${_vendor_help}" FORCE)
    endif()
endforeach()
unset(_vendor_setting)
unset(_vendor_help)
unset(_vendor_type)
set(DEPTHAI_DEVICE_RVC4_VENDOR_DIR "")
if(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR)
    if(DEPTHAI_DEVICE_RVC4_VENDOR)
        message(FATAL_ERROR "Set DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR (vendor module sources) or DEPTHAI_DEVICE_RVC4_VENDOR "
                            "(prebuilt vendor archive), not both.")
    endif()
    message(STATUS "Vendor module: sources '${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}'")
else()
    depthai_device_rvc4_vendor_identity()
    set(_vendor_suffix "")
    if(NOT DEPTHAI_DEVICE_RVC4_VENDOR_SANITIZER STREQUAL "none")
        set(_vendor_suffix "-${DEPTHAI_DEVICE_RVC4_VENDOR_SANITIZER}")
    endif()
    # The prebuilt archive is proprietary software of Luxonis and its licensors: the build uses it only after the user
    # accepts its EULA. The EULA of the archive is published next to it, so the build downloads it first and shows it.
    if(NOT DEPTHAI_ACCEPT_VENDOR_EULA STREQUAL "yes")
        if(DEPTHAI_DEVICE_RVC4_VENDOR)
            set(_vendor_eula "legal/DEPTHAI-EULA.txt in the archive ${DEPTHAI_DEVICE_RVC4_VENDOR}")
        else()
            DepthaiDeviceRvc4VendorEula("${_vendor_suffix}" "${DEPTHAI_DEVICE_BINARY_DIR}/vendor/download" _vendor_eula_file _vendor_eula_url)
            if(_vendor_eula_file)
                set(_vendor_eula "${_vendor_eula_file} (downloaded from ${_vendor_eula_url})")
            elseif(_vendor_eula_url)
                set(_vendor_eula "${_vendor_eula_url} (the download failed), and legal/DEPTHAI-EULA.txt in the archive")
            else()
                set(_vendor_eula "legal/DEPTHAI-EULA.txt in the archive")
            endif()
        endif()
        message(FATAL_ERROR "The firmware build uses the prebuilt vendor module (depthai-device-rvc4-vendor): proprietary "
                            "software of Luxonis and its licensors, for Luxonis devices only. The Luxonis DepthAI Vendor "
                            "Module End User License Agreement covers it. Read the agreement before you accept it: "
                            "${_vendor_eula}. The firmware that you build is under the Luxonis Firmware Package License "
                            "(LICENSE.fwp), and the agreement covers the vendor module in it. The firmware package "
                            "contains both texts (LICENSE.fwp and legal/DEPTHAI-EULA.txt).\n"
                            "To accept the agreement and use the archive, set DEPTHAI_ACCEPT_VENDOR_EULA=yes (environment "
                            "variable or CMake cache entry). Without the archive:\n"
                            "  * download a released firmware package for depthai-core (DEPTHAI_DEVICE_RVC4_VERSION), or\n"
                            "  * build with the sources of the vendor module (DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR, Luxonis "
                            "licensees only).")
    endif()
    if(DEPTHAI_DEVICE_RVC4_VENDOR)
        get_filename_component(_vendor_archive "${DEPTHAI_DEVICE_RVC4_VENDOR}" ABSOLUTE)
        if(NOT EXISTS "${_vendor_archive}" OR IS_DIRECTORY "${_vendor_archive}")
            message(FATAL_ERROR "DEPTHAI_DEVICE_RVC4_VENDOR is not a file: '${DEPTHAI_DEVICE_RVC4_VENDOR}'. Set it to a vendor archive "
                                "depthai-device-rvc4-vendor-<version>${_vendor_suffix}.tar.xz.")
        endif()
        message(STATUS "Vendor module: local archive '${_vendor_archive}' (DEPTHAI_DEVICE_RVC4_VENDOR)")
    else()
        DepthaiDeviceRvc4VendorDownloader("${_vendor_suffix}" "${DEPTHAI_DEVICE_BINARY_DIR}/vendor/download" _vendor_archive)
    endif()
    depthai_device_rvc4_vendor_extract("${_vendor_archive}" "${DEPTHAI_DEVICE_BINARY_DIR}/vendor" DEPTHAI_DEVICE_RVC4_VENDOR_DIR)
    message(STATUS "Vendor module: prebuilt '${DEPTHAI_DEVICE_RVC4_VENDOR_DIR}'")
    unset(_vendor_suffix)
    unset(_vendor_archive)
endif()

# STMicroelectronics VD55H1 ToF processing (ST SLA0106 library + Qualcomm depth-map wrapper), behind the C interface
# vendor/dai_device_tof.h. It is available when the vendor module sources have the wrapper (depthmapwrapper-vd55h1), or
# when the prebuilt vendor archive has a ToF part (its tof/ folder). Without it the firmware compiles the stub of the C
# functions, and the ToF node reports an error.
set(_tof_available OFF)
if(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR AND EXISTS "${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}/depthmapwrapper-vd55h1/CMakeLists.txt")
    set(_tof_available ON)
elseif(DEPTHAI_DEVICE_RVC4_VENDOR_DIR AND EXISTS "${DEPTHAI_DEVICE_RVC4_VENDOR_DIR}/tof/PREBUILT.libraries")
    set(_tof_available ON)
endif()
option(DEPTHAI_DEVICE_TOF_VD55H1 "Build with the VD55H1 ToF depth processing of the vendor module" ${_tof_available})
if(DEPTHAI_DEVICE_TOF_VD55H1 AND NOT _tof_available)
    message(FATAL_ERROR "DEPTHAI_DEVICE_TOF_VD55H1 is ON, but neither the vendor module sources (with depthmapwrapper-vd55h1) "
                        "nor the prebuilt vendor archive have the ToF part. Set DEPTHAI_DEVICE_TOF_VD55H1=OFF.")
endif()
message(STATUS "ToF VD55H1 processing: ${DEPTHAI_DEVICE_TOF_VD55H1}")
set(DEPTHAI_DEVICE_TOF_SOURCES src/pipeline/node/ToF/tofVD55H1.cpp)
if(NOT DEPTHAI_DEVICE_TOF_VD55H1)
    list(APPEND DEPTHAI_DEVICE_TOF_SOURCES src/pipeline/node/ToF/dai_device_tof_stub.cpp)
    # The node then reports "ToF VD55H1 support is not built into this firmware"
    set_source_files_properties(src/pipeline/node/ToF/tofVD55H1.cpp PROPERTIES COMPILE_DEFINITIONS DEPTHAI_DEVICE_TOF_VD55H1_STUB)
endif()
unset(_tof_available)

# Seek Thermal SDK library (libseekcamera.so.5.2) of the Seek Eagle thermal cores: src/camera/SeekEagle.cpp loads it at
# run time from seek-eagle/ of the firmware package, nothing links it. It is not in this repository: the vendor module
# sources have it in seek-sdk/, the prebuilt vendor archive in seek/. Without it the firmware package has only the
# loader stubs in seek-eagle/, and a Seek Eagle does not start.
set(DEPTHAI_DEVICE_SEEK_SDK_DIR "")
if(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR AND EXISTS "${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}/seek-sdk/libseekcamera.so.5.2")
    set(DEPTHAI_DEVICE_SEEK_SDK_DIR "${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}/seek-sdk")
elseif(DEPTHAI_DEVICE_RVC4_VENDOR_DIR AND EXISTS "${DEPTHAI_DEVICE_RVC4_VENDOR_DIR}/seek/libseekcamera.so.5.2")
    set(DEPTHAI_DEVICE_SEEK_SDK_DIR "${DEPTHAI_DEVICE_RVC4_VENDOR_DIR}/seek")
endif()
set(_seek_available OFF)
if(DEPTHAI_DEVICE_SEEK_SDK_DIR)
    set(_seek_available ON)
endif()
option(DEPTHAI_DEVICE_SEEK_SDK "Package the Seek Thermal SDK library of the vendor module (Seek Eagle thermal cores)" ${_seek_available})
if(DEPTHAI_DEVICE_SEEK_SDK AND NOT _seek_available)
    message(FATAL_ERROR "DEPTHAI_DEVICE_SEEK_SDK is ON, but neither the vendor module sources (seek-sdk/) nor the prebuilt "
                        "vendor archive (seek/) have the Seek Thermal SDK library. Set DEPTHAI_DEVICE_SEEK_SDK=OFF.")
endif()
message(STATUS "Seek Thermal SDK library: ${DEPTHAI_DEVICE_SEEK_SDK}")
unset(_seek_available)

# Add dependencies
find_package(Eigen3 CONFIG REQUIRED)
add_subdirectory("external")

target_include_directories(depthai BEFORE PUBLIC "${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/include/python3.10/")
target_include_directories(depthai BEFORE PUBLIC "${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/include/opencv4")
set(PYTHON_LIBRARY "${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/lib/libpython3.10.so")

# Set default build type depending on context
set(default_build_type "Release")

if(EXISTS "${DEPTHAI_DEVICE_SOURCE_DIR}/.git" AND NOT DEFINED ENV{CI})
    set(default_build_type "RelWithDebInfo")
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)

    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set policies
# CMP0074 dictates that find_package searches environment variable "[packageName]_ROOT" along with regular variable [packageName]_ROOT
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW) # Only introduced in 3.12
endif()

if(POLICY CMP0028)
    cmake_policy(SET CMP0028 NEW)
endif()

# Include constants
include("cmake/DepthaiModelsKbConfig.cmake")
include("cmake/DepthaiModelsKbDownloader.cmake")

# Set constant
set(DEPTHAI_RESOURCES_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/resources")
set(DEPTHAI_CUSTOM_SCRIPTS_DIR "${CMAKE_CURRENT_LIST_DIR}/scripts")
set(DEPTHAI_CUSTOM_SCRIPTS_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/scripts")

file(COPY ${DEPTHAI_CUSTOM_SCRIPTS_DIR} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Set to export compile commands for tools like clang-tidy and format
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ## Constants
set(PROJECT_EXPORT_GROUP "${PROJECT_NAME}Targets")
message(PROJECT_NAME="${PROJECT_NAME}")

# Add threads preference
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(THREADS_PREFER_PTHREAD_FLAG ON)

# ninja uses pipes to capture compiler output - that made it to omit coloring
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")

set(TARGET_DEVICE_KB_NAME ${PROJECT_NAME})

set(TARGET_DEVICE_KB_ALIAS device-rvc4)

# ---------------------------------------------------------------------
# Common source list
# ---------------------------------------------------------------------

set(DEPTHAI_COMMON_SOURCES
    src/XLinkRpc.cpp
    src/Logging.cpp
    src/SpdlogDaiConversion.cpp
    src/eeprom.cpp
    src/ImuExtrinsicsData.cpp
    src/ImuModelData.cpp
    src/Timesync.cpp
    src/utilities/GpuAvailability.cpp
    src/utilities/RectangleRemapUtils.cpp
    src/utilities/FsyncController.cpp
    src/pipeline/PipelineBuilder.cpp
    src/pipeline/node/XLinkOut.cpp
    src/pipeline/node/XLinkIn.cpp
    src/pipeline/node/DetectionParserNode.cpp
    src/pipeline/node/ImgDetectionsFilter.cpp
    src/pipeline/node/SegmentationParser.cpp
    src/pipeline/node/NNDataViewer.cpp
    src/pipeline/node/SpatialLocationCalculator.cpp
    src/pipeline/node/MessageDemux/MessageDemux.cpp
    src/pipeline/node/Script/Script.cpp
    src/utilities/CameraDiscovery.cpp
    src/utilities/CbaSensorDriver.cpp
    src/utilities/DeviceTelemetry.cpp
    src/utilities/IMUDiscovery.cpp
    src/utilities/AnonymousTelemetryId.cpp
    src/pipeline/node/Camera/Camera.cpp
    src/pipeline/node/ImgTransformationUtils.cpp
    src/pipeline/MultiDeviceCalibrationStore.cpp
)

if(DEPTHAI_BUILD_BETA)
    list(APPEND DEPTHAI_COMMON_SOURCES
        src/pipeline/node/ClassificationParser.cpp
        src/pipeline/node/ClassificationSequenceParser.cpp
        src/pipeline/node/EmbeddingsParser.cpp
        src/pipeline/node/FastSAMParser.cpp
        src/pipeline/node/HRNetParser.cpp
        src/pipeline/node/ImageOutputParser.cpp
        src/pipeline/node/KeypointParser.cpp
        src/pipeline/node/LaneDetectionParser.cpp
        src/pipeline/node/MLSDParser.cpp
        src/pipeline/node/MPPalmDetectionParser.cpp
        src/pipeline/node/MapOutputParser.cpp
        src/pipeline/node/PPTextDetectionParser.cpp
        src/pipeline/node/RFDETRParser.cpp
        src/pipeline/node/RegressionParser.cpp
        src/pipeline/node/SCRFDParser.cpp
        src/pipeline/node/SuperAnimalParser.cpp
        src/pipeline/node/XFeatMonoParser.cpp
        src/pipeline/node/XFeatStereoParser.cpp
        src/pipeline/node/YuNetParser.cpp
    )
endif()

# ---------------------------------------------------------------------
# Platform-specific source list
# ---------------------------------------------------------------------

set(DEPTHAI_PLATFORM_SOURCES
    src/utilities/RectangleRemapUtils.cpp

    # Utilities
    src/utilities/ClockOffset.cpp
    src/utilities/OSUtils.cpp
    src/utilities/PtpUtils.cpp
    src/utilities/DeviceModel.cpp
    src/utilities/FastCvUtils.cpp
    src/utilities/StereoPairs.cpp
    src/utilities/FpsRegulator.cpp

    # DeviceTreeDiscovery
    src/utilities/DeviceTreeDiscovery.cpp
    # The C interface of the vendor module (src/vendor/dai_device_*.h): readable errors and the log of the vendor module,
    # the accelerator buffers, and the public C++ wrappers of the units of the vendor module
    src/vendor/DeviceError.cpp
    src/utilities/AcceleratorMemory.cpp
    src/pipeline/node/OfflineJpegEncoder.cpp
    src/pipeline/node/FeatureTracker/FeatureTrackerAccel.cpp
    src/pipeline/node/ImageManip/DsImgDispatcher.cpp
    src/pipeline/node/StereoDepth/PyrImgDispatcher.cpp
    src/pipeline/node/StereoDepth/rvc4/StereoDepthDispatcherRvc4.cpp
    src/pipeline/node/Warp/rvc4/WarpDispatcherRvc4.cpp

    # Pipeline / nodes
    src/pipeline/node/NeuralNetwork/rvc4/NeuralNetworkRvc4.cpp
    src/pipeline/node/NeuralNetwork/CreateUserBuffer.cpp
    src/pipeline/node/NeuralDepth/NeuralDepth.cpp
    src/pipeline/node/DynamicCalibration/DynamicCalibration.cpp
    src/pipeline/node/ToFStereoFusion/ToFStereoFusion.cpp
    src/pipeline/node/GstPipelineInterface.cpp
    src/pipeline/node/rvc4/GstVaapiEncRvc4.cpp
    src/pipeline/node/VideoEncoder.cpp
    src/pipeline/node/Sync/Sync.cpp
    src/pipeline/node/ImageManip/rvc4/ImageManipRvc4.cpp
    src/pipeline/node/ImageManip/rvc4/gpu/ImageManipGPU.cpp
    src/pipeline/node/ImageManip/rvc4/gpu/ImageManipGPUMat.cpp
    src/pipeline/node/ImageManip/rvc4/gpu/ImageManipGPUMesh.cpp
    src/pipeline/node/PointCloud/PointCloudGpu.cpp
    src/pipeline/node/SystemLogger.cpp
    src/pipeline/node/Measurements.cpp
    src/pipeline/node/SystemResourceCounters.cpp
    src/pipeline/node/Warp/Warp.cpp
    src/pipeline/node/PipelineEventAggregation.cpp
    src/pipeline/node/Vpp/Vpp.cpp
    src/pipeline/node/Vpp/VppImpl.cpp
    src/pipeline/node/GPUStereo/GPUStereo.cpp
    src/pipeline/node/GPUStereo/gpu_stereo/cl_kernels.cpp
    src/pipeline/node/GPUStereo/gpu_stereo/PyramidStereo.cpp
    # StereoDepth
    src/pipeline/node/StereoDepth/rvc4/StereoDepthRvc4.cpp
    src/pipeline/node/StereoDepth/TemporalFilter/TemporalFilter.cpp
    src/pipeline/node/StereoDepth/SpatialFilter/SpatialFilter.cpp
    src/pipeline/node/StereoDepth/DecimationFilter/DecimationFilter.cpp
    src/pipeline/node/StereoDepth/SpeckleFilter/SpeckleFilter.cpp
    src/pipeline/node/StereoDepth/ThresholdFilter/ThresholdFilter.cpp
    src/pipeline/node/StereoDepth/MedianFilter/MedianFilter.cpp
    src/pipeline/node/StereoDepth/BrightnessFilter/BrightnessFilter.cpp

    # FeatureTracker / ObjectTracker / ImageAlign / GST
    src/pipeline/node/FeatureTracker/FeatureTrackerRvc4.cpp
    src/pipeline/node/Align/Align.cpp
    src/pipeline/node/ImageAlign/ImageAlign.cpp
    src/pipeline/node/ToF/ToF.cpp
    src/pipeline/node/ToF/BleedingFilter.cpp
    ${DEPTHAI_DEVICE_TOF_SOURCES}

    # Camera
    src/camera/CameraSensor.cpp
    src/camera/SeekEagle.cpp
    src/pipeline/node/Thermal/Thermal.cpp
    src/camera/IspRouteAllocator.cpp
    src/camera/LuxonisSidechannel.cpp
    src/camera/CameraSingleton.cpp
    src/camera/CameraPool.cpp
    src/camera/HalUtil.cpp

    # Drivers
    src/driver/spi/spi.c
)

# ---------------------------------------------------------------------
# Create the device core library
# ---------------------------------------------------------------------
add_library(depthai-device-core STATIC
    ${DEPTHAI_COMMON_SOURCES}
    ${DEPTHAI_PLATFORM_SOURCES}
)

# What the firmware libraries compile against: the include roots of depthai-core and of this tree, and the platform
# defines that change what these headers declare. The core library links it PUBLIC, so all firmware libraries see
# the same headers with the same defines. The vendor module does not link it: it links only depthai-device-vendor-api.
add_library(depthai-device-accel-api INTERFACE)
target_include_directories(depthai-device-accel-api
    INTERFACE
        "${DEPTHAI_DEVICE_KB_CORE_PATH}/include"
        "${DEPTHAI_DEVICE_KB_CORE_PATH}/src"
        "${CMAKE_CURRENT_LIST_DIR}/include"
        "${CMAKE_CURRENT_LIST_DIR}/src"
)
target_compile_definitions(depthai-device-accel-api
    INTERFACE
        TARGET_DEVICE_RVC4
        DEPTHAI_HAVE_FASTCV_SUPPORT
        # spdlog uses the external fmt (the fmt and spdlog ports of vcpkg.json)
        SPDLOG_FMT_EXTERNAL
)
target_link_libraries(depthai-device-accel-api INTERFACE depthai-core)

# What the vendor module (depthai-device-rvc4-vendor) and the DSP offload compile against: only the C interface
# (src/vendor/dai_device_*.h). It has no other header of depthai-core and no define, so a prebuilt vendor module stays
# valid when depthai-core changes.
add_library(depthai-device-vendor-api INTERFACE)
target_include_directories(depthai-device-vendor-api INTERFACE "${CMAKE_CURRENT_LIST_DIR}/src/vendor")

# Each header of the C interface compiles alone as C11 without warnings, so that no C++ comes into the interface
file(GLOB _vendor_api_headers RELATIVE "${CMAKE_CURRENT_LIST_DIR}/src/vendor" CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/src/vendor/dai_device_*.h")
set(_vendor_api_check_sources "")
foreach(_header IN LISTS _vendor_api_headers)
    string(REGEX REPLACE "\\.h$" ".c" _source "${CMAKE_CURRENT_BINARY_DIR}/vendor-api-check/${_header}")
    file(CONFIGURE OUTPUT "${_source}" CONTENT "#include \"${_header}\"\n" @ONLY)
    list(APPEND _vendor_api_check_sources "${_source}")
endforeach()
add_library(depthai-device-vendor-api-check OBJECT ${_vendor_api_check_sources})
target_link_libraries(depthai-device-vendor-api-check PRIVATE depthai-device-vendor-api)
set_target_properties(depthai-device-vendor-api-check PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
target_compile_options(depthai-device-vendor-api-check PRIVATE -Wall -Wextra -Wpedantic -Werror)
unset(_vendor_api_headers)
unset(_vendor_api_check_sources)

target_link_libraries(depthai-device-core
    PUBLIC
        depthai-device-accel-api
        depthai-device-vendor-api
)
add_dependencies(depthai-device-core depthai-device-vendor-api-check)

add_subdirectory(src/pipeline/node/IMU)
add_library("${PROJECT_NAME}::${TARGET_DEVICE_KB_ALIAS}" ALIAS depthai-device-core)

# SNPE library directory (SNPE_ROOT is selected at the top of this file)
target_link_directories(depthai-device-core PRIVATE ${SNPE_ROOT}/lib/aarch64-oe-linux-gcc11.2)
target_link_libraries(depthai-device-core PRIVATE gstqtimemorybase gstqtivideobase)

# add_executable("gstMinimalExample"
# gst_examples/gst_camera.cpp)

# target_link_libraries("gstMinimalExample"
# PRIVATE
# ${GST_LIBRARIES}
# gstallocators-1.0
# gstapp-1.0
# gstvideo-1.0
# )

# target_include_directories("gstMinimalExample"
# PRIVATE
# ${GST_INCLUDE_DIRS}
# )

# add_executable("gstMinimalExampleST"
# gst_examples/gst_camera_single_thread.cpp)

# target_link_libraries("gstMinimalExampleST"
# PRIVATE
# ${GST_LIBRARIES}
# gstallocators-1.0
# gstapp-1.0
# gstvideo-1.0
# )

# target_include_directories("gstMinimalExampleST"
# PRIVATE
# ${GST_INCLUDE_DIRS}
# )

# Specifies name of generated IMPORTED target (set to alias)
set_target_properties(depthai-device-core PROPERTIES EXPORT_NAME ${TARGET_DEVICE_KB_ALIAS})

# Thin archives for the three large static libraries. They are only an input of the firmware link. A normal archive
# holds a copy of each object file (2.7 GB for the Python bindings with debug data), and that copy takes more time
# than the link of the firmware. A library that leaves the build folder must stay a normal archive, because a thin
# archive only names its object files: the libraries of the private module and of the DSP offload go into the
# prebuilt archive.
set_target_properties(depthai-core depthai depthai-device-core PROPERTIES STATIC_LIBRARY_OPTIONS "--thin")

# Add to list of targets to export and install
list(APPEND targets_to_export depthai-device-core)

# Add module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/")

file(GLOB SNPE_INCLUDE_PATH "${SNPE_ROOT}/include/SNPE")

# Add include directories (the include roots of depthai-core and of this tree come from depthai-device-accel-api)
target_include_directories(depthai-device-core
    PUBLIC
    # Relative path to include directories after installed
    "$<INSTALL_INTERFACE:include>"

    # Build time path to include directories
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/depthai>"
    ${SNPE_INCLUDE_PATH}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/external/cpp-subprocess>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src/pipeline/node/IMU/rvc4>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src/pipeline/node/GPUStereo>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src/pipeline/node/GPUStereo/gpu_stereo>"
)

# Add clang format after specifying include directories
option(DEPTHAI_DEVICE_KB_CLANG_FORMAT "Enable clang-format target" ON)

if(DEPTHAI_DEVICE_KB_CLANG_FORMAT)
    # HEADER DIRECTORIES
    set(header_dirs "${CMAKE_CURRENT_LIST_DIR}/include")
    include(ClangFormat)
    target_clangformat_setup(depthai-device-core "${header_dirs}")
endif()

# Force Colored output when using Ninja
# Global option - affects all targets
option(FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)" OFF)

if(FORCE_COLORED_OUTPUT)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        target_compile_options(depthai-device-core PRIVATE "-fdiagnostics-color=always")
    elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        target_compile_options(depthai-device-core PRIVATE "-fcolor-diagnostics")
    endif()
endif()

target_compile_definitions(depthai-device-core PRIVATE CL_TARGET_OPENCL_VERSION=300)
option(DEPTHAI_GPUStereo_TIMING "Enable GPUStereo OpenCL per-stage wall timing (TIMING macro)" OFF)
if(DEPTHAI_GPUStereo_TIMING)
    set_source_files_properties(
        "${CMAKE_CURRENT_LIST_DIR}/src/pipeline/node/GPUStereo/GPUStereo.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/src/pipeline/node/GPUStereo/gpu_stereo/PyramidStereo.cpp"
        PROPERTIES COMPILE_DEFINITIONS TIMING=1
    )
    message(STATUS "DEPTHAI_GPUStereo_TIMING=ON (GPUStereo pipeline compiled with TIMING)")
endif()

# Find Git
find_package(Git)

# Add build information (commit, date)
set(BUILD_COMMIT "dev")

if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE BUILD_COMMIT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} show -s --format=%ci ${BUILD_COMMIT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE BUILD_COMMIT_DATETIME
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    # Commit of the depthai-core sources the firmware is built from (ABI identification of the private module)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${DEPTHAI_DEVICE_KB_CORE_PATH}
        OUTPUT_VARIABLE DEPTHAI_DEVICE_KB_CORE_COMMIT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()

# # Add local commit hash (or 'dev' if unable to retrieve) if not build by CI
# if(NOT DEFINED ENV{CI} AND NOT DEPTHAI_DEVICE_KB_COMMIT_HASH)
# set(DEPTHAI_DEVICE_KB_COMMIT_HASH ${BUILD_COMMIT})
# endif()

# Always add hash
set(DEPTHAI_DEVICE_KB_COMMIT_HASH ${BUILD_COMMIT})

# # link libraries
# target_link_libraries(depthai-device-core
# PUBLIC
# nlohmann_json::nlohmann_json
# XLink
# libnop
# PRIVATE
# Threads::Threads
# BZip2::bz2
# FP16::fp16
# archive_static
# spdlog::spdlog
# ZLIB::zlib
# )

# # DepthAI
# find_package(depthai)

# pybind for Script support
find_package(pybind11 CONFIG REQUIRED)

# Threads
find_package(Threads)

find_package(OpenCL REQUIRED)

## String formatting
find_package(fmt)
message(STATUS "Found fmt at ${fmt_DIR}")

find_package(spdlog CONFIG REQUIRED)
message(STATUS "Found spdlog at ${spdlog_DIR}")

find_package(PkgConfig REQUIRED)
pkg_search_module(GST REQUIRED gstreamer-1.0>=1.4 gstreamer-app-1.0>=1.4 gstallocators-1.0>=1.4)
message(STATUS "GST Libraries: ${GST_LIBRARIES}")

# Include directories (for non-CMake targets)
target_include_directories(depthai-device-core
    PRIVATE
    ${GST_INCLUDE_DIRS}
)

# Compile options required by the Qualcomm camera HAL headers (shared with the private module)
set(DEPTHAI_DEVICE_CAMERA_HAL_COMPILE_OPTIONS
    -DANDROID
    -DTARGET_ION_ABI_VERSION=2
    -DFLUSH_RESTART_NOTAVAILABLE
    -DTARGET_USES_GBM
    -DDISABLE_OP_MODES
    -DQCAMERA3_TAG_LOCAL_COPY
    -DCAM_ARCH_V2
    -DJPEG_BLOB_OFFSET=0
    -DHFR_THRESHOLD=120
    -DUSE_FPS_IDX
    -DDISABLE_RESCALER_COLORSPACE
    -DCAMERA_HAL_API_VERSION=0x0307
    -DCAMERA_METADATA_1_0_NS
    -DENABLE_IMAGE_NV12
    -DCAMX_ANDROID_API=32
    -DENABLE_OFFLINE_JPEG
    -DHAVE_BINDER
)
target_compile_options(depthai-device-core PRIVATE ${DEPTHAI_DEVICE_CAMERA_HAL_COMPILE_OPTIONS})

# link libraries
target_link_libraries(depthai-device-core
    PRIVATE
    OpenCL::OpenCL

    # PRIVATE
    depthai::core
    Threads::Threads
    dl
    tbb

    # depthai::opencv
    ${GST_LIBRARIES}
    gstallocators-1.0
    gstapp-1.0
    gstvideo-1.0

    # python bindings
    depthai
    pybind11::embed

    # cpp-subprocess
    subprocess

    # eigen
    Eigen3::Eigen
    SNPE
    opencv_core
    opencv_calib3d
    fastcvopt

    # HAL usage dependencies
    gbm
    hardware
    cutils

    # qualcomm qmmf-sdk
    camera_metadata
    qmmf_camera_metadata

    # string formatting (the fmt port of vcpkg; a plain "fmt" would link the libfmt of the OS sysroot)
    fmt::fmt
    spdlog::spdlog
    gpiod

    # HAL usage dependencies
    gbm
    hardware
    cutils

)

if(DEPTHAI_ASAN_UBSAN)
    target_compile_options(depthai-device-core PRIVATE ${DEPTHAI_ASAN_UBSAN_FLAGS})
    target_link_options(depthai-device-core PRIVATE ${DEPTHAI_ASAN_UBSAN_FLAGS})
elseif(DEPTHAI_TSAN)
    target_compile_options(depthai-device-core PRIVATE -fsanitize=thread)
    target_link_options(depthai-device-core PRIVATE -fsanitize=thread)
endif()
# ---------------------------------------------------------------------
# Hexagon DSP offload (ImageAlign and StereoDepth DSP kernels)
# ---------------------------------------------------------------------
# Every firmware carries the DSP offload. It is built from source when HEXAGON_SDK_ROOT has the Qualcomm Hexagon SDK
# (Luxonis builds use the SDK of the Luxonis OS SDK image; other builds install it from Qualcomm), else it comes from
# the dsp/ folder of the prebuilt vendor module.
set(DEPTHAI_DEVICE_HEXAGON_SDK_VERSION "5.5.0.1")  # the version that the firmware is built and tested with
set(_dsp_prebuilt OFF)
if(NOT DEFINED ENV{HEXAGON_SDK_ROOT} OR NOT EXISTS "$ENV{HEXAGON_SDK_ROOT}/build/cmake/hexagon_fun.cmake")
    if(NOT DEPTHAI_DEVICE_RVC4_VENDOR_DIR)
        message(FATAL_ERROR "The firmware needs the Qualcomm Hexagon SDK ${DEPTHAI_DEVICE_HEXAGON_SDK_VERSION} to build its DSP offload "
                            "with the vendor module sources: install it from Qualcomm and set the environment variable "
                            "HEXAGON_SDK_ROOT to it (now: '$ENV{HEXAGON_SDK_ROOT}').")
    endif()
    set(_dsp_prebuilt ON)
endif()
if(_dsp_prebuilt)
    depthai_device_dsp_prebuilt_import("${DEPTHAI_DEVICE_RVC4_VENDOR_DIR}/dsp")
    message(STATUS "Hexagon DSP offload: prebuilt '${DEPTHAI_DEVICE_RVC4_VENDOR_DIR}/dsp' (no Hexagon SDK at '$ENV{HEXAGON_SDK_ROOT}')")
else()
    set(_hexagon_sdk_version "unknown")
    if(EXISTS "$ENV{HEXAGON_SDK_ROOT}/hexagon_sdk.json")
        # Not strict JSON (trailing commas): take the first "version" entry, the one of the SDK
        file(READ "$ENV{HEXAGON_SDK_ROOT}/hexagon_sdk.json" _hexagon_sdk_json)
        if(_hexagon_sdk_json MATCHES "\"version\": *\"([0-9.]+)\"")
            set(_hexagon_sdk_version "${CMAKE_MATCH_1}")
        endif()
        unset(_hexagon_sdk_json)
    endif()
    message(STATUS "Hexagon DSP offload: Hexagon SDK ${_hexagon_sdk_version} at $ENV{HEXAGON_SDK_ROOT}")
    if(NOT _hexagon_sdk_version STREQUAL DEPTHAI_DEVICE_HEXAGON_SDK_VERSION)
        message(WARNING "The firmware is built and tested with the Hexagon SDK ${DEPTHAI_DEVICE_HEXAGON_SDK_VERSION}; "
                        "HEXAGON_SDK_ROOT has version ${_hexagon_sdk_version}.")
    endif()
    unset(_hexagon_sdk_version)
    add_subdirectory(src/hexagon/ImageAlignDsp)
    add_subdirectory(src/hexagon/StereoDepthDsp)
    set(DEPTHAI_DEVICE_DSP_SKELS
        "${DEPTHAI_DEVICE_BINARY_DIR}/src/hexagon/ImageAlignDsp/hexagon_build/libImageAlignDsp_skel.so"
        "${DEPTHAI_DEVICE_BINARY_DIR}/src/hexagon/StereoDepthDsp/hexagon_build/libStereoDepthDsp_skel.so")
endif()
target_link_libraries(depthai-device-core PRIVATE ImageAlignDsp StereoDepthDsp)

target_link_libraries(depthai-device-core PRIVATE pipeline.node.IMU)

# ---------------------------------------------------------------------
# Private module: licensee-only Qualcomm/ST code behind the public interfaces
# ---------------------------------------------------------------------
if(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR)
    add_subdirectory("${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}" "${DEPTHAI_DEVICE_BINARY_DIR}/depthai-device-rvc4-vendor")
    # One direction only: the core library uses the implementations of the private module, and the module
    # compiles only against depthai-device-vendor-api (the C interface).
    target_link_libraries(depthai-device-core PRIVATE depthai-device-rvc4-vendor)
    # The IMU node library uses the sensor hub functions of the private module (vendor/dai_device_sensor.h).
    target_link_libraries(pipeline.node.IMU PRIVATE depthai-device-rvc4-vendor)
    # VD55H1 ToF sensor pipeline: the separate archive of the private module when it is built with the ST wrapper
    if(DEPTHAI_DEVICE_TOF_VD55H1)
        target_link_libraries(depthai-device-core PRIVATE depthai-device-rvc4-vendor-tof)
    endif()
    depthai_device_rvc4_vendor_prebuilt_describe("${DEPTHAI_DEVICE_BINARY_DIR}/vendor-prebuilt")
else()
    depthai_device_rvc4_vendor_prebuilt_import("${DEPTHAI_DEVICE_RVC4_VENDOR_DIR}" "${DEPTHAI_DEVICE_BINARY_DIR}/vendor-prebuilt" ${_dsp_prebuilt})
    target_link_libraries(depthai-device-core PRIVATE depthai-device-rvc4-vendor)
    target_link_libraries(pipeline.node.IMU PRIVATE depthai-device-rvc4-vendor)
    # The ToF part of the archive (tof/), when it has one
    if(DEPTHAI_DEVICE_TOF_VD55H1)
        target_link_libraries(depthai-device-core PRIVATE depthai-device-rvc4-vendor-tof)
    endif()
    add_dependencies(depthai-device-core depthai-device-rvc4-vendor-prebuilt-check)
    add_dependencies(pipeline.node.IMU depthai-device-rvc4-vendor-prebuilt-check)
endif()

# Create final version
set(DEPTHAI_DEVICE_KB_FINAL_VERSION "${PROJECT_VERSION}")

if(DEFINED DEPTHAI_DEVICE_KB_COMMIT_HASH)
    set(DEPTHAI_DEVICE_KB_FINAL_VERSION "${PROJECT_VERSION}-${DEPTHAI_DEVICE_KB_COMMIT_HASH}")
endif()

target_compile_definitions(depthai-device-core PRIVATE DEPTHAI_DEVICE_KB_FINAL_VERSION="${DEPTHAI_DEVICE_KB_FINAL_VERSION}")
target_compile_definitions(depthai-device-core PRIVATE DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4)

# # Add packaging step (with versioning)
# add_custom_command(TARGET depthai-device-core POST_BUILD COMMAND
# ${CMAKE_COMMAND} -E tar cfJ depthai-device-kb-fwp-${DEPTHAI_DEVICE_KB_FINAL_VERSION}.tar.xz $<TARGET_FILE:depthai-device-core>
# COMMAND_EXPAND_LISTS
# )

# Add device models
DepthaiModelsKbDownload(
    FOLDER      "${DEPTHAI_DEVICE_BINARY_DIR}" # Placed next to the executable in the fwp
    OUTPUT_LIST DEPTHAI_MODELS_KB_RESOURCE_LIST # List of output resources
    MATURITY    "${DEPTHAI_MODELS_KB_MATURITY}" # Maturity
    VERSION     "${DEPTHAI_MODELS_KB_VERSION}" # Version directory, may be empty
    PATH_PREFIX "${DEPTHAI_MODELS_KB_PATH_PREFIX}" # Path under the repository
    NAME        "${DEPTHAI_MODELS_KB_TOF_FUSION_NAME}"
    SHA256      "${DEPTHAI_MODELS_KB_TOF_FUSION_SHA256}"
)

# ---------------------------------------------------------------------
# Create final device executable from the core library
# ---------------------------------------------------------------------
set(TARGET_DEVICE_EXECUTABLE depthai-device-rvc4)

add_executable(${TARGET_DEVICE_EXECUTABLE} src/main.cpp)
# Finds libdynamic_calibration.so next to the executable
set_target_properties(${TARGET_DEVICE_EXECUTABLE} PROPERTIES BUILD_RPATH "$ORIGIN")
target_sources(${TARGET_DEVICE_EXECUTABLE}
    PRIVATE
        src/utilities/GpioLine.cpp
        src/pipeline/node/Script/GPIOBindings.cpp
)

if(DEPTHAI_ASAN_UBSAN)
    target_compile_options(${TARGET_DEVICE_EXECUTABLE} PRIVATE ${DEPTHAI_ASAN_UBSAN_FLAGS})
    target_link_options(${TARGET_DEVICE_EXECUTABLE} PRIVATE ${DEPTHAI_ASAN_UBSAN_FLAGS})
elseif(DEPTHAI_TSAN)
    target_compile_options(${TARGET_DEVICE_EXECUTABLE} PRIVATE -fsanitize=thread)
    target_link_options(${TARGET_DEVICE_EXECUTABLE} PRIVATE -fsanitize=thread)
endif()

# Include dirs
target_include_directories(${TARGET_DEVICE_EXECUTABLE}
    PRIVATE
        ${GST_INCLUDE_DIRS}
        "${CMAKE_CURRENT_LIST_DIR}/src"
)

target_include_directories(${TARGET_DEVICE_EXECUTABLE}
    PRIVATE
        "${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/include/python3.10/"
)

# Link libraries
target_link_libraries(${TARGET_DEVICE_EXECUTABLE}
    PRIVATE
        ${GST_LIBRARIES}
)


target_link_libraries(${TARGET_DEVICE_EXECUTABLE} PRIVATE depthai-device-core)

# ---------------------------------------------------------------------
# Post-build packaging and debug info stripping
# ---------------------------------------------------------------------
# DSP skeletons: copied next to the executable and packed into the fwp.
set(DEPTHAI_DEVICE_DSP_SKEL_COPY_COMMANDS "")
set(DEPTHAI_DEVICE_DSP_SKELS_IN_FWP "")
foreach(_skel IN LISTS DEPTHAI_DEVICE_DSP_SKELS)
    get_filename_component(_skel_name "${_skel}" NAME)
    list(APPEND DEPTHAI_DEVICE_DSP_SKEL_COPY_COMMANDS COMMAND ${CMAKE_COMMAND} -E copy "${_skel}" "${DEPTHAI_DEVICE_BINARY_DIR}/${_skel_name}")
    list(APPEND DEPTHAI_DEVICE_DSP_SKELS_IN_FWP "${DEPTHAI_DEVICE_BINARY_DIR}/${_skel_name}")
endforeach()
# Seek Eagle thermal cores (src/camera/SeekEagle.cpp): seek-eagle/ next to the executable has the loader stubs of the
# three v4l-utils libraries that the Seek Thermal SDK library needs (src/camera/seek_stubs), and the library itself
# (DEPTHAI_DEVICE_SEEK_SDK).
set(DEPTHAI_DEVICE_SEEK_EAGLE_DIR "${DEPTHAI_DEVICE_BINARY_DIR}/seek-eagle")
set(DEPTHAI_DEVICE_SEEK_EAGLE_COMMANDS
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${DEPTHAI_DEVICE_SEEK_EAGLE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DEPTHAI_DEVICE_SEEK_EAGLE_DIR}")
foreach(_stub v4l2 v4l2subdev mediactl)
    add_library(seek-eagle-stub-${_stub} SHARED src/camera/seek_stubs/seek_sdk_stubs.c)
    set_target_properties(seek-eagle-stub-${_stub} PROPERTIES OUTPUT_NAME ${_stub} SOVERSION 0)
    add_dependencies(${TARGET_DEVICE_EXECUTABLE} seek-eagle-stub-${_stub})
    list(APPEND DEPTHAI_DEVICE_SEEK_EAGLE_COMMANDS
         COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:seek-eagle-stub-${_stub}>" "${DEPTHAI_DEVICE_SEEK_EAGLE_DIR}/lib${_stub}.so.0")
endforeach()
if(DEPTHAI_DEVICE_SEEK_SDK)
    list(APPEND DEPTHAI_DEVICE_SEEK_EAGLE_COMMANDS
         COMMAND ${CMAKE_COMMAND} -E copy "${DEPTHAI_DEVICE_SEEK_SDK_DIR}/libseekcamera.so.5.2" "${DEPTHAI_DEVICE_SEEK_EAGLE_DIR}/libseekcamera.so.5.2")
endif()
# The license bundle starts with LICENSE.fwp and ends with the legal files of the vendor module (NOTICE.txt, and legal/
# with the Luxonis DepthAI Vendor Module EULA that covers the module, NOTICE.md and the licence texts):
# DEPTHAI_DEVICE_RVC4_VENDOR_LEGAL_DIR, which the module sources (target depthai-device-rvc4-vendor-license) or the
# prebuilt archive supply. The SBOMs list the third-party parts of the private module sources (the ToF wrapper); a
# build with the prebuilt archive has none of them.
if(NOT DEPTHAI_DEVICE_RVC4_VENDOR_LEGAL_DIR)
    message(FATAL_ERROR "The vendor module gives no legal files (DEPTHAI_DEVICE_RVC4_VENDOR_LEGAL_DIR): the firmware "
                        "package license must include them.")
endif()
if(TARGET depthai-device-rvc4-vendor-license)
    add_dependencies(${TARGET_DEVICE_EXECUTABLE} depthai-device-rvc4-vendor-license)
endif()
set(DEPTHAI_DEVICE_LICENSE_PRIVATE_MODULE_ARGS --vendor-legal-dir "${DEPTHAI_DEVICE_RVC4_VENDOR_LEGAL_DIR}")
if(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR)
    list(APPEND DEPTHAI_DEVICE_LICENSE_PRIVATE_MODULE_ARGS --private-module-root "${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}")
endif()
# The firmware package itself contains its license (LICENSE.fwp) and the Luxonis DepthAI Vendor Module EULA of the
# vendor module that it links (legal/DEPTHAI-EULA.txt of the legal files of the module), and the license of the
# dynamic calibration library that it ships (legal/dynamic_calibration-LICENSE). The full license bundle, with
# the third-party notices, stays next to the package.
set(DEPTHAI_DEVICE_FWP_LEGAL_FILES "${DEPTHAI_DEVICE_BINARY_DIR}/LICENSE.fwp" "${DEPTHAI_DEVICE_BINARY_DIR}/legal/DEPTHAI-EULA.txt"
    "${DEPTHAI_DEVICE_BINARY_DIR}/legal/dynamic_calibration-LICENSE")
add_custom_command(TARGET ${TARGET_DEVICE_EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} --only-keep-debug "$<TARGET_FILE:${TARGET_DEVICE_EXECUTABLE}>" "$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/$<TARGET_FILE_BASE_NAME:${TARGET_DEVICE_EXECUTABLE}>.debug"
    COMMAND ${CMAKE_STRIP} --strip-unneeded "$<TARGET_FILE:${TARGET_DEVICE_EXECUTABLE}>"
    COMMAND ${CMAKE_OBJCOPY} --add-gnu-debuglink="$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/$<TARGET_FILE_BASE_NAME:${TARGET_DEVICE_EXECUTABLE}>.debug" "$<TARGET_FILE:${TARGET_DEVICE_EXECUTABLE}>"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${TARGET_DEVICE_EXECUTABLE}>" "${DEPTHAI_DEVICE_BINARY_DIR}/$<TARGET_FILE_NAME:${TARGET_DEVICE_EXECUTABLE}>"
    ${DEPTHAI_DEVICE_DSP_SKEL_COPY_COMMANDS}
    ${DEPTHAI_DEVICE_SEEK_EAGLE_COMMANDS}
    COMMAND ${CMAKE_STRIP} --strip-unneeded -o "${DEPTHAI_DEVICE_BINARY_DIR}/libdynamic_calibration.so" "${DYNAMIC_CALIBRATION_RELEASE}"
    COMMAND ${CMAKE_COMMAND} -E copy "${DEPTHAI_DEVICE_SOURCE_DIR}/LICENSE.fwp" "${DEPTHAI_DEVICE_BINARY_DIR}/LICENSE.fwp"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DEPTHAI_DEVICE_BINARY_DIR}/legal"
    COMMAND ${CMAKE_COMMAND} -E copy "${DEPTHAI_DEVICE_RVC4_VENDOR_LEGAL_DIR}/legal/DEPTHAI-EULA.txt" "${DEPTHAI_DEVICE_BINARY_DIR}/legal/DEPTHAI-EULA.txt"
    COMMAND ${CMAKE_COMMAND} -E copy "${DEPTHAI_DEVICE_KB_CORE_PATH}/notices/dynamic_calibration-LICENSE" "${DEPTHAI_DEVICE_BINARY_DIR}/legal/dynamic_calibration-LICENSE"
    COMMAND ${CMAKE_COMMAND} -E tar cfJ "$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/depthai-device-rvc4-fwp.tar.xz"
        "${DEPTHAI_DEVICE_BINARY_DIR}/$<TARGET_FILE_NAME:${TARGET_DEVICE_EXECUTABLE}>"
        ${DEPTHAI_DEVICE_DSP_SKELS_IN_FWP}
        "${DEPTHAI_DEVICE_SEEK_EAGLE_DIR}"
        "${DEPTHAI_DEVICE_BINARY_DIR}/libdynamic_calibration.so"
        ${DEPTHAI_MODELS_KB_RESOURCE_LIST}
        ${DEPTHAI_CUSTOM_SCRIPTS_OUTPUT_DIR}
        ${DEPTHAI_DEVICE_FWP_LEGAL_FILES}
    COMMAND /usr/bin/python3 "${DEPTHAI_DEVICE_SOURCE_DIR}/utility_scripts/generate_license.py"
        --repo-root "${DEPTHAI_DEVICE_SOURCE_DIR}"
        --product "depthai-device-rvc4-fwp"
        --version "${DEPTHAI_DEVICE_KB_FINAL_VERSION}"
        --commit "${BUILD_COMMIT}"
        --fwp-archive "$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/depthai-device-rvc4-fwp.tar.xz"
        --triplet "${VCPKG_TARGET_TRIPLET}"
        --vcpkg-root "${DEPTHAI_DEVICE_VCPKG_ROOT}"
        --install-root "${CMAKE_BINARY_DIR}/vcpkg_installed"
        --license-input "${DEPTHAI_DEVICE_SOURCE_DIR}/LICENSE.fwp"
        ${DEPTHAI_DEVICE_LICENSE_PRIVATE_MODULE_ARGS}
        --license-output "$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/depthai-device-rvc4-fwp-LICENSE"
        --spdx-output "$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/depthai-device-rvc4-fwp.spdx.json"
        --cyclonedx-output "$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/depthai-device-rvc4-fwp.cdx.json"
    COMMAND_EXPAND_LISTS
)

# #######################
# Build configuration
# #######################
# Add year information
string(TIMESTAMP BUILD_DATETIME "%Y-%m-%d %H:%M:%S +0000" UTC)
message(STATUS "BUILD_DATETIME: ${BUILD_DATETIME}, DEPTHAI_DEVICE_KB_COMMIT_HASH: ${DEPTHAI_DEVICE_KB_COMMIT_HASH}, BUILD_COMMIT_DATETIME: ${BUILD_COMMIT_DATETIME}")
