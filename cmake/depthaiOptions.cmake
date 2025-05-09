option(DEPTHAI_ENABLE_LIBUSB "Enable usage of libusb and interaction with USB devices" OFF)
option(DEPTHAI_ENABLE_APRIL_TAG "Enable AprilTag node (not available for Windows)" ON)
option(DEPTHAI_RTABMAP_SUPPORT "Enable optional RTABMap support" OFF)
option(DEPTHAI_BASALT_SUPPORT "Enable optional Basalt support" OFF)
option(DEPTHAI_ENABLE_PROTOBUF "Enable Protobuf support" ON)
option(DEPTHAI_BUILD_PYTHON "Build python bindings" OFF)
option(DEPTHAI_BUILD_TESTS "Build tests" OFF)
option(DEPTHAI_OPENCV_SUPPORT "Enable optional OpenCV support" ON)
OPTION(DEPTHAI_ENABLE_KOMPUTE "Enable Kompute support" OFF)
option(DEPTHAI_PCL_SUPPORT "Enable optional PCL support" OFF)
option(DEPTHAI_MERGED_TARGET "Enable merged target build" ON)
option(DEPTHAI_NEW_FIND_PYTHON "Use new FindPython module" ON)
option(DEPTHAI_BUILD_ZOO_HELPER "Build the Zoo helper" OFF)
option(DEPTHAI_ENABLE_MP4V2 "Enable video recording using the MP4V2 library" ON)
option(DEPTHAI_XTENSOR_SUPPORT "Enable optional xtensor support" ON)

#VCPKG related options
option(DEPTHAI_BOOTSTRAP_VCPKG "Automatically bootstrap VCPKG" ON)
# DepthAI uses VCPKG internally, to fetch its private dependencies, but not for the public interface dependencies by default
# (OpenCV, PCL, etc.)
# This is to avoid conflicts with the system installed libraries when downstream libraries use DepthAI.
option(DEPTHAI_VCPKG_INTERNAL_ONLY "Use VCPKG internally, but not for libraries on the interface" ON)
set(USE_EXTERNAL_INTERFACE_LIBS_DEFAULT ON)
if(DEPTHAI_VCPKG_INTERNAL_ONLY)
    set(USE_EXTERNAL_INTERFACE_LIBS_DEFAULT OFF)
endif()

option(DEPTHAI_JSON_EXTERNAL "Use external nlohmann_json library" ${USE_EXTERNAL_INTERFACE_LIBS_DEFAULT})
option(DEPTHAI_LIBNOP_EXTERNAL "Use external libnop library" ${USE_EXTERNAL_INTERFACE_LIBS_DEFAULT})
option(DEPTHAI_XTENSOR_EXTERNAL "Use external xtensor library" ${USE_EXTERNAL_INTERFACE_LIBS_DEFAULT})

if(NOT DEPTHAI_OPENCV_SUPPORT)
    set(DEPTHAI_MERGED_TARGET OFF CACHE BOOL "Enable merged target build" FORCE)
endif()

set(DEPTHAI_HAS_APRIL_TAG ${DEPTHAI_ENABLE_APRIL_TAG})
if(WIN32)
    message(STATUS "AprilTag node is not supported on Windows")
    set(DEPTHAI_HAS_APRIL_TAG OFF)
endif()

# Enable backward stack printing on crash
if(ANDROID OR EMSCRIPTEN)
    # Backward not supported currently on Android
    set(DEPTHAI_ENABLE_BACKWARD OFF CACHE BOOL "" FORCE)
else()
    option(DEPTHAI_ENABLE_BACKWARD "Enable stacktrace printing on crash using Backward" ON)
endif()

# Check if on 32 bit linux - default without CURL support
if(CMAKE_SIZEOF_VOID_P EQUAL 4 AND UNIX)
    set(DEPTHAI_DEFAULT_CURL_SUPPORT OFF)
else()
    set(DEPTHAI_DEFAULT_CURL_SUPPORT ON)
endif()

option(DEPTHAI_ENABLE_CURL "Enable CURL support" ${DEPTHAI_DEFAULT_CURL_SUPPORT})


if(DEPTHAI_ENABLE_PROTOBUF)
    option(DEPTHAI_ENABLE_REMOTE_CONNECTION "Enable Remote Connection support" ON)
    if(DEPTHAI_ENABLE_CURL AND DEPTHAI_HAVE_OPENCV_SUPPORT)
        option(DEPTHAI_ENABLE_EVENTS_MANAGER "Enable Events Manager" ON)
    else()
        message(STATUS "Events Manager disabled because Protobuf & curl support is disabled.")
        option(DEPTHAI_ENABLE_EVENTS_MANAGER "Enable Events Manager" OFF)
    endif()
else()
    option(DEPTHAI_ENABLE_REMOTE_CONNECTION "Enable Remote Connection support" OFF)
    message(STATUS "Remote Connection support disabled because Protobuf support is disabled.")
endif()

# Additional options
option(DEPTHAI_CLANG_FORMAT "Enable clang-format target"                  ON)
option(DEPTHAI_CLANG_TIDY "Enable clang-tidy checks during compilation" OFF)
option(DEPTHAI_SANITIZE "Enable Address and Undefined sanitizers for library, examples and tests" OFF)

# Should install depthai core libraries
option(DEPTHAI_INSTALL   "Enable install target for depthai-core targets" ON)


# Debug option
set(DEPTHAI_XLINK_LOCAL "" CACHE STRING "Path to local XLink source to use instead of Hunter")
set(DEPTHAI_BOOTLOADER_SHARED_LOCAL "" CACHE STRING "Path to local depthai-bootloader-shared source to use instead of submodule")


# First specify options
option(DEPTHAI_BUILD_EXAMPLES "Build examples - Requires OpenCV library to be installed" OFF)
option(DEPTHAI_BUILD_DOCS "Build documentation - requires doxygen to be installed" OFF)

# Specify support for FW
option(DEPTHAI_ENABLE_DEVICE_FW "Enable MyriadX Device FW" ON)
option(DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW "Enable MyriadX Device Bootloader FW" ON)
option(DEPTHAI_ENABLE_DEVICE_RVC3_FW "Enable RVC3 Device FW" OFF)
option(DEPTHAI_ENABLE_DEVICE_RVC4_FW "Enable RVC4 Device FW" ON)

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    option(DEPTHAI_EMBED_FRONTEND "Embed frontend resources into library" ON)
else()
    option(DEPTHAI_EMBED_FRONTEND "Embed frontend resources into library" OFF)
endif()

option(DEPTHAI_BINARIES_RESOURCE_COMPILE "Compile Depthai device side binaries into library" ON)
