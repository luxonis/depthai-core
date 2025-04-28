option(DEPTHAI_ENABLE_LIBUSB "Enable usage of libusb and interaction with USB devices" OFF)
option(DEPTHAI_ENABLE_APRIL_TAG "Enable AprilTag node (only available for Windows)" OFF)
option(DEPTHAI_RTABMAP_SUPPORT "Enable optional RTABMap support" OFF)
option(DEPTHAI_BASALT_SUPPORT "Enable optional Basalt support" OFF)
option(DEPTHAI_ENABLE_PROTOBUF "Enable Protobuf support" OFF)
option(DEPTHAI_BUILD_PYTHON "Build python bindings" OFF)
option(DEPTHAI_BUILD_TESTS "Build tests" OFF)
option(DEPTHAI_OPENCV_SUPPORT "Enable optional OpenCV support" OFF)
OPTION(DEPTHAI_ENABLE_KOMPUTE "Enable Kompute support" OFF)
option(DEPTHAI_PCL_SUPPORT "Enable optional PCL support" OFF)
option(DEPTHAI_BOOTSTRAP_VCPKG "Automatically bootstrap VCPKG" ON)
option(DEPTHAI_MERGED_TARGET "Enable merged target build" OFF)
option(DEPTHAI_NEW_FIND_PYTHON "Use new FindPython module" ON)
option(DEPTHAI_SUBMODULE_BUILD "Enable if building depthai as a submodule" OFF)
option(DEPTHAI_BUILD_ZOO_HELPER "Build the Zoo helper" OFF)
option(DEPTHAI_ENABLE_MP4V2 "Enable video recording using the MP4V2 library" OFF)
option(DEPTHAI_XTENSOR_SUPPORT "Enable optional xtensor support" OFF)

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
    option(DEPTHAI_ENABLE_BACKWARD "Enable stacktrace printing on crash using Backward" OFF)
endif()

# Check if on 32 bit linux - default without CURL support
if(CMAKE_SIZEOF_VOID_P EQUAL 4 AND UNIX)
    set(DEPTHAI_DEFAULT_CURL_SUPPORT OFF)
else()
    set(DEPTHAI_DEFAULT_CURL_SUPPORT OFF)
endif()

option(DEPTHAI_ENABLE_CURL "Enable CURL support" ${DEPTHAI_DEFAULT_CURL_SUPPORT})


if(DEPTHAI_ENABLE_PROTOBUF)
    option(DEPTHAI_ENABLE_REMOTE_CONNECTION "Enable Remote Connection support" ON)
    if(DEPTHAI_ENABLE_CURL)
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

