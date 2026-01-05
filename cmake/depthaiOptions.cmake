# ==================================
# DepthAI Build Options (organized)
# ==================================

# ---------- Platform‑based defaults ----------

# Detect 32‑bit Linux for default CURL support
if(CMAKE_SIZEOF_VOID_P EQUAL 4 AND UNIX)
    set(DEPTHAI_DEFAULT_CURL_SUPPORT OFF)
else()
    set(DEPTHAI_DEFAULT_CURL_SUPPORT ON)
endif()

# ---------- Core Feature Toggles (private) -------------
option(DEPTHAI_ENABLE_LIBUSB "Enable usage of libusb and interaction with USB devices" ON)
option(DEPTHAI_ENABLE_APRIL_TAG "Enable AprilTag node (not available for Windows)" ON)
option(DEPTHAI_ENABLE_PROTOBUF "Enable Protobuf support" ON)
option(DEPTHAI_ENABLE_CURL "Enable CURL support" ${DEPTHAI_DEFAULT_CURL_SUPPORT})
option(DEPTHAI_ENABLE_KOMPUTE "Enable Kompute support" OFF)
option(DEPTHAI_ENABLE_MP4V2 "Enable video recording using the MP4V2 library" ON)
option(DEPTHAI_FETCH_ARTIFACTS "Enable fetching artifacts from remote repository" ON)

# ---------- Optional Features (public) -------------
option(DEPTHAI_OPENCV_SUPPORT "Enable optional OpenCV support" ON)
option(DEPTHAI_XTENSOR_SUPPORT "Enable optional xtensor support" ON)
option(DEPTHAI_PCL_SUPPORT "Enable optional PCL support" OFF)

option(DEPTHAI_RTABMAP_SUPPORT "Enable optional RTABMap support" OFF)
option(DEPTHAI_BASALT_SUPPORT "Enable optional Basalt support" OFF)

option(DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT "Enable Dynamic Calibration support" ON)

# Build Behaviour
option(DEPTHAI_MERGED_TARGET "Enable merged target build" ON)
option(DEPTHAI_BUILD_PYTHON "Build python bindings" OFF)
option(DEPTHAI_BUILD_TESTS "Build tests" OFF)
option(DEPTHAI_BUILD_EXAMPLES "Build examples - Requires OpenCV library to be installed" OFF)
option(DEPTHAI_BUILD_DOCS "Build documentation - requires doxygen to be installed" OFF)
option(DEPTHAI_BUILD_ZOO_HELPER "Build the Zoo helper" OFF)
option(DEPTHAI_BUILD_EXT_HOST_NODES "Build the contrib host nodes lib" OFF)
option(DEPTHAI_NEW_FIND_PYTHON "Use new FindPython module" ON)
option(DEPTHAI_INSTALL "Enable install target for depthai-core targets" ON)

# ---------- Dependency Management -------------
option(DEPTHAI_BOOTSTRAP_VCPKG "Automatically bootstrap VCPKG" ON)
option(DEPTHAI_VCPKG_INTERNAL_ONLY "Use VCPKG internally, but not for interface libraries" ON)

set(USE_EXTERNAL_INTERFACE_LIBS_DEFAULT ON)
if(DEPTHAI_VCPKG_INTERNAL_ONLY)
    set(USE_EXTERNAL_INTERFACE_LIBS_DEFAULT OFF)
endif()

option(DEPTHAI_JSON_EXTERNAL "Use external nlohmann_json library" ${USE_EXTERNAL_INTERFACE_LIBS_DEFAULT})
option(DEPTHAI_LIBNOP_EXTERNAL "Use external libnop library" ${USE_EXTERNAL_INTERFACE_LIBS_DEFAULT})
option(DEPTHAI_XTENSOR_EXTERNAL "Use external xtensor library" ${USE_EXTERNAL_INTERFACE_LIBS_DEFAULT})

option(DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT "Enable Dynamic Calibration support" ${DEPTHAI_DEFAULT_DYNAMIC_CALIBRATION_SUPPORT})

# ---------- Platform / Compiler Tweaks ---------
if(CMAKE_SIZEOF_VOID_P EQUAL 4 AND DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT)
    # There is not 32b build of Dynamic Calibration Library
    message(FATAL_ERROR "Dynamic calibration is not supported on 32b machines. Build with DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=OFF")
endif()

# AprilTag node support
set(DEPTHAI_HAS_APRIL_TAG ${DEPTHAI_ENABLE_APRIL_TAG})
if(WIN32)
    message(STATUS "AprilTag node is not supported on Windows")
    set(DEPTHAI_HAS_APRIL_TAG OFF)
endif()

# Disable merged target when OpenCV is disabled
if(NOT DEPTHAI_OPENCV_SUPPORT)
    set(DEPTHAI_MERGED_TARGET OFF CACHE BOOL "Enable merged target build" FORCE)
endif()

# Backward stacktrace printing
if(ANDROID OR EMSCRIPTEN)
    # Backward not supported currently on Android
    set(DEPTHAI_ENABLE_BACKWARD OFF CACHE BOOL "" FORCE)
else()
    option(DEPTHAI_ENABLE_BACKWARD "Enable stacktrace printing on crash using Backward" ON)
endif()

if(WIN32)
    set(DEPTHAI_DEFAULT_EVENTS_MANAGER_SUPPORT OFF) # TODO (Morato) turn on by default again once the number of symbols is reduced
else()
    set(DEPTHAI_DEFAULT_EVENTS_MANAGER_SUPPORT ON)
endif()

# ---------- Remote connection options
if(DEPTHAI_ENABLE_PROTOBUF)
    option(DEPTHAI_ENABLE_REMOTE_CONNECTION "Enable Remote Connection support" ON)
    if(DEPTHAI_ENABLE_CURL AND DEPTHAI_OPENCV_SUPPORT)
        option(DEPTHAI_ENABLE_EVENTS_MANAGER "Enable Events Manager" ${DEPTHAI_DEFAULT_EVENTS_MANAGER_SUPPORT})
    else()
        message(STATUS "Events Manager disabled because Protobuf or curl or opencv support is disabled.")
        option(DEPTHAI_ENABLE_EVENTS_MANAGER "Disable Events Manager" OFF)
    endif()
else()
    option(DEPTHAI_ENABLE_REMOTE_CONNECTION "Enable Remote Connection support" OFF)
    message(STATUS "Remote Connection support disabled because Protobuf support is disabled.")
endif()

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    option(DEPTHAI_EMBED_FRONTEND "Embed frontend resources into library" ON)
else()
    option(DEPTHAI_EMBED_FRONTEND "Embed frontend resources into library" OFF)
endif()

# ---------- Embedded firmware  ---------------------------
option(DEPTHAI_ENABLE_DEVICE_FW "Enable MyriadX Device FW" ON)
option(DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW "Enable MyriadX Device Bootloader FW" ON)
option(DEPTHAI_ENABLE_DEVICE_RVC3_FW "Enable RVC3 Device FW" OFF)
option(DEPTHAI_ENABLE_DEVICE_RVC4_FW "Enable RVC4 Device FW" ON)
option(DEPTHAI_BINARIES_RESOURCE_COMPILE "Compile Depthai device side binaries into library" ON)

# ---------- Development Aids -------------------
option(DEPTHAI_CLANG_FORMAT "Enable clang-format target" ON)
option(DEPTHAI_CLANG_TIDY "Enable clang-tidy checks during compilation" OFF)
option(DEPTHAI_SANITIZE "Enable Address and Undefined sanitizers for library, examples and tests" OFF)

# Local override paths
set(DEPTHAI_XLINK_LOCAL "" CACHE STRING "Path to local XLink source to use instead of Hunter")
set(DEPTHAI_BOOTLOADER_SHARED_LOCAL "" CACHE STRING "Path to local depthai-bootloader-shared source to use instead of submodule")

# Options for internal use
option(DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4 "This build is part of the rvc4 firmware build" OFF)
