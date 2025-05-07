if(CONFIG_MODE)
    set(_DEPTHAI_PREFIX_PATH_ORIGINAL ${CMAKE_PREFIX_PATH})
    set(_DEPTHAI_MODULE_PATH_ORIGINAL ${CMAKE_MODULE_PATH})
    set(_DEPTHAI_FIND_ROOT_PATH_MODE_PACKAGE_ORIGINAL ${CMAKE_FIND_ROOT_PATH_MODE_PACKAGE})
    # Fixes Android NDK build, where prefix path is ignored as its not inside sysroot
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE "BOTH")
    # Sets where to search for packages about to follow
    set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/${_IMPORT_PREFIX}" ${CMAKE_PREFIX_PATH})
    set(_QUIET "QUIET")
else()
    set(DEPTHAI_SHARED_LIBS ${BUILD_SHARED_LIBS})
endif()
# If library was build as static, find all dependencies
if(NOT CONFIG_MODE OR (CONFIG_MODE AND NOT DEPTHAI_SHARED_LIBS))

    if(DEPTHAI_BUILD_PYTHON)
        # Some dependencies (xtensor) still use the old FindPythonInterp and FindPythonLibs
        if(PYTHON_EXECUTABLE AND NOT Python_EXECUTABLE)
            set(Python_EXECUTABLE ${PYTHON_EXECUTABLE})
        endif()
        if(DEPTHAI_NEW_FIND_PYTHON)
            if(POLICY CMP0094)
                cmake_policy(SET CMP0094 NEW)
            endif()
            set(Python_FIND_UNVERSIONED_NAMES FIRST)
        endif()

        if(DEPTHAI_PYTHON_EMBEDDED_MODULE)
            find_package(Python COMPONENTS Interpreter Development.Embed REQUIRED)
        else()
            find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
        endif()
        find_package(pybind11 CONFIG REQUIRED)
        # Print out the pybind11 version that was found
        message(STATUS "Found pybind11 v${pybind11_VERSION}")
    endif()
    # BZip2 (for bspatch)
    find_package(BZip2 ${_QUIET}  REQUIRED)

    find_package(lz4 CONFIG REQUIRED)
    # FP16 for conversions
    find_path(FP16_INCLUDE_DIR NAMES fp16.h)

    find_package(PNG  REQUIRED)

    if(DEPTHAI_ENABLE_KOMPUTE)
        find_package(kompute ${_QUIET} CONFIG REQUIRED)
    endif()
    # libarchive for firmware packages
    find_package(LibArchive ${_QUIET}  REQUIRED)
    find_package(liblzma ${_QUIET} CONFIG REQUIRED)
    # httplib for Gate communication
    find_package(httplib ${_QUIET} CONFIG REQUIRED)
    # ZLIB for compressing Apps
    find_package(ZLIB REQUIRED)

    # spdlog for library and device logging
    find_package(spdlog ${_QUIET} CONFIG REQUIRED)

    find_package(OpenSSL REQUIRED)
    # Log collection dependencies
    if(DEPTHAI_ENABLE_CURL)
        find_package(CURL ${_QUIET} CONFIG REQUIRED)
        find_package(cpr ${_QUIET} CONFIG REQUIRED)
    endif()

    # Backward
    if(DEPTHAI_ENABLE_BACKWARD)
        # Disable automatic check for additional stack unwinding libraries
        # Just use the default compiler one
        set(STACK_DETAILS_AUTO_DETECT FALSE CACHE BOOL "Auto detect backward's stack details dependencies")
        find_package(Backward ${_QUIET} CONFIG REQUIRED)
        unset(STACK_DETAILS_AUTO_DETECT)
    endif()
    find_package(yaml-cpp ${_QUIET} CONFIG REQUIRED)
    find_package(semver ${_QUIET} CONFIG REQUIRED)
    if(DEPTHAI_HAS_APRIL_TAG)
        find_package(apriltag ${_QUIET} CONFIG REQUIRED)
    endif()
    find_package(magic_enum ${_QUIET} CONFIG REQUIRED)
endif()

# Xtensor
if(DEPTHAI_XTENSOR_SUPPORT)
    find_package(xtl ${_QUIET} CONFIG REQUIRED)
    find_package(xtensor ${_QUIET} CONFIG REQUIRED)
endif()

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    get_filename_component(PARENT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)
    add_subdirectory("${PARENT_DIRECTORY}/3rdparty/foxglove/ws-protocol/cpp/foxglove-websocket" foxglove-websocket)
endif()

# Add threads (c++)
find_package(Threads ${_QUIET} REQUIRED)

# Nlohmann JSON
include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
# Json is a public dependancy, so it has to be installed
set(JSON_Install ON CACHE BOOL "Install nlohmann_json" FORCE)

FetchContent_MakeAvailable(nlohmann_json)

list(APPEND targets_to_export nlohmann_json)

# libnop for serialization
FetchContent_Declare(
    libnop
    GIT_REPOSITORY https://github.com/luxonis/libnop.git
    GIT_TAG        ab842f51dc2eb13916dc98417c2186b78320ed10
    #FIND_PACKAGE_ARGS
)


FetchContent_MakeAvailable(libnop)

# 1. Ask CMake what include dirs libnop exports
get_target_property(_nop_inc libnop INTERFACE_INCLUDE_DIRECTORIES)

# 2. Tell the compiler those dirs are "system" dirs
set_target_properties(libnop PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_nop_inc}"
)

list(APPEND targets_to_export libnop)

if(DEPTHAI_ENABLE_MP4V2)
    # MP4V2 for video encoding
    find_package(mp4v2 ${_QUIET} CONFIG REQUIRED)
endif()

if(DEPTHAI_ENABLE_PROTOBUF)
    find_package(Protobuf ${_QUIET} REQUIRED)
endif()

if(DEPTHAI_BUILD_TESTS)
    find_package(Catch2 CONFIG REQUIRED)
endif()

# XLink
set(_BUILD_SHARED_LIBS_SAVED "${BUILD_SHARED_LIBS}")
set(BUILD_SHARED_LIBS OFF)
if(DEPTHAI_XLINK_LOCAL AND (NOT CONFIG_MODE))
    add_subdirectory("${DEPTHAI_XLINK_LOCAL}" ${CMAKE_CURRENT_BINARY_DIR}/XLink)
else()
    # TODO(themarpe) - might be required
    # elseif(NOT DEPTHAI_XLINK_LOCAL)
    FetchContent_Declare(
        XLink
        GIT_REPOSITORY https://github.com/luxonis/XLink.git
        GIT_TAG        84d17d2d8586c894c77c0205388ebccf579d13cc
    )

    set(XLINK_ENABLE_LIBUSB OFF CACHE BOOL "Enable libusb" FORCE)

    FetchContent_MakeAvailable(
        XLink
    )
endif()
set(BUILD_SHARED_LIBS "${_BUILD_SHARED_LIBS_SAVED}")
unset(_BUILD_SHARED_LIBS_SAVED)
list(APPEND targets_to_export XLink XLinkPublic)

# OpenCV 4 - (optional)
message("DEPTHAI_OPENCV_SUPPORT: ${DEPTHAI_OPENCV_SUPPORT}")
if(DEPTHAI_OPENCV_SUPPORT)
    find_package(OpenCV 4 ${_QUIET} CONFIG REQUIRED)
endif()

if(DEPTHAI_PCL_SUPPORT AND NOT TARGET JsonCpp::JsonCpp)
    find_package(jsoncpp)
endif()


if(DEPTHAI_PCL_SUPPORT)
    find_package(PCL CONFIG COMPONENTS common)
endif()
if(DEPTHAI_RTABMAP_SUPPORT)
    find_package(RTABMap ${_QUIET} CONFIG REQUIRED COMPONENTS core utilite)
    find_package(g2o ${_QUIET} CONFIG REQUIRED)
    find_package(Ceres ${_QUIET} CONFIG REQUIRED)
endif()

if(DEPTHAI_BASALT_SUPPORT)
    find_package(basalt-headers ${_QUIET} CONFIG REQUIRED)
    find_package(basalt_sdk ${_QUIET} CONFIG REQUIRED)
endif()

# include optional dependency cmake
if(DEPTHAI_DEPENDENCY_INCLUDE)
    include(${DEPTHAI_DEPENDENCY_INCLUDE} OPTIONAL)
endif()

# Cleanup
if(CONFIG_MODE)
    set(CMAKE_PREFIX_PATH ${_DEPTHAI_PREFIX_PATH_ORIGINAL})
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ${_DEPTHAI_FIND_ROOT_PATH_MODE_PACKAGE_ORIGINAL})
    unset(_DEPTHAI_PREFIX_PATH_ORIGINAL)
    unset(_DEPTHAI_FIND_ROOT_PATH_MODE_PACKAGE_ORIGINAL)
    unset(_QUIET)
else()
    set(DEPTHAI_SHARED_LIBS)
endif()
