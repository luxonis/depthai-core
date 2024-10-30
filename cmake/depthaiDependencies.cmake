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

    # BZip2 (for bspatch)
    find_package(BZip2 ${_QUIET}  REQUIRED)

    find_package(lz4 CONFIG REQUIRED)
    # FP16 for conversions
    find_package(FP16 ${_QUIET} CONFIG REQUIRED)

find_package(PNG  REQUIRED)
    # libarchive for firmware packages
    find_package(archive_static ${_QUIET} CONFIG REQUIRED)
    find_package(liblzma ${_QUIET} CONFIG REQUIRED)
    # httplib for Gate communication
    find_package(httplib ${_QUIET} CONFIG REQUIRED)
    # ZLIB for compressing Apps
    find_package(ZLIB CONFIG REQUIRED)

    # spdlog for library and device logging
    find_package(spdlog ${_QUIET} CONFIG REQUIRED)

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
endif()

# Xtensor
get_filename_component(PARENT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)
if(DEPTHAI_XTENSOR_SUPPORT)
    add_subdirectory("${PARENT_DIRECTORY}/3rdparty/xtl" xtl)
    add_subdirectory("${PARENT_DIRECTORY}/3rdparty/xtensor" xtensor)
endif()

add_subdirectory("${PARENT_DIRECTORY}/3rdparty/foxglove/ws-protocol/cpp/foxglove-websocket" foxglove-websocket)

# Add threads (c++)
find_package(Threads ${_QUIET} REQUIRED)

# Nlohmann JSON
find_package(nlohmann_json 3.6.0 ${_QUIET} CONFIG REQUIRED)

# libnop for serialization
find_package(libnop ${_QUIET} CONFIG REQUIRED)

# MP4V2 for video encoding
find_package(mp4v2 ${_QUIET} CONFIG REQUIRED)

# TODO(Morato) - potentially move this to a private library
find_package(Protobuf ${_QUIET} CONFIG REQUIRED)

# XLink
if(DEPTHAI_XLINK_LOCAL AND (NOT CONFIG_MODE))
    set(_BUILD_SHARED_LIBS_SAVED "${BUILD_SHARED_LIBS}")
    set(BUILD_SHARED_LIBS OFF)
    add_subdirectory("${DEPTHAI_XLINK_LOCAL}" ${CMAKE_CURRENT_BINARY_DIR}/XLink)
    set(BUILD_SHARED_LIBS "${_BUILD_SHARED_LIBS_SAVED}")
    unset(_BUILD_SHARED_LIBS_SAVED)
    list(APPEND targets_to_export XLink XLinkPublic)
else()
    # TODO(themarpe) - might be required
    # elseif(NOT DEPTHAI_XLINK_LOCAL)
    find_package(XLink ${_QUIET} CONFIG REQUIRED HINTS "${CMAKE_CURRENT_LIST_DIR}/XLink" "${CMAKE_CURRENT_LIST_DIR}/../XLink")
endif()

# OpenCV 4 - (optional)

if(DEPTHAI_PCL_SUPPORT AND NOT TARGET JsonCpp::JsonCpp)
    find_package(jsoncpp)
endif()

set(MODULE_TEMP ${CMAKE_MODULE_PATH})
set(PREFIX_TEMP ${CMAKE_PREFIX_PATH})
set(CMAKE_MODULE_PATH ${_DEPTHAI_MODULE_PATH_ORIGINAL})
set(CMAKE_PREFIX_PATH ${_DEPTHAI_PREFIX_PATH_ORIGINAL})
set(EIGEN_ROOT_TEMP ${EIGEN_ROOT})
set(EIGEN_ROOT "")
set(CMAKE_FIND_ROOT_PATH_TMP ${CMAKE_FIND_ROOT_PATH})
set(CMAKE_FIND_ROOT_PATH "")

message("DEPTHAI_OPENCV_SUPPORT: ${DEPTHAI_OPENCV_SUPPORT}")
if(DEPTHAI_OPENCV_SUPPORT)
    find_package(OpenCV 4 ${_QUIET} CONFIG REQUIRED)
endif()
if(DEPTHAI_PCL_SUPPORT)
    find_package(PCL CONFIG COMPONENTS common)
endif()
set(CMAKE_FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH_TMP})
if(DEPTHAI_RTABMAP_SUPPORT)
    find_package(RTABMap ${_QUIET} CONFIG REQUIRED COMPONENTS core utilite)
endif()

set(CMAKE_MODULE_PATH ${MODULE_TEMP})
set(CMAKE_PREFIX_PATH ${PREFIX_TEMP})
set(EIGEN_ROOT ${EIGEN_ROOT_TEMP})

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
