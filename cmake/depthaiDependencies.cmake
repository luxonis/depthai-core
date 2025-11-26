include(FetchContent)
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
    find_package(Eigen3 ${_QUIET} CONFIG REQUIRED)

    # spdlog for library and device logging
    find_package(spdlog ${_QUIET} CONFIG REQUIRED)
    find_package(fmt ${_QUIET} CONFIG REQUIRED)

    find_package(OpenSSL REQUIRED)
    # Log collection dependencies
    if(DEPTHAI_ENABLE_CURL)
        find_package(CURL ${_QUIET} CONFIG REQUIRED)
        find_package(cpr ${_QUIET} CONFIG REQUIRED)
    endif()
    if(DEPTHAI_BASALT_SUPPORT)
        find_package(basalt-headers ${_QUIET} CONFIG REQUIRED)
        find_package(basalt_sdk ${_QUIET} CONFIG REQUIRED)
    endif()
    if(DEPTHAI_RTABMAP_SUPPORT)
        find_package(g2o ${_QUIET} CONFIG REQUIRED)
        find_package(Ceres ${_QUIET} CONFIG REQUIRED)
        find_package(PCL CONFIG COMPONENTS common)
        find_package(RTABMap ${_QUIET} CONFIG REQUIRED COMPONENTS core utilite)
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
    if(NOT DEPTHAI_XTENSOR_EXTERNAL)
        FetchContent_Declare(
            xtl
            GIT_REPOSITORY https://github.com/luxonis/xtl.git
            GIT_TAG        2da8e13ef3d7d9d6ccae3fd68f07892160e8b6c6
            GIT_SHALLOW    TRUE
        )

        FetchContent_Declare(
            xtensor
            GIT_REPOSITORY https://github.com/luxonis/xtensor.git
            GIT_TAG        d070cfcba9418ec45e5399708100eee1181a9573
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(xtl xtensor)
        get_target_property(_xtensor_inc xtensor INTERFACE_INCLUDE_DIRECTORIES)
        set_target_properties(xtensor PROPERTIES
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_xtensor_inc}"
        )
    else()
        find_package(xtensor ${_QUIET} CONFIG REQUIRED)
    endif()
endif()

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    get_filename_component(PARENT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)
    add_subdirectory("${PARENT_DIRECTORY}/3rdparty/foxglove/ws-protocol/cpp/foxglove-websocket" foxglove-websocket)
endif()

# Add threads (c++)
find_package(Threads ${_QUIET} REQUIRED)

if(NOT DEPTHAI_JSON_EXTERNAL)
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.12.0
    )
    # Json is a public dependancy, so it has to be installed
    set(JSON_Install ON CACHE BOOL "Install nlohmann_json" FORCE)

    FetchContent_MakeAvailable(nlohmann_json)
    list(APPEND targets_to_export nlohmann_json)
else()
    find_package(nlohmann_json CONFIG REQUIRED)
endif()

if(NOT DEPTHAI_LIBNOP_EXTERNAL)
    FetchContent_Declare(
        libnop
        GIT_REPOSITORY https://github.com/luxonis/libnop.git
        GIT_TAG        ab842f51dc2eb13916dc98417c2186b78320ed10
    )

    FetchContent_MakeAvailable(libnop)

    # Thread libnop in all cases as a system include, to avoid many warnings from it
    get_target_property(_nop_inc libnop INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(libnop PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_nop_inc}"
    )

    list(APPEND targets_to_export libnop)
else()
    find_package(libnop CONFIG REQUIRED)
endif()

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
# Always compile XLink as static library, even when DepthAI is built as shared
set(_BUILD_SHARED_LIBS_SAVED "${BUILD_SHARED_LIBS}")
set(BUILD_SHARED_LIBS OFF)
set(XLINK_ENABLE_LIBUSB ${DEPTHAI_ENABLE_LIBUSB} CACHE BOOL "Enable libusb" FORCE)
set(XLINK_INSTALL_PUBLIC_ONLY ON CACHE BOOL "Install only public headers" FORCE)
if(DEPTHAI_ENABLE_LIBUSB)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(libusb REQUIRED libusb-1.0)
endif()
set(XLINK_LIBUSB_SYSTEM ON)
if(DEPTHAI_XLINK_LOCAL AND (NOT CONFIG_MODE))
    add_subdirectory("${DEPTHAI_XLINK_LOCAL}" ${CMAKE_CURRENT_BINARY_DIR}/XLink)
else()
    FetchContent_Declare(
        XLink
        GIT_REPOSITORY https://github.com/luxonis/XLink.git
        GIT_TAG        ffe0f85a0d0cdfd89cfef90611eb53af2748ea11
    )

    FetchContent_MakeAvailable(
        XLink
    )
endif()
set(BUILD_SHARED_LIBS "${_BUILD_SHARED_LIBS_SAVED}")
unset(_BUILD_SHARED_LIBS_SAVED)
list(APPEND targets_to_export XLinkPublic)

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


# include optional dependency cmake
if(DEPTHAI_DEPENDENCY_INCLUDE)
    include(${DEPTHAI_DEPENDENCY_INCLUDE} OPTIONAL)
endif()

if(DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT)
    set(DEPTHAI_DYNAMIC_CALIBRATION_PATH "" CACHE FILEPATH "Override path to local dynamic_calibration .zip file")

    if(DEPTHAI_DYNAMIC_CALIBRATION_PATH AND EXISTS "${DEPTHAI_DYNAMIC_CALIBRATION_PATH}")
        message(STATUS "Using local dynamic_calibration zip: ${DEPTHAI_DYNAMIC_CALIBRATION_PATH}")
        FetchContent_Declare(
            dynamic_calibration
            URL "file://${DEPTHAI_DYNAMIC_CALIBRATION_PATH}"
        )
    else()
        include(Depthai/DepthaiDynamicCalibrationConfig)
        include(PlatformParsing)
        detect_platform_arch(DEPTHAI_HOST_PLATFORM_ARCH)
        message(STATUS "Platform architecture: ${DEPTHAI_HOST_PLATFORM_ARCH}")
        # TODO - Add URL_HASH
        message(STATUS "Using remote dynamic_calibration zip")
        FetchContent_Declare(
            dynamic_calibration
            URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-helper-binaries/dynamic_calibration/${DEPTHAI_DYNAMIC_CALIBRATION_VERSION}/dynamic_calibration_${DEPTHAI_DYNAMIC_CALIBRATION_VERSION}_${DEPTHAI_HOST_PLATFORM_ARCH}.zip"
        )
    endif()

    FetchContent_MakeAvailable(dynamic_calibration)
    message(STATUS "Dynamic calibration extracted to ${dynamic_calibration_SOURCE_DIR}")
    set(DYNAMIC_CALIBRATION_DIR ${dynamic_calibration_SOURCE_DIR})

    if(WIN32)
        # On Windows, find both .lib (import libraries) and .dll (runtime libraries)
        # Search for the import libraries (.lib files)
        find_library(DYNAMIC_CALIBRATION_RELEASE_LIB dynamic_calibration
            PATHS ${DYNAMIC_CALIBRATION_DIR}/lib
            NO_DEFAULT_PATH
        )
        find_library(DYNAMIC_CALIBRATION_DEBUG_LIB dynamic_calibrationd
            PATHS ${DYNAMIC_CALIBRATION_DIR}/lib
            NO_DEFAULT_PATH
        )

        # Search for the runtime libraries (.dll files)
        find_file(DYNAMIC_CALIBRATION_RELEASE_DLL dynamic_calibration.dll
            PATHS ${DYNAMIC_CALIBRATION_DIR}/bin ${DYNAMIC_CALIBRATION_DIR}/lib
            NO_DEFAULT_PATH
        )
        find_file(DYNAMIC_CALIBRATION_DEBUG_DLL dynamic_calibrationd.dll
            PATHS ${DYNAMIC_CALIBRATION_DIR}/bin ${DYNAMIC_CALIBRATION_DIR}/lib
            NO_DEFAULT_PATH
        )

        if(NOT DYNAMIC_CALIBRATION_DEBUG_LIB OR NOT DYNAMIC_CALIBRATION_DEBUG_DLL)
            message(FATAL_ERROR "Dynamic Calibration debug library (.lib) or runtime (.dll) not found, disable support by setting DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT to OFF")
        endif()
        if(NOT DYNAMIC_CALIBRATION_RELEASE_LIB OR NOT DYNAMIC_CALIBRATION_RELEASE_DLL)
            message(FATAL_ERROR "Dynamic Calibration release library (.lib) or runtime (.dll) not found, disable support by setting DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT to OFF")
        endif()
    else()
        # On non-Windows platforms, search for the release version of the library
        find_library(DYNAMIC_CALIBRATION_RELEASE dynamic_calibration
            PATHS ${DYNAMIC_CALIBRATION_DIR}/lib
            NO_DEFAULT_PATH
        )
        if(NOT DYNAMIC_CALIBRATION_RELEASE)
            message(FATAL_ERROR "Dynamic Calibration library not found, disable support by setting DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT to OFF")
        endif()
    endif()

    # Set up legacy variables for backward compatibility
    if(WIN32)
        # Set a generator expression to select the right one based on build type (import libraries)
        set(DYNAMIC_CALIBRATION_LIB
            $<$<CONFIG:Debug>:${DYNAMIC_CALIBRATION_DEBUG_LIB}>
            $<$<CONFIG:Release>:${DYNAMIC_CALIBRATION_RELEASE_LIB}>
            $<$<CONFIG:RelWithDebInfo>::${DYNAMIC_CALIBRATION_RELEASE_LIB}>
            $<$<CONFIG:MinSizeRel>:${DYNAMIC_CALIBRATION_RELEASE_LIB}>
        )
    else()
        set(DYNAMIC_CALIBRATION_LIB ${DYNAMIC_CALIBRATION_RELEASE})
    endif()

    # Create imported target for runtime dependencies
    add_library(dynamic_calibration_imported SHARED IMPORTED)
    if(WIN32)
        set_target_properties(dynamic_calibration_imported PROPERTIES
            IMPORTED_LOCATION_DEBUG "${DYNAMIC_CALIBRATION_DEBUG_DLL}"
            IMPORTED_LOCATION_RELEASE "${DYNAMIC_CALIBRATION_RELEASE_DLL}"
            IMPORTED_IMPLIB_DEBUG "${DYNAMIC_CALIBRATION_DEBUG_LIB}"
            IMPORTED_IMPLIB_RELEASE "${DYNAMIC_CALIBRATION_RELEASE_LIB}"
            IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
            INTERFACE_INCLUDE_DIRECTORIES "${DYNAMIC_CALIBRATION_DIR}/include"
        )
    else()
        # On non-Windows platforms, use the release library for all configurations
        set_target_properties(dynamic_calibration_imported PROPERTIES
            IMPORTED_LOCATION_DEBUG "${DYNAMIC_CALIBRATION_RELEASE}"
            IMPORTED_LOCATION_RELEASE "${DYNAMIC_CALIBRATION_RELEASE}"
            IMPORTED_LOCATION_RELWITHDEBINFO "${DYNAMIC_CALIBRATION_RELEASE}"
            IMPORTED_LOCATION_MINSIZEREL "${DYNAMIC_CALIBRATION_RELEASE}"
            IMPORTED_CONFIGURATIONS "DEBUG;RELEASE;RELWITHDEBINFO;MINSIZEREL"
            INTERFACE_INCLUDE_DIRECTORIES "${DYNAMIC_CALIBRATION_DIR}/include"
        )
    endif()
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
