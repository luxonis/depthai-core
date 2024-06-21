cmake_minimum_required(VERSION 3.4) # For Hunter

# MSVC variable isn't available before 'project' call
# Generalize to Win32 platform for now
if(NOT WIN32)
    set(HUNTER_CONFIGURATION_TYPES "Release" CACHE STRING "Hunter dependencies list of build configurations")
endif()

# Early options
option(DEPTHAI_ENABLE_LIBUSB "Enable usage of libusb and interaction with USB devices" ON)

# Set to use native tls for windows before including Hunter (used for Curl)
if(WIN32)
    set(DEPTHAI_CURL_USE_SCHANNEL ON)
    set(DEPTHAI_CURL_USE_OPENSSL OFF)
else()
    set(DEPTHAI_CURL_USE_SCHANNEL OFF)
    set(DEPTHAI_CURL_USE_OPENSSL ON)
endif()

# Set type to canonicalize relative paths for user-provided toolchain
set(CMAKE_TOOLCHAIN_FILE "" CACHE FILEPATH "CMake toolchain path")

# Create a custom toolchain to pass certain options to dependencies
set(gen_toolchain "${CMAKE_CURRENT_BINARY_DIR}/generated/toolchain.cmake")

if(EXISTS "${gen_toolchain}" AND ("${_INTERNAL_DEPTHAI_ORIGINAL_CMAKE_TOOLCHAIN_FILE}" STREQUAL "${CMAKE_TOOLCHAIN_FILE}" OR NOT "${CMAKE_TOOLCHAIN_FILE}" STREQUAL ""))
    message(STATUS "Using existing generated toolchain")
else()
    message(STATUS "Generating new toolchain...")
    configure_file(
        "${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain/custom.cmake.in"
        "${gen_toolchain}"
        @ONLY
    )
endif()

set(CMAKE_TOOLCHAIN_FILE "${gen_toolchain}" CACHE STRING "" FORCE)
if(DEFINED _INTERNAL_DEPTHAI_ORIGINAL_CMAKE_TOOLCHAIN_FILE)
    message(STATUS "Using specified toolchain file: ${_INTERNAL_DEPTHAI_ORIGINAL_CMAKE_TOOLCHAIN_FILE} combined into: ${CMAKE_TOOLCHAIN_FILE}")
else()
    message(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
endif()

include("cmake/HunterGate.cmake")
HunterGate(
    URL "https://github.com/cpp-pm/hunter/archive/v0.25.5.tar.gz"
    SHA1 "a20151e4c0740ee7d0f9994476856d813cdead29"
    LOCAL # Local config for dependencies
)

# Move binary dir if windows, to shorten the path
if(WIN32)
    set(HUNTER_BINARY_DIR "${HUNTER_GATE_ROOT}/_bin" CACHE STRING "Hunter binary directory")
endif()

# Create depthai project
project(depthai VERSION "2.25.0" LANGUAGES CXX C)
get_directory_property(has_parent PARENT_DIRECTORY)
if(has_parent)
    set(DEPTHAI_VERSION ${PROJECT_VERSION} PARENT_SCOPE)
endif()

# Set default build type depending on context
set(default_build_type "Release")
if(EXISTS "${CMAKE_SOURCE_DIR}/.git" AND NOT DEFINED ENV{CI})
    set(default_build_type "Debug")
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set default installation directory
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory" FORCE)
endif()

# Set policies
# CMP0074 dictates that find_package searches environment variable "[packageName]_ROOT" along with regular variable [packageName]_ROOT
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW) # Only introduced in 3.12
endif()

if(POLICY CMP0028)
  cmake_policy(SET CMP0028 NEW)
endif()

# Set to export compile commands for tools like clang-tidy and format
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/sanitizers")

# Additional options
option(DEPTHAI_CLANG_FORMAT "Enable clang-format target"                  ON)
option(DEPTHAI_CLANG_TIDY "Enable clang-tidy checks during compilation" OFF)
option(DEPTHAI_SANITIZE "Enable Address and Undefined sanitizers for library, examples and tests" OFF)

# Should install depthai core libraries
option(DEPTHAI_INSTALL   "Enable install target for depthai-core targets" ON)

# Debug option
set(DEPTHAI_XLINK_LOCAL "" CACHE STRING "Path to local XLink source to use instead of Hunter")
set(DEPTHAI_BOOTLOADER_SHARED_LOCAL "" CACHE STRING "Path to local depthai-bootloader-shared source to use instead of submodule")

# Enable backward stack printing on crash
if(ANDROID OR EMSCRIPTEN)
    # Backward not supported currently on Android
    set(DEPTHAI_ENABLE_BACKWARD OFF CACHE BOOL "" FORCE)
else()
    option(DEPTHAI_ENABLE_BACKWARD "Enable stacktrace printing on crash using Backward" ON)
    # Additional function information for 'backward' stacktrace
    if(DEPTHAI_ENABLE_BACKWARD)
        set(CMAKE_ENABLE_EXPORTS ON)
    endif()
endif()

# Check if on 32 bit linux - default without CURL support
if(CMAKE_SIZEOF_VOID_P EQUAL 4 AND UNIX)
    set(DEPTHAI_DEFAULT_CURL_SUPPORT OFF)
else()
    set(DEPTHAI_DEFAULT_CURL_SUPPORT ON)
endif()

option(DEPTHAI_ENABLE_CURL "Enable CURL support" ${DEPTHAI_DEFAULT_CURL_SUPPORT})

# Force Colored output when using Ninja
# Global option - affects all targets
option(FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)" OFF)
if(FORCE_COLORED_OUTPUT)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_options(-fdiagnostics-color=always)
    elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_options(-fcolor-diagnostics)
    endif()
endif()

# Specify exporting all symbols on Windows
if(WIN32 AND BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON CACHE BOOL "")
endif()

### Constants
set(PROJECT_EXPORT_GROUP "${PROJECT_NAME}Targets")

## Check if cloned or sources
find_package(Git)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --show-toplevel
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        RESULT_VARIABLE _git_root_dir_error
        OUTPUT_VARIABLE _git_root_dir
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(DEPTHAI_DOWNLOADED_SOURCES ON)
    if(_git_root_dir_error EQUAL 0 AND "${_git_root_dir}" STREQUAL "${CMAKE_CURRENT_LIST_DIR}")
        set(DEPTHAI_DOWNLOADED_SOURCES OFF)
    endif()
    message(DEBUG "Git root dir (${_git_root_dir_error}): ${_git_root_dir}")
    message(DEBUG "DepthAI as downloaded sources: ${DEPTHAI_DOWNLOADED_SOURCES}")
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
endif()

### Get and find dependencies

# Specify optional dependencies
option(DEPTHAI_XTENSOR_SUPPORT "Enable optional xtensor support" ON)
option(DEPTHAI_OPENCV_SUPPORT "Enable optional OpenCV support" ON)
option(DEPTHAI_PCL_SUPPORT "Enable optional PCL support" OFF)
option(DEPTHAI_RTABMAP_SUPPORT "Enable optional RTABMap support" OFF)
option(DEPTHAI_BASALT_SUPPORT "Enable optional Basalt support" OFF)


# Include project dependencies
set(DEPTHAI_DEPENDENCY_INCLUDE "" CACHE FILEPATH "Optional cmake file to append to dependency processing, e.g. additional find_package()")
include(depthaiDependencies)

# Add threads preference
set(THREADS_PREFER_PTHREAD_FLAG ON)

# TODO Remove shared naming
set(DEPTHAI_SHARED_3RDPARTY_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}/include/3rdparty/
)
# Add depthai-bootloader-shared
include(${CMAKE_CURRENT_LIST_DIR}/shared/depthai-bootloader-shared.cmake)

# Add flags helpers
include(Flags)

### End of dependencies

set(TARGET_CORE_NAME ${PROJECT_NAME}-core)
set(TARGET_CORE_ALIAS core)

# Create core library
add_library(${TARGET_CORE_NAME}
    # depthai-bootloader-shared sources
    "${DEPTHAI_BOOTLOADER_SHARED_SOURCES}"
    # sources
    src/device/Device.cpp
    src/device/DeviceBase.cpp
    src/device/DeviceBootloader.cpp
    src/device/DataQueue.cpp
    # src/device/CallbackHandler.cpp
    src/device/CalibrationHandler.cpp
    src/device/Version.cpp
    src/pipeline/Pipeline.cpp
    src/pipeline/AssetManager.cpp
    src/pipeline/MessageQueue.cpp
    src/pipeline/Node.cpp
    src/pipeline/InputQueue.cpp
    src/pipeline/ThreadedNode.cpp
    src/pipeline/DeviceNode.cpp
    src/pipeline/node/XLinkIn.cpp
    src/pipeline/node/XLinkOut.cpp
    src/pipeline/node/ColorCamera.cpp
    src/pipeline/node/Camera.cpp
    src/pipeline/node/ToF.cpp
    src/pipeline/node/MessageDemux.cpp
    src/pipeline/node/MonoCamera.cpp
    src/pipeline/node/StereoDepth.cpp
    src/pipeline/node/Sync.cpp
    src/pipeline/node/NeuralNetwork.cpp
    src/pipeline/node/ImageManip.cpp
    src/pipeline/node/Warp.cpp
    src/pipeline/node/VideoEncoder.cpp
    src/pipeline/node/DetectionNetwork.cpp
    src/pipeline/node/Script.cpp
    src/pipeline/node/Pool.cpp
    src/pipeline/node/Benchmark.cpp
    src/pipeline/node/SpatialDetectionNetwork.cpp
    src/pipeline/node/SystemLogger.cpp
    src/pipeline/node/SpatialLocationCalculator.cpp
    src/pipeline/node/AprilTag.cpp
    src/pipeline/node/ObjectTracker.cpp
    src/pipeline/node/IMU.cpp
    src/pipeline/node/EdgeDetector.cpp
    src/pipeline/node/SPIIn.cpp
    src/pipeline/node/FeatureTracker.cpp
    src/pipeline/node/ToF.cpp
    src/pipeline/node/DetectionParser.cpp
    src/pipeline/node/test/MyProducer.cpp
    src/pipeline/node/test/MyConsumer.cpp
    src/pipeline/node/UVC.cpp
    src/pipeline/node/host/XLinkInHost.cpp
    src/pipeline/node/host/XLinkOutHost.cpp
    src/pipeline/node/host/HostNode.cpp
    src/pipeline/datatype/DatatypeEnum.cpp
    src/pipeline/node/PointCloud.cpp
    src/pipeline/datatype/Buffer.cpp
    src/pipeline/datatype/ImgFrame.cpp
    src/pipeline/datatype/ImgTransformations.cpp
    src/pipeline/datatype/EncodedFrame.cpp
    src/pipeline/datatype/ImageManipConfig.cpp
    src/pipeline/datatype/CameraControl.cpp
    src/pipeline/datatype/NNData.cpp
    src/pipeline/datatype/ImgDetections.cpp
    src/pipeline/datatype/SpatialImgDetections.cpp
    src/pipeline/datatype/SystemInformation.cpp
    src/pipeline/datatype/SystemInformationS3.cpp
    src/pipeline/datatype/StreamMessageParser.cpp
    src/pipeline/datatype/SpatialLocationCalculatorData.cpp
    src/pipeline/datatype/SpatialLocationCalculatorConfig.cpp
    src/pipeline/datatype/AprilTags.cpp
    src/pipeline/datatype/AprilTagConfig.cpp
    src/pipeline/datatype/Tracklets.cpp
    src/pipeline/datatype/IMUData.cpp
    src/pipeline/datatype/StereoDepthConfig.cpp
    src/pipeline/datatype/EdgeDetectorConfig.cpp
    src/pipeline/datatype/TrackedFeatures.cpp
    src/pipeline/datatype/FeatureTrackerConfig.cpp
    src/pipeline/datatype/ToFConfig.cpp
    src/pipeline/datatype/BenchmarkReport.cpp
    src/pipeline/datatype/PointCloudConfig.cpp
    src/pipeline/datatype/PointCloudData.cpp
    src/pipeline/datatype/MessageGroup.cpp
    src/pipeline/datatype/TransformData.cpp
    src/utility/H26xParsers.cpp
    src/utility/Initialization.cpp
    src/utility/Resources.cpp
    src/utility/Path.cpp
    src/utility/Platform.cpp
    src/utility/RecordReplay.cpp
    src/utility/McapImpl.cpp
    src/utility/Environment.cpp
    src/utility/Compression.cpp
    src/utility/XLinkGlobalProfilingLogger.cpp
    src/utility/Logging.cpp
    src/utility/Checksum.cpp
    src/utility/matrixOps.cpp
    src/utility/EepromDataParser.cpp
    src/utility/LogCollection.cpp
    src/xlink/XLinkConnection.cpp
    src/xlink/XLinkStream.cpp
    src/openvino/OpenVINO.cpp
    src/openvino/BlobReader.cpp
    src/bspatch/bspatch.c
    src/device/DeviceGate.cpp
    src/utility/ArchiveUtil.cpp
    src/nn_archive/NNArchive.cpp
    src/nn_archive/NNArchiveConfig.cpp
    src/nn_archive/NNArchiveBlob.cpp
    src/nn_archive/NNArchiveConfigHelper.cpp
)
add_library("${PROJECT_NAME}::${TARGET_CORE_ALIAS}" ALIAS ${TARGET_CORE_NAME})
# Specify that we are building core
target_compile_definitions(${TARGET_CORE_NAME} PUBLIC DEPTHAI_TARGET_CORE)
# Specifies name of generated IMPORTED target (set to alias)
set_target_properties(${TARGET_CORE_NAME} PROPERTIES EXPORT_NAME ${TARGET_CORE_ALIAS})
# Add to list of targets to export and install
list(APPEND targets_to_export ${TARGET_CORE_NAME})

# Add default flags to core
add_default_flags(${TARGET_CORE_NAME})

# And clang-tidy and format
if(DEPTHAI_CLANG_TIDY)
    include(ClangTidy)
    target_clangtidy_setup(${TARGET_CORE_NAME})
endif()

# Set compiler features (c++17), and disables extensions (g++17)
set_property(TARGET ${TARGET_CORE_NAME} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${TARGET_CORE_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${TARGET_CORE_NAME} PROPERTY CXX_EXTENSIONS OFF)
# Add interface transitive property (C++17)
if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    target_compile_features(${TARGET_CORE_NAME} INTERFACE cxx_generic_lambdas)
else()
    target_compile_features(${TARGET_CORE_NAME} INTERFACE cxx_std_17)
endif()
# Minimal supported macOS SDK version for C++17 support with std::filesystem
set(CMAKE_OSX_DEPLOYMENT_TARGET 10.15)

# First specify options
option(DEPTHAI_BUILD_TESTS "Build tests" OFF)
option(DEPTHAI_BUILD_EXAMPLES "Build examples - Requires OpenCV library to be installed" OFF)
option(DEPTHAI_BUILD_PYTHON "Build python bindings" OFF)
option(DEPTHAI_BUILD_DOCS "Build documentation - requires doxygen to be installed" OFF)

# Specify support for FW
option(DEPTHAI_ENABLE_DEVICE_FW "Enable MyriadX Device FW" ON)
option(DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW "Enable MyriadX Device Bootloader FW" ON)
option(DEPTHAI_ENABLE_DEVICE_RVC3_FW "Enable RVC3 Device FW" ON)
option(DEPTHAI_ENABLE_DEVICE_RVC4_FW "Enable RVC4 Device FW" ON)

option(DEPTHAI_BINARIES_RESOURCE_COMPILE "Compile Depthai device side binaries into library" ON)

# Set constant
set(DEPTHAI_RESOURCES_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/resources")

# Include configuration
include(Depthai/DepthaiDeviceSideConfig)    # Depthai device binary commit/version configuration
include(Depthai/DepthaiBootloaderConfig)    # Depthai bootloader binary commit/version configuration
include(Depthai/DepthaiDeviceKbConfig)    # depthai-device-kb fwp commit/version configuration
include(Depthai/DepthaiDeviceRVC4Config)    # depthai-device-rvc4 fwp commit/version configuration

# Include downloaders
include(DepthaiDownloader)                  # Depthai device binary downloader
include(DepthaiBootloaderDownloader)        # Depthai bootloader binary downloader
include(DepthaiDeviceKbDownloader)          # depthai-device-kb fwp downloader


# depthai-shared enforce commit hash match if CI
if($ENV{CI})
    # TODO(themarpe) - Disable before final merge
    # set(DEPTHAI_SHARED_COMMIT_HASH_ENFORCE ON)
    set(DEPTHAI_SHARED_COMMIT_HASH_ENFORCE OFF)
    set(DEPTHAI_BOOTLOADER_SHARED_COMMIT_HASH_ENFORCE ON)
endif()

# No user specified paths, download from server
message(STATUS "Downloading Depthai device side binaries from server...")

# Then get the Depthai device side binaries (local or download)
if(DEPTHAI_CMD_PATH OR DEPTHAI_USB2_CMD_PATH OR DEPTHAI_USB2_PATCH_PATH)
    # At least one of the paths is set. include binaries locally
    message(STATUS "Using local Depthai device side binaries...")

    DepthaiLocal(
        PATCH_ONLY ${DEPTHAI_USB2_PATCH_ONLY_MODE}
        "${DEPTHAI_RESOURCES_OUTPUT_DIR}"            # Output folder
        DEPTHAI_RESOURCE_LIST                       # List of output resources
        "${DEPTHAI_CMD_PATH}"                       # depthai.cmd
        "${DEPTHAI_USB2_CMD_PATH}"                  # depthai-usb2.cmd
        "${DEPTHAI_USB2_PATCH_PATH}"                # depthai-usb2-patch.patch
    )

else()
    # No user specified paths, download from server
    message(STATUS "Downloading Depthai device side binaries from server...")
endif()

if(DEPTHAI_ENABLE_DEVICE_FW)
    # Add device FW

    DepthaiDownload(
        "${DEPTHAI_SHARED_COMMIT_HASH}" "${DEPTHAI_SHARED_COMMIT_HASH_ENFORCE}"
        PATCH_ONLY ON
        "${DEPTHAI_RESOURCES_OUTPUT_DIR}"            # Output folder
        DEPTHAI_RESOURCE_LIST                       # List of output resources
        "${DEPTHAI_DEVICE_SIDE_MATURITY}"           # Maturity
        "${DEPTHAI_DEVICE_SIDE_COMMIT}"             # commit hash
        "${DEPTHAI_DEVICE_SIDE_VERSION}"            # Optional version
    )
    list(APPEND RESOURCE_COMPILED_FILES ${DEPTHAI_RESOURCE_LIST})
endif()

if(DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW)
    # Add bootloader FW
    DepthaiBootloaderDownload(
        "${DEPTHAI_BOOTLOADER_SHARED_COMMIT_HASH}" "${DEPTHAI_BOOTLOADER_SHARED_COMMIT_HASH_ENFORCE}"
        "${DEPTHAI_RESOURCES_OUTPUT_DIR}"                # Output folder
        DEPTHAI_BOOTLOADER_RESOURCE_LIST                # List of output resources
        "${DEPTHAI_BOOTLOADER_MATURITY}"                # Maturity
        "${DEPTHAI_BOOTLOADER_VERSION}"                 # if maturity == snapshot -> hash else version
    )
    list(APPEND RESOURCE_COMPILED_FILES ${DEPTHAI_BOOTLOADER_RESOURCE_LIST})
endif()

if(DEPTHAI_ENABLE_DEVICE_RVC3_FW)
    # Add device-kb FW
    DepthaiDeviceDownloader(
        "depthai-device-kb"
        "luxonis-keembay-snapshot-local"
        "luxonis-keembay-release-local"
        "${DEPTHAI_SHARED_COMMIT_HASH}" "${DEPTHAI_SHARED_COMMIT_HASH_ENFORCE}"
        "${DEPTHAI_RESOURCES_OUTPUT_DIR}"                # Output folder
        DEPTHAI_DEVICE_KB_RESOURCE_LIST                 # List of output resources
        "${DEPTHAI_DEVICE_KB_MATURITY}"                # Maturity
        "${DEPTHAI_DEVICE_RVC3_VERSION}"
    )
    list(APPEND RESOURCE_COMPILED_FILES ${DEPTHAI_DEVICE_KB_RESOURCE_LIST})
endif()

if(DEPTHAI_ENABLE_DEVICE_RVC4_FW)
    # Add device-RVC4 FW
    DepthaiDeviceDownloader(
        "depthai-device-rvc4"
        "luxonis-rvc4-snapshot-local"
        "luxonis-rvc4-release-local"
        "${DEPTHAI_SHARED_COMMIT_HASH}" "${DEPTHAI_SHARED_COMMIT_HASH_ENFORCE}"
        "${DEPTHAI_RESOURCES_OUTPUT_DIR}"                # Output folder
        DEPTHAI_DEVICE_RVC4_RESOURCE_LIST                 # List of output resources
        "${DEPTHAI_DEVICE_RVC4_MATURITY}"                # Maturity
        "${DEPTHAI_DEVICE_RVC4_VERSION}"
    )
    list(APPEND RESOURCE_COMPILED_FILES ${DEPTHAI_DEVICE_RVC4_RESOURCE_LIST})
endif()

message(STATUS "LIST OF RESOURCE COMPILED FILES: ${RESOURCE_COMPILED_FILES}")

if(DEPTHAI_BINARIES_RESOURCE_COMPILE)
    # Add RC and resource compile the binares
    include(CMakeRC)

    set(DEPTHAI_RESOURCE_LIBRARY_NAME "depthai-resources")

    # Add resource library
    cmrc_add_resource_library("${DEPTHAI_RESOURCE_LIBRARY_NAME}" NAMESPACE depthai
        WHENCE "${DEPTHAI_RESOURCES_OUTPUT_DIR}"
        "${RESOURCE_COMPILED_FILES}"
    )

    # Link to resource library
    target_link_libraries(${TARGET_CORE_NAME} PRIVATE "${DEPTHAI_RESOURCE_LIBRARY_NAME}")

    # Set define that binaries are resource compiled
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_RESOURCE_COMPILED_BINARIES)

else()
    # TODO
    # Don't add RC and don't resource compile the binaries
    # Install to share/ instead for instance
endif()

# Add include directories
target_include_directories(${TARGET_CORE_NAME}
    PUBLIC
        # Relative path to include directories after installed
        "$<INSTALL_INTERFACE:include>"
        "$<INSTALL_INTERFACE:include/${DEPTHAI_SHARED_3RDPARTY_HEADERS_PATH}>"

        # Build time path to include directories
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
        "$<BUILD_INTERFACE:${DEPTHAI_SHARED_PUBLIC_INCLUDE}>"
        "$<BUILD_INTERFACE:${DEPTHAI_BOOTLOADER_SHARED_PUBLIC_INCLUDE}>"
    #INTERFACE
    #    # ...
    PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/depthai>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>"
        "$<BUILD_INTERFACE:${DEPTHAI_SHARED_INCLUDE}>"
        "$<BUILD_INTERFACE:${DEPTHAI_BOOTLOADER_SHARED_INCLUDE}>"
)

target_include_directories(${TARGET_CORE_NAME} SYSTEM
    PUBLIC
        "$<BUILD_INTERFACE:${DEPTHAI_SHARED_3RDPARTY_INCLUDE}>"
)

# Add clang format after specifying include directories
if(DEPTHAI_CLANG_FORMAT)
    # HEADER DIRECTORIES
    set(header_dirs "${CMAKE_CURRENT_LIST_DIR}/include" "${DEPTHAI_SHARED_PUBLIC_INCLUDE}" "${DEPTHAI_SHARED_INCLUDE}")
    include(ClangFormat)
    target_clangformat_setup(${TARGET_CORE_NAME} "${header_dirs}")
endif()

# link libraries
target_link_libraries(${TARGET_CORE_NAME}
    PUBLIC
        nlohmann_json::nlohmann_json
        libnop
        spdlog::spdlog
    INTERFACE
        XLinkPublic
    PRIVATE
        XLink
        Threads::Threads
        BZip2::bz2
        FP16::fp16
        archive_static
        ZLIB::zlib
        LZ4::lz4
        httplib::httplib
        mp4v2::mp4v2
)

if(DEPTHAI_ENABLE_CURL)
    target_link_libraries(${TARGET_CORE_NAME} PRIVATE
        CURL::libcurl
        cpr::cpr
    )
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_ENABLE_CURL)
endif()

# Add compile & CMake definitions
set(DEPTHAI_DEVICE_VERSION "${DEPTHAI_DEVICE_SIDE_COMMIT}")
target_compile_definitions(${TARGET_CORE_NAME}
    PRIVATE
        # Add depthai-device version
        DEPTHAI_DEVICE_VERSION="${DEPTHAI_DEVICE_VERSION}"
        # Add depthai-bootloader version
        DEPTHAI_BOOTLOADER_VERSION="${DEPTHAI_BOOTLOADER_VERSION}"
        # Add depthai-device-kb version
        DEPTHAI_DEVICE_RVC3_VERSION="${DEPTHAI_DEVICE_RVC3_VERSION}"
        # Add depthai-device-rvc4 version
        DEPTHAI_DEVICE_RVC4_VERSION="${DEPTHAI_DEVICE_RVC4_VERSION}"
)
# Add compile flag if libusb is available
if(DEPTHAI_ENABLE_LIBUSB)
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_ENABLE_LIBUSB)
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_HAVE_LIBUSB_SUPPORT)
    set(DEPTHAI_HAVE_LIBUSB_SUPPORT ON)
endif()

if(DEPTHAI_XTENSOR_SUPPORT)
    target_compile_definitions(${TARGET_CORE_NAME} PUBLIC DEPTHAI_XTENSOR_SUPPORT)
    target_link_libraries(${TARGET_CORE_NAME} PUBLIC xtensor)
endif()

# Specify available FW
if(DEPTHAI_ENABLE_DEVICE_FW)
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_ENABLE_DEVICE_FW)
endif()
if(DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW)
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW)
endif()
if(DEPTHAI_ENABLE_DEVICE_RVC3_FW)
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_ENABLE_DEVICE_RVC3_FW)
endif()
if(DEPTHAI_ENABLE_DEVICE_RVC4_FW)
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_ENABLE_DEVICE_RVC4_FW)
endif()

# Add Backward dependency if enabled (On by default)
if(DEPTHAI_ENABLE_BACKWARD)
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_ENABLE_BACKWARD)
    target_link_libraries(${TARGET_CORE_NAME} PRIVATE Backward::Backward)
endif()

# Add patch only mode definition
if(DEPTHAI_USB2_PATCH_ONLY_MODE)
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_PATCH_ONLY_MODE)
endif()

# Helper function
macro(add_runtime_dependencies depending_target dependency)
    if(WIN32)
        if(TARGET ${dependency})
            get_property(imported_configs TARGET ${dependency} PROPERTY IMPORTED_CONFIGURATIONS)
            set(dlls "")
            foreach(cfg ${imported_configs})
                get_property(dll TARGET ${dependency} PROPERTY IMPORTED_LOCATION_${cfg})
                set(dlls ${depthai_dll_libraries} $<$<CONFIG:${cfg}>:${dll}>)
            endforeach()
        endif()
        file(GLOB depthai_dll_libraries "${HUNTER_INSTALL_PREFIX}/bin/*.dll")
        # Create a list of required dll files
        set(required_dll_files ${dlls} ${depthai_dll_libraries})
        # Copy the required dlls
        add_custom_command(TARGET ${depending_target} POST_BUILD COMMAND
            "$<$<BOOL:${required_dll_files}>:${CMAKE_COMMAND};-E;copy_if_different;${required_dll_files};$<TARGET_FILE_DIR:${depending_target}>>"
            COMMAND_EXPAND_LISTS
            VERBATIM
        )
        message(STATUS "Required dlls for core are: ${required_dll_files}")
    endif()
endmacro()
# Add libusb dll in build time
add_runtime_dependencies(${TARGET_CORE_NAME} usb-1.0)

########################
# OpenCV Support
########################
set(THIRDPARTY_OPENCV_LIBRARIES "" CACHE STRING "Optional libraries to link OpenCV support, e.g. TBB::tbb")
set(TARGET_OPENCV_NAME ${PROJECT_NAME}-opencv)
set(TARGET_OPENCV_ALIAS opencv)
if(DEPTHAI_OPENCV_SUPPORT)
    # Check if required libraries are available
    set(REQUIRED_OPENCV_LIBRARIES "opencv_core" "opencv_imgproc" "opencv_videoio")
    set(OPENCV_SUPPORT_AVAILABLE ${OpenCV_FOUND})
    foreach(lib ${REQUIRED_OPENCV_LIBRARIES})
        if(NOT (lib IN_LIST OpenCV_LIBS))
            set(OPENCV_SUPPORT_AVAILABLE FALSE)
        endif()
    endforeach()

    if(OPENCV_SUPPORT_AVAILABLE)

        # Add depthai-core-opencv library and depthai::core::opencv alias
        add_library(${TARGET_OPENCV_NAME}
            src/opencv/ImgFrame.cpp
            src/pipeline/node/host/Display.cpp
            src/pipeline/node/host/HostCamera.cpp
            src/pipeline/node/host/Record.cpp
            src/pipeline/node/host/Replay.cpp
            src/opencv/RecordReplay.cpp
        )
        add_library("${PROJECT_NAME}::${TARGET_OPENCV_ALIAS}" ALIAS ${TARGET_OPENCV_NAME})
        # Specifies name of generated IMPORTED target (set to alias)
        set_target_properties(${TARGET_OPENCV_NAME} PROPERTIES EXPORT_NAME ${TARGET_OPENCV_ALIAS})

        # Add default flags
        add_default_flags(${TARGET_OPENCV_NAME})
        add_flag(${TARGET_OPENCV_NAME} -Wno-switch-enum)

        # Link to OpenCV (publically)
        target_link_libraries(${TARGET_OPENCV_NAME} PUBLIC ${REQUIRED_OPENCV_LIBRARIES} ${THIRDPARTY_OPENCV_LIBRARIES})

        # Add public compile definition indicating that OpenCV support is available
        set(DEPTHAI_HAVE_OPENCV_SUPPORT ON)

        # Specify that we are building target opencv
        target_compile_definitions(${TARGET_OPENCV_NAME} PUBLIC DEPTHAI_TARGET_OPENCV)
        target_compile_definitions(${TARGET_OPENCV_NAME} PUBLIC DEPTHAI_HAVE_OPENCV_SUPPORT)
        # Add public dependency to depthai::core library
        target_link_libraries(${TARGET_OPENCV_NAME} PUBLIC ${TARGET_CORE_NAME})

        # Add to clangformat target
        if(COMMAND target_clangformat_setup)
            target_clangformat_setup(${TARGET_OPENCV_NAME} "")
        endif()

        # Add to list of targets to export and install
        list(APPEND targets_to_export ${TARGET_OPENCV_NAME})

        message(STATUS "OpenCV and required libraries (${REQUIRED_OPENCV_LIBRARIES}) found. OpenCV Support enabled")

    else()
        message(STATUS "OpenCV or required libraries (${REQUIRED_OPENCV_LIBRARIES}) not found. OpenCV Support disabled")
    endif()
endif()


########################
# PCL Support
########################
set(TARGET_PCL_NAME ${PROJECT_NAME}-pcl)
set(TARGET_PCL_ALIAS pcl)
if(DEPTHAI_PCL_SUPPORT)
    if(PCL_FOUND)
        # Add depthai-core-pcl library and depthai::core::pcl alias
        add_library(${TARGET_PCL_NAME} src/pcl/PointCloudData.cpp)
        add_library("${PROJECT_NAME}::${TARGET_PCL_ALIAS}" ALIAS ${TARGET_PCL_NAME})
        # Specifies name of generated IMPORTED target (set to alias)
        set_target_properties(${TARGET_PCL_NAME} PROPERTIES EXPORT_NAME ${TARGET_PCL_ALIAS})

        # Add default flags
        add_default_flags(${TARGET_PCL_NAME})
        add_flag(${TARGET_PCL_NAME} -Wno-switch-enum)

        # Link to PCL (publically)
        target_link_libraries(${TARGET_PCL_NAME} PUBLIC ${PCL_LIBRARIES})
        target_link_libraries(${TARGET_PCL_NAME} PUBLIC pcl_io_ply)

        # Add public compile definition indicating that PCL support is available
        set(DEPTHAI_HAVE_PCL_SUPPORT ON)

        # Specify that we are building target pcl
        target_compile_definitions(${TARGET_PCL_NAME} PUBLIC DEPTHAI_TARGET_PCL)
        target_compile_definitions(${TARGET_PCL_NAME} PUBLIC DEPTHAI_HAVE_PCL_SUPPORT)


        target_include_directories(${TARGET_PCL_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
        target_link_directories(${TARGET_PCL_NAME} PUBLIC ${PCL_LIBRARY_DIRS})
        target_compile_definitions(${TARGET_PCL_NAME} PUBLIC ${PCL_DEFINITIONS})

        # Add public dependency to depthai::core library
        target_include_directories(${TARGET_CORE_NAME} PUBLIC $<TARGET_PROPERTY:${TARGET_PCL_NAME},INTERFACE_INCLUDE_DIRECTORIES>)
        target_link_libraries(${TARGET_PCL_NAME} PUBLIC ${TARGET_CORE_NAME})

        # Add to clangformat target
        if(COMMAND target_clangformat_setup)
            target_clangformat_setup(${TARGET_PCL_NAME} "")
        endif()

        # Add to list of targets to export and install
        list(APPEND targets_to_export ${TARGET_PCL_NAME})

        message(STATUS "PCL found. PCL Support enabled")

    else()
        message(STATUS "PCL not found. PCL Support disabled")
    endif()
endif()

########################
# RTABMap Support
########################
set(THIRDPARTY_RTABMAP_LIBRARIES "" CACHE STRING "Optional libraries to link RTABMap support, e.g. TBB::tbb")
set(TARGET_RTABMAP_NAME ${PROJECT_NAME}-rtabmap)
set(TARGET_RTABMAP_ALIAS rtabmap)
if(DEPTHAI_RTABMAP_SUPPORT)
    # Check if required libraries are available
    set(RTABMAP_SUPPORT_AVAILABLE ${RTABMap_FOUND})

    if(RTABMAP_SUPPORT_AVAILABLE)

        # Add depthai-rtabmap library and depthai::rtabmap alias
        add_library(${TARGET_RTABMAP_NAME} src/rtabmap/CalibrationHandler.cpp src/rtabmap/TransformData.cpp  src/rtabmap/RTABMapVIO.cpp src/rtabmap/RTABMapSLAM.cpp )
        add_library("${PROJECT_NAME}::${TARGET_RTABMAP_ALIAS}" ALIAS ${TARGET_RTABMAP_NAME})
        # Specifies name of generated IMPORTED target (set to alias)
        set_target_properties(${TARGET_RTABMAP_NAME} PROPERTIES EXPORT_NAME ${TARGET_RTABMAP_ALIAS})

        # Add default flags
        add_default_flags(${TARGET_RTABMAP_NAME})
        add_flag(${TARGET_RTABMAP_NAME} -Wno-switch-enum)
        target_link_options(${TARGET_RTABMAP_NAME} INTERFACE LINKER:--disable-new-dtags)
        # Link to RTABMap (publically)
        target_link_libraries(${TARGET_RTABMAP_NAME} PUBLIC rtabmap::core ${TARGET_OPENCV_NAME} ${TARGET_PCL_NAME} ${THIRDPARTY_RTABMAP_LIBRARIES})

        # Add public compile definition indicating that RTABMap support is available
        set(DEPTHAI_HAVE_RTABMAP_SUPPORT ON)

        # Specify that we are building target rtabmap
        target_compile_definitions(${TARGET_RTABMAP_NAME} PUBLIC DEPTHAI_TARGET_RTABMAP)
        target_compile_definitions(${TARGET_RTABMAP_NAME} PUBLIC DEPTHAI_HAVE_RTABMAP_SUPPORT)
        # Add public dependency to depthai::core library
        target_link_libraries(${TARGET_RTABMAP_NAME} PUBLIC ${TARGET_CORE_NAME})

        # Add to clangformat target
        if(COMMAND target_clangformat_setup)
            target_clangformat_setup(${TARGET_RTABMAP_NAME} "")
        endif()

        # Add to list of targets to export and install
        list(APPEND targets_to_export ${TARGET_RTABMAP_NAME})

        message(STATUS "RTABMap and required libraries (${REQUIRED_RTABMAP_LIBRARIES}) found. RTABMap Support enabled")

    else()
        message(STATUS "RTABMap or required libraries (${REQUIRED_RTABMAP_LIBRARIES}) not found. RTABMap Support disabled")
    endif()
endif()


########################
# Basalt Support
########################

set(THIRDPARTY_BASALT_LIBRARIES "TBB::tbb;Eigen3::Eigen" CACHE STRING "Optional libraries to link Basalt support, e.g. TBB::tbb")
set(TARGET_BASALT_NAME ${PROJECT_NAME}-basalt)
set(TARGET_BASALT_ALIAS basalt)

if(DEPTHAI_BASALT_SUPPORT)
    # Check if required libraries are available
    find_package(basalt_sdk CONFIG REQUIRED)
    find_package(TBB CONFIG REQUIRED)
    set(BASALT_SUPPORT_AVAILABLE ${basalt_sdk_FOUND})

    if(BASALT_SUPPORT_AVAILABLE)

        # Add depthai-basalt library and depthai::basalt alias
        add_library(${TARGET_BASALT_NAME} src/basalt/BasaltVIO.cpp )
        add_library("${PROJECT_NAME}::${TARGET_BASALT_ALIAS}" ALIAS ${TARGET_BASALT_NAME})

        # Specifies name of generated IMPORTED target (set to alias)
        set_target_properties(${TARGET_BASALT_NAME} PROPERTIES EXPORT_NAME ${TARGET_BASALT_ALIAS})

        # Add default flags
        add_default_flags(${TARGET_BASALT_NAME})
        add_flag(${TARGET_BASALT_NAME} -Wno-switch-enum)

        # Link to Basalt (publically)
        target_link_libraries(${TARGET_BASALT_NAME} PUBLIC basalt_sdk::basalt_sdk ${THIRDPARTY_BASALT_LIBRARIES} ${TARGET_OPENCV_NAME} )

        # Add public compile definition indicating that Basalt support is available
        set(DEPTHAI_HAVE_BASALT_SUPPORT ON)

        # Specify that we are building target basalt
        target_compile_definitions(${TARGET_BASALT_NAME} PUBLIC DEPTHAI_TARGET_BASALT)
        target_compile_definitions(${TARGET_BASALT_NAME} PUBLIC DEPTHAI_HAVE_BASALT_SUPPORT)

        # Add public dependency to depthai::core library
        target_link_libraries(${TARGET_BASALT_NAME} PUBLIC ${TARGET_CORE_NAME})

        # Add to clangformat target
        if(COMMAND target_clangformat_setup)
            target_clangformat_setup(${TARGET_BASALT_NAME} "")
        endif()

        # Add to list of targets to export and install
        list(APPEND targets_to_export ${TARGET_BASALT_NAME})

        message(STATUS "Basalt and required libraries (${REQUIRED_BASALT_LIBRARIES}) found. Basalt Support enabled")

    else()
        message(STATUS "Basalt or required libraries (${REQUIRED_BASALT_LIBRARIES}) not found. Basalt Support disabled")
    endif()
endif()

########################
# Combined target
########################
set(TARGET_ALL_ALIAS all)
set(TARGET_ALL_NAME ${PROJECT_NAME}-all)
if(DEPTHAI_HAVE_OPENCV_SUPPORT AND DEPTHAI_HAVE_PCL_SUPPORT AND DEPTHAI_HAVE_BASALT_SUPPORT AND DEPTHAI_HAVE_RTABMAP_SUPPORT)
    add_library(${TARGET_ALL_NAME} INTERFACE)
    add_library("${PROJECT_NAME}::${TARGET_ALL_ALIAS}" ALIAS ${TARGET_ALL_NAME})
    # Specifies name of generated IMPORTED target (set to alias)
    set_target_properties(${TARGET_ALL_NAME} PROPERTIES EXPORT_NAME ${TARGET_ALL_ALIAS})
    target_link_libraries(${TARGET_ALL_NAME} INTERFACE ${TARGET_CORE_NAME} ${TARGET_OPENCV_NAME} ${TARGET_PCL_NAME} ${TARGET_BASALT_NAME} ${TARGET_RTABMAP_NAME})

    # Add to list of targets to export and install
    list(APPEND targets_to_export ${TARGET_ALL_NAME})

    message(STATUS "OpenCV, PCL, Basalt and RTABMap found. Combined target enabled")
endif()



########################
# Sanitizers
########################
if(DEPTHAI_SANITIZE)
    set(SANITIZE_ADDRESS ON CACHE BOOL "Enable AddressSanitizer for sanitized targets." FORCE)
    set(SANITIZE_UNDEFINED ON CACHE BOOL "Enable UndefinedBehaviorSanitizer for sanitized targets." FORCE)
    find_package(Sanitizers)
    add_sanitizers(${TARGET_CORE_NAME})
    if(DEPTHAI_HAVE_OPENCV_SUPPORT)
        add_sanitizers(${TARGET_OPENCV_NAME})
    endif()
    if(DEPTHAI_HAVE_RTABMAP_SUPPORT)
        add_sanitizers(${TARGET_RTABMAP_NAME})
    endif()
    if(DEPTHAI_HAVE_BASALT_SUPPORT)
        add_sanitizers(${TARGET_BASALT_NAME})
    endif()
    if(DEPTHAI_HAVE_PCL_SUPPORT)
        add_sanitizers(${TARGET_PCL_NAME})
    endif()
    if(DEPTHAI_XLINK_LOCAL)
        add_sanitizers(XLink)
        if(XLINK_LIBUSB_LOCAL)
            add_sanitizers(usb-1.0)
        endif()
    endif()
endif()

########################
# Testing infrastructure
########################
include(CTest)
enable_testing()

########################
# Tests
########################
if (DEPTHAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()

########################
# Examples (can also act as tests)
########################
if (DEPTHAI_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if (EMSCRIPTEN)
    add_subdirectory(bindings/js)
endif()

if (DEPTHAI_BUILD_PYTHON)
  add_subdirectory(bindings/python)
endif()

########################
# Documentation
########################
if (DEPTHAI_BUILD_DOCS)
    add_subdirectory(docs)
endif()

########################
# Build configuration
########################
# Add year information
string(TIMESTAMP BUILD_DATETIME "%Y-%m-%d %H:%M:%S +0000" UTC)
message(STATUS "BUILD_DATETIME: ${BUILD_DATETIME}, BUILD_COMMIT: ${BUILD_COMMIT}, BUILD_COMMIT_DATETIME: ${BUILD_COMMIT_DATETIME}")

# Configure build information (version, opencv, pcl support)
configure_file("${CMAKE_CURRENT_LIST_DIR}/cmake/version.hpp.in" "${CMAKE_CURRENT_LIST_DIR}/include/depthai/build/version.hpp")

########################
# Export and install
########################
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Add additional targets to export group
if(NOT BUILD_SHARED_LIBS)
    list(APPEND targets_to_export ${DEPTHAI_RESOURCE_LIBRARY_NAME} cmrc-base)
endif()

# Export targets (capability to import current build directory)
export(TARGETS ${targets_to_export} NAMESPACE ${PROJECT_NAME}:: FILE "${PROJECT_NAME}Targets.cmake")

# Dependencies file
configure_file("cmake/${PROJECT_NAME}Dependencies.cmake" ${PROJECT_NAME}Dependencies.cmake COPYONLY)

# Write project version
write_basic_package_version_file(${PROJECT_NAME}ConfigVersion.cmake VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion)

# Configure config file (one for exporting build directory, one for installation)
file(RELATIVE_PATH DEPTHAI_DEPENDENCIES_INSTALLATION_PATH_REL "${CMAKE_CURRENT_BINARY_DIR}" "${HUNTER_INSTALL_PREFIX}")
configure_file(cmake/${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake @ONLY)

# Config for installation
set(DEPTHAI_DEPENDENCIES_INSTALLATION_PATH_REL "./dependencies")
configure_file(cmake/${PROJECT_NAME}Config.cmake.in _install/${PROJECT_NAME}Config.cmake @ONLY)

# Modify RPath to point to the cmake/depthai/dependencies/lib
# note: macOS is APPLE and also UNIX!
if(APPLE)
  set_target_properties(${TARGET_CORE_NAME} PROPERTIES INSTALL_RPATH "@loader_path;@loader_path/cmake/${PROJECT_NAME}/dependencies/lib")
elseif(UNIX)
  set_target_properties(${TARGET_CORE_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/cmake/${PROJECT_NAME}/dependencies/lib")
endif()

# Export to CMake registry if specified
if(CMAKE_EXPORT_PACKAGE_REGISTRY)
    export(PACKAGE depthai)
endif()

if(DEPTHAI_INSTALL)

    # Install targets
    install(
        TARGETS ${targets_to_export}
        EXPORT ${PROJECT_EXPORT_GROUP}
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    )

    # Install depthai public headers
    install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
    # Install depthai-bootloader-shared public headers
    install(DIRECTORY "${DEPTHAI_BOOTLOADER_SHARED_PUBLIC_INCLUDE}/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
    # Install Hunter dependencies
    install(DIRECTORY "${HUNTER_INSTALL_PREFIX}/" DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/dependencies")
    # Install depthai-core dependencies
    install(DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/3rdparty" DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake")
    # Install resources if not RC'd
    if(NOT DEPTHAI_BINARIES_RESOURCE_COMPILE)
        install(DIRECTORY "${DEPTHAI_RESOURCES_OUTPUT_DIR}/" DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}")
    endif()
    # Install any required dll files
    if(DEFINED required_dll_files)
        install(FILES ${required_dll_files} DESTINATION "${CMAKE_INSTALL_BINDIR}")
    endif()

    # Install export group (information about targets)
    install(EXPORT ${PROJECT_EXPORT_GROUP}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
    )

    # Install CMake specific files
    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/_install/${PROJECT_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Dependencies.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
    )

endif()

##############################
# Integration settings export
##############################
set(tmp_definitions "$<TARGET_PROPERTY:${TARGET_CORE_NAME},INTERFACE_COMPILE_DEFINITIONS>")
set(tmp_includes "$<TARGET_PROPERTY:${TARGET_CORE_NAME},INTERFACE_INCLUDE_DIRECTORIES>")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/depthai-core-integration.txt" CONTENT
    "$<$<BOOL:${tmp_definitions}>:\n-D\'$<JOIN:${tmp_definitions},\'\n-D\'>\'>\n
    $<$<BOOL:${tmp_includes}>:\n-I\'$<JOIN:${tmp_includes},\'\n-I\'>\'>\n"
)

if(DEPTHAI_HAVE_OPENCV_SUPPORT)
    set(tmp_definitions "$<TARGET_PROPERTY:${TARGET_OPENCV_NAME},INTERFACE_COMPILE_DEFINITIONS>")
    set(tmp_includes "$<TARGET_PROPERTY:${TARGET_OPENCV_NAME},INTERFACE_INCLUDE_DIRECTORIES>")
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/depthai-opencv-integration.txt" CONTENT
        "$<$<BOOL:${tmp_definitions}>:\n-D\'$<JOIN:${tmp_definitions},\'\n-D\'>\'>\n
        $<$<BOOL:${tmp_includes}>:\n-I\'$<JOIN:${tmp_includes},\'\n-I\'>\'>\n"
    )
endif()
if(DEPTHAI_HAVE_RTABMAP_SUPPORT)
    set(tmp_definitions "$<TARGET_PROPERTY:${TARGET_RTABMAP_NAME},INTERFACE_COMPILE_DEFINITIONS>")
    set(tmp_includes "$<TARGET_PROPERTY:${TARGET_RTABMAP_NAME},INTERFACE_INCLUDE_DIRECTORIES>")
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/depthai-rtabmap-integration.txt" CONTENT
        "$<$<BOOL:${tmp_definitions}>:\n-D\'$<JOIN:${tmp_definitions},\'\n-D\'>\'>\n
        $<$<BOOL:${tmp_includes}>:\n-I\'$<JOIN:${tmp_includes},\'\n-I\'>\'>\n"
    )
endif()
if(DEPTHAI_HAVE_BASALT_SUPPORT)
    set(tmp_definitions "$<TARGET_PROPERTY:${TARGET_BASALT_NAME},INTERFACE_COMPILE_DEFINITIONS>")
    set(tmp_includes "$<TARGET_PROPERTY:${TARGET_BASALT_NAME},INTERFACE_INCLUDE_DIRECTORIES>")
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/depthai-basalt-integration.txt" CONTENT
        "$<$<BOOL:${tmp_definitions}>:\n-D\'$<JOIN:${tmp_definitions},\'\n-D\'>\'>\n
        $<$<BOOL:${tmp_includes}>:\n-I\'$<JOIN:${tmp_includes},\'\n-I\'>\'>\n"
    )
endif()
if(DEPTHAI_HAVE_PCL_SUPPORT)
    set(tmp_definitions "$<TARGET_PROPERTY:${TARGET_PCL_NAME},INTERFACE_COMPILE_DEFINITIONS>")
    set(tmp_includes "$<TARGET_PROPERTY:${TARGET_PCL_NAME},INTERFACE_INCLUDE_DIRECTORIES>")
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/depthai-pcl-integration.txt" CONTENT
        "$<$<BOOL:${tmp_definitions}>:\n-D\'$<JOIN:${tmp_definitions},\'\n-D\'>\'>\n
        $<$<BOOL:${tmp_includes}>:\n-I\'$<JOIN:${tmp_includes},\'\n-I\'>\'>\n"
    )
endif()
