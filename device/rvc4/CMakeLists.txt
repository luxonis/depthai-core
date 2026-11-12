cmake_minimum_required(VERSION 3.13) # For Hunter

# ---------------------------------------------------------------------
# Check for the RVC4 sysroot (by Python sysroot include path)
# ---------------------------------------------------------------------

# The RVC4 Yocto SDK environment script exports OECORE_TARGET_SYSROOT. The legacy
# hard-coded location is kept as a fallback for the current build image.
set(DEPTHAI_DEVICE_RVC4_SYSROOT "$ENV{OECORE_TARGET_SYSROOT}" CACHE PATH "RVC4 target sysroot (aarch64-oe-linux)")
if(NOT DEPTHAI_DEVICE_RVC4_SYSROOT AND EXISTS "/usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux")
    set(DEPTHAI_DEVICE_RVC4_SYSROOT "/usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux" CACHE PATH "RVC4 target sysroot (aarch64-oe-linux)" FORCE)
endif()
if(NOT EXISTS "${DEPTHAI_DEVICE_RVC4_SYSROOT}/usr/include/python3.10/")
    message(FATAL_ERROR "Could not find the RVC4 sysroot: set OECORE_TARGET_SYSROOT (source the SDK environment script) or DEPTHAI_DEVICE_RVC4_SYSROOT. Looked at '${DEPTHAI_DEVICE_RVC4_SYSROOT}'.")
endif()
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
    message(FATAL_ERROR "No QAIRT/SNPE SDK found under ${DEPTHAI_DEVICE_RVC4_SYSROOT}/opt/qcom/aistack/qairt")
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
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/toolchains/OEToolchainConfig.cmake" CACHE STRING "Toolchain file to overload" FORCE)
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
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Toolchain file to overload" FORCE)

set(DEPTHAI_ENABLE_DEVICE_FW OFF CACHE BOOL "" FORCE)
set(DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW OFF CACHE BOOL "" FORCE)
set(DEPTHAI_ENABLE_DEVICE_RVC3_FW OFF CACHE BOOL "" FORCE)
set(DEPTHAI_ENABLE_DEVICE_RVC4_FW OFF CACHE BOOL "" FORCE)
set(DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT OFF CACHE BOOL "" FORCE)
set(XLINK_LIBUSB_SYSTEM ON CACHE BOOL "Use target sysroot libusb for XLink" FORCE)

# Set compiler features (c++17), and disables extensions (g++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add core
set(DEPTHAI_DEVICE_KB_CORE_PATH "${CMAKE_CURRENT_LIST_DIR}/external/depthai-core" CACHE STRING "Path to depthai-core source")
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

# Location of this tree and of its build directory. They differ from CMAKE_SOURCE_DIR /
# CMAKE_BINARY_DIR when the firmware is built as a subdirectory of depthai-core.
set(DEPTHAI_DEVICE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(DEPTHAI_DEVICE_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(DEPTHAI_DEVICE_VCPKG_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg" CACHE PATH "vcpkg checkout used by the build (license and SBOM generation)")

find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")

    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()

add_subdirectory("${DEPTHAI_DEVICE_KB_CORE_PATH}" EXCLUDE_FROM_ALL)
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

# Private Qualcomm/ST module sources (licensee-only code; not part of the public repository): the private repository
# depthai-device-vendor-fw. Default: the environment variable DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR, then a sibling checkout named
# depthai-device-vendor-fw.
set(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR "$ENV{DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}" CACHE PATH "Path to the checkout of the private module (repository depthai-device-vendor-fw)")
if(NOT DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/../depthai-device-vendor-fw/CMakeLists.txt")
    get_filename_component(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../depthai-device-vendor-fw" ABSOLUTE)
endif()
message(STATUS "Private module source dir: '${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}'")

# STMicroelectronics VD55H1 ToF processing (ST SLA0106 library + Qualcomm depth-map wrapper).
# Available only when the private module carries the wrapper; without it the ToF node reports an error.
if(EXISTS "${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}/depthmapwrapper-vd55h1/CMakeLists.txt")
    set(_tof_default ON)
else()
    set(_tof_default OFF)
endif()
option(DEPTHAI_DEVICE_TOF_VD55H1 "Build the VD55H1 ToF depth processing (needs the private depthmapwrapper-vd55h1)" ${_tof_default})
message(STATUS "ToF VD55H1 processing: ${DEPTHAI_DEVICE_TOF_VD55H1}")
# Without the wrapper the core library compiles the stub of the ToF sensor pipeline (see the private module section)
set(DEPTHAI_DEVICE_TOF_SOURCES src/pipeline/node/ToF/tofVD55H1.cpp)
if(NOT DEPTHAI_DEVICE_TOF_VD55H1)
    list(APPEND DEPTHAI_DEVICE_TOF_SOURCES src/pipeline/node/ToF/tofVD55H1Stub.cpp)
endif()

# Add dependencies
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

    # Pipeline / nodes
    src/pipeline/node/NeuralNetwork/rvc4/NeuralNetworkRvc4.cpp
    src/pipeline/node/NeuralNetwork/CreateUserBuffer.cpp
    src/pipeline/node/NeuralDepth/NeuralDepth.cpp
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

# What the private module (depthai-device-rvc4-vendor) compiles against: the include roots of the interface
# headers, depthai-core and the platform defines that change what these headers declare. The core library
# links it PUBLIC, so the firmware and the private module see the same headers with the same defines.
# The private module links only this target: it needs no symbol of depthai-device-core, so the two
# libraries do not form a link cycle.
add_library(depthai-device-accel-api INTERFACE)
target_include_directories(depthai-device-accel-api
    INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}/external/depthai-core/include"
        "${CMAKE_CURRENT_LIST_DIR}/external/depthai-core/src"
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

target_link_libraries(depthai-device-core
    PUBLIC
        depthai-device-accel-api
)

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
    ${GST_INCLUDE_DIRS}
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
set(DEPTHAI_SYSROOT "${DEPTHAI_DEVICE_RVC4_SYSROOT}")

# Point pkg-config to the sysroot pkgconfig directories and tell it to use sysroot
# This makes pkg_search_module(GST ...) find gstreamer-1.0 .pc files in the sysroot
set(ENV{PKG_CONFIG_LIBDIR} "${DEPTHAI_SYSROOT}/usr/lib/pkgconfig;${DEPTHAI_SYSROOT}/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${DEPTHAI_SYSROOT}")
pkg_search_module(GST REQUIRED gstreamer-1.0>=1.4 gstreamer-app-1.0>=1.4 gstallocators-1.0>=1.4)
message(STATUS "GST Libraries: ${GST_LIBRARIES}")

# Include directories (for non-CMake targets)
target_include_directories(depthai-device-core
    PRIVATE
    ${GST_INCLUDE_DIRS}
)

# Compile options required by the Qualcomm camera HAL headers (shared with the private module)
set(DEPTHAI_DEVICE_CAMX_COMPILE_OPTIONS
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
    -DJPEG_POSTPROC_LIB="/usr/lib/camera/components/com.qti.offline.jpeg.so"
    -DENABLE_OFFLINE_JPEG
    -DHAVE_BINDER
)
target_compile_options(depthai-device-core PRIVATE ${DEPTHAI_DEVICE_CAMX_COMPILE_OPTIONS})

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
    eigen
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

    # string formatting
    fmt
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
# Every firmware carries the DSP offload, built from source with the Qualcomm Hexagon SDK. Luxonis
# builds use the SDK of the Luxonis OS SDK image; other builds install it from Qualcomm and set
# HEXAGON_SDK_ROOT to it.
set(DEPTHAI_DEVICE_HEXAGON_SDK_VERSION "5.5.0.1")  # the version that the firmware is built and tested with
if(NOT DEFINED ENV{HEXAGON_SDK_ROOT} OR NOT EXISTS "$ENV{HEXAGON_SDK_ROOT}/build/cmake/hexagon_fun.cmake")
    message(FATAL_ERROR "The firmware needs the Qualcomm Hexagon SDK ${DEPTHAI_DEVICE_HEXAGON_SDK_VERSION} for its DSP offload: "
                        "install it from Qualcomm and set the environment variable HEXAGON_SDK_ROOT to it "
                        "(now: '$ENV{HEXAGON_SDK_ROOT}').")
endif()
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
target_link_libraries(depthai-device-core PRIVATE ImageAlignDsp StereoDepthDsp)
set(DEPTHAI_DEVICE_DSP_SKELS
    "${DEPTHAI_DEVICE_BINARY_DIR}/src/hexagon/ImageAlignDsp/hexagon_build/libImageAlignDsp_skel.so"
    "${DEPTHAI_DEVICE_BINARY_DIR}/src/hexagon/StereoDepthDsp/hexagon_build/libStereoDepthDsp_skel.so")

target_link_libraries(depthai-device-core PRIVATE pipeline.node.IMU)

# ---------------------------------------------------------------------
# Private module: licensee-only Qualcomm/ST code behind the public interfaces
# ---------------------------------------------------------------------
if(DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR)
    add_subdirectory("${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}" "${DEPTHAI_DEVICE_BINARY_DIR}/depthai-device-rvc4-vendor")
    # One direction only: the core library uses the implementations of the private module, and the module
    # compiles against depthai-device-accel-api.
    target_link_libraries(depthai-device-core PRIVATE depthai-device-rvc4-vendor)
    # The IMU node library uses the SensorDirectChannel implementation of the private module.
    target_link_libraries(pipeline.node.IMU PRIVATE depthai-device-rvc4-vendor)
    # VD55H1 ToF sensor pipeline: the separate archive of the private module when it is built with the ST wrapper
    if(DEPTHAI_DEVICE_TOF_VD55H1)
        target_link_libraries(depthai-device-core PRIVATE depthai-device-rvc4-vendor-tof)
    endif()
else()
    message(FATAL_ERROR "DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR is not set and no sibling depthai-device-vendor-fw checkout was found. "
                        "The private module is required to build the firmware.")
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
add_custom_command(TARGET ${TARGET_DEVICE_EXECUTABLE} POST_BUILD
    COMMAND $ENV{OBJCOPY} --only-keep-debug "$<TARGET_FILE:${TARGET_DEVICE_EXECUTABLE}>" "$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/$<TARGET_FILE_BASE_NAME:${TARGET_DEVICE_EXECUTABLE}>.debug"
    COMMAND $ENV{STRIP} --strip-unneeded "$<TARGET_FILE:${TARGET_DEVICE_EXECUTABLE}>"
    COMMAND $ENV{OBJCOPY} --add-gnu-debuglink="$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/$<TARGET_FILE_BASE_NAME:${TARGET_DEVICE_EXECUTABLE}>.debug" "$<TARGET_FILE:${TARGET_DEVICE_EXECUTABLE}>"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${TARGET_DEVICE_EXECUTABLE}>" "${DEPTHAI_DEVICE_BINARY_DIR}/$<TARGET_FILE_NAME:${TARGET_DEVICE_EXECUTABLE}>"
    ${DEPTHAI_DEVICE_DSP_SKEL_COPY_COMMANDS}
    COMMAND ${CMAKE_COMMAND} -E tar cfJ "$<TARGET_FILE_DIR:${TARGET_DEVICE_EXECUTABLE}>/depthai-device-rvc4-fwp.tar.xz"
        "${DEPTHAI_DEVICE_BINARY_DIR}/$<TARGET_FILE_NAME:${TARGET_DEVICE_EXECUTABLE}>"
        ${DEPTHAI_DEVICE_DSP_SKELS_IN_FWP}
        ${DEPTHAI_MODELS_KB_RESOURCE_LIST}
        ${DEPTHAI_CUSTOM_SCRIPTS_OUTPUT_DIR}
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
        --private-module-root "${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR}"
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
