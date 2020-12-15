cmake_minimum_required(VERSION 3.2) # For Hunter

# Set defaults
set(HUNTER_CONFIGURATION_TYPES "Release" CACHE STRING "Hunter dependencies list of build configurations")

# Read the BUILD_SHARED_LIBS option and select PIC toolchain
if(BUILD_SHARED_LIBS)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain/pic.cmake" CACHE STRING "")
endif()

include("cmake/HunterGate.cmake")
HunterGate(
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.258.tar.gz"
    SHA1 "062a19ab13ce8dffa9a882b6ce3e43bdabdf75d3"
    LOCAL # Local config for dependencies
)

# Move binary dir if windows, to shorten the path
if(WIN32)
    set(HUNTER_BINARY_DIR "${HUNTER_GATE_ROOT}/_bin" CACHE STRING "Hunter binary directory")
endif()

set(TARGET_NAME depthai-core)
project(${TARGET_NAME} VERSION "0.0.2" LANGUAGES CXX C)

# Set policies
# CMP0074 dictates that find_package searches environment variable "[packageName]_ROOT" along with regular variable [packageName]_ROOT
cmake_policy(SET CMP0074 NEW)

# Set to export compile commands for tools like clang-tidy and format
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Additional options
option(DEPTHAI_CLANG_FORMAT "Enable clang-format target"                  ON )
option(DEPTHAI_CLANG_TIDY   "Enable clang-tidy checks during compilation" OFF)

# Debug option
set(DEPTHAI_XLINK_LOCAL "" CACHE STRING "Path to local XLink source to use instead of Hunter")

# Variable to installed 3rdparty headers
set(DEPTHAI_3RDPARTY_HEADERS_PATH "depthai/3rdparty")

### Get and find dependencies

# Nlohmann JSON
hunter_add_package(nlohmann_json)
find_package(nlohmann_json CONFIG REQUIRED)

# XLink
if(DEPTHAI_XLINK_LOCAL)
    add_subdirectory("${DEPTHAI_XLINK_LOCAL}" ${CMAKE_CURRENT_BINARY_DIR}/XLink EXCLUDE_FROM_ALL)
else()
    hunter_add_package(XLink)
    find_package(XLink CONFIG REQUIRED)
endif()

# BZip2 (for bspatch)
hunter_add_package(BZip2)
find_package(BZip2 CONFIG REQUIRED)

# FP16 for conversions
hunter_add_package(FP16)
find_package(FP16 REQUIRED)

# libarchive for firmware packages
hunter_add_package(libarchive)
find_package(archive_static REQUIRED)
find_package(lzma REQUIRED)

# spdlog for library and device logging
hunter_add_package(spdlog)
find_package(spdlog CONFIG REQUIRED)

# Add depthai-shared, and definitions that it is PC side
include(${CMAKE_CURRENT_LIST_DIR}/shared/depthai-shared.cmake)

# Add depthai-bootloader-shared
include(${CMAKE_CURRENT_LIST_DIR}/shared/depthai-bootloader-shared.cmake)

# Add threads (c++)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

### End of dependencies

# Create library
add_library(${TARGET_NAME}
    # depthai-shared sources
    "${DEPTHAI_SHARED_SOURCES}"
    # depthai-bootloader-shared sources
    "${DEPTHAI_BOOTLOADER_SHARED_SOURCES}"
    # sources
    src/device/Device.cpp
    src/device/DeviceBootloader.cpp
    src/device/DataQueue.cpp
    src/device/CallbackHandler.cpp
    src/pipeline/Pipeline.cpp
    src/pipeline/AssetManager.cpp
    src/pipeline/Node.cpp
    src/pipeline/node/XLinkIn.cpp
    src/pipeline/node/XLinkOut.cpp
    src/pipeline/node/ColorCamera.cpp
    src/pipeline/node/MonoCamera.cpp
    src/pipeline/node/StereoDepth.cpp
    src/pipeline/node/NeuralNetwork.cpp
    src/pipeline/node/ImageManip.cpp
    src/pipeline/node/MyProducer.cpp
    src/pipeline/node/VideoEncoder.cpp
    src/pipeline/datatype/Buffer.cpp
    src/pipeline/datatype/ImgFrame.cpp
    src/pipeline/datatype/ImageManipConfig.cpp
    src/pipeline/datatype/NNData.cpp
    src/pipeline/datatype/StreamPacketParser.cpp
    src/utility/Initialization.cpp
    src/utility/Resources.cpp
    src/xlink/XLinkConnection.cpp
    src/openvino/OpenVINO.cpp
    src/openvino/BlobReader.cpp
    src/bspatch/bspatch.c
)

# Add aditional flags
include(cmake/flags.cmake)

# And clang-tidy and format
if(DEPTHAI_CLANG_TIDY)
    include(cmake/clang-tidy.cmake)
endif()

# Set compiler features (c++11)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)

# Add cmake folder to modules path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# First specify options
option(DEPTHAI_BUILD_TESTS "Build tests" OFF)
option(DEPTHAI_BUILD_EXAMPLES "Build examples - Requires OpenCV library to be installed" OFF)
option(DEPTHAI_TEST_EXAMPLES "Test examples - examples will be ran as a part of test suite" OFF)

option(DEPTHAI_BINARIES_RESOURCE_COMPILE "Compile Depthai device side binaries into library" ON)
option(DEPTHAI_USB2_PATCH_ONLY_MODE "Use patch file and full depthai.cmd binary for usb2 mode" ON)
option(DEPTHAI_CMD_PATH "Use local path for depthai.cmd instead of downloading" OFF)
if(DEPTHAI_USB2_PATCH_ONLY_MODE)
    option(DEPTHAI_USB2_PATCH_PATH "Use local path for depthai-usb2-patch.patch instead of downloading" OFF)
else()
    option(DEPTHAI_USB2_CMD_PATH "Use local path for depthai-usb2.cmd instead of downloading" OFF)
endif()

if(DEPTHAI_USB2_PATCH_ONLY_MODE)
    message(STATUS "Compiling ${TARGET_NAME} resources in PATCH_ONLY mode")
else()
    message(STATUS "Compiling ${TARGET_NAME} depthai and depthai-usb2 resources")
endif()

# Set constant
set(DEPTHAI_RESOURCES_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/resources")

# Include configuration
include(Depthai/DepthaiDeviceSideConfig)    # Depthai device binary commit/version configuration    
include(Depthai/DepthaiBootloaderConfig)    # Depthai bootloader binary commit/version configuration

# Include downloaders
include(DepthaiDownloader)                  # Depthai device binary downloader
include(DepthaiBootloaderDownloader)        # Depthai bootloader binary downloader


# depthai-shared enforce commit hash match if CI
if($ENV{CI})
    set(DEPTHAI_SHARED_COMMIT_HASH_ENFORCE ON)
    set(DEPTHAI_BOOTLOADER_SHARED_COMMIT_HASH_ENFORCE ON)
endif()


# Then get the Depthai device side binaries (local or download)
if(DEPTHAI_CMD_PATH OR DEPTHAI_USB2_CMD_PATH OR DEPTHAI_USB2_PATCH_PATH)
    # Atleast one of the paths is set. include binaries locally
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
    
    DepthaiDownload(
        "${DEPTHAI_SHARED_COMMIT_HASH}" "${DEPTHAI_SHARED_COMMIT_HASH_ENFORCE}"
        PATCH_ONLY ${DEPTHAI_USB2_PATCH_ONLY_MODE}
        "${DEPTHAI_RESOURCES_OUTPUT_DIR}"            # Output folder 
        DEPTHAI_RESOURCE_LIST                       # List of output resources
        "${DEPTHAI_DEVICE_SIDE_MATURITY}"           # Maturity
        "${DEPTHAI_DEVICE_SIDE_COMMIT}"             # commit hash
        "${DEPTHAI_DEVICE_SIDE_VERSION}"            # Optional version
    )
endif()
list(APPEND RESOURCE_COMPILED_FILES ${DEPTHAI_RESOURCE_LIST})

# Add bootloader
DepthaiBootloaderDownload(
    "${DEPTHAI_BOOTLOADER_SHARED_COMMIT_HASH}" "${DEPTHAI_BOOTLOADER_SHARED_COMMIT_HASH_ENFORCE}"
    "${DEPTHAI_RESOURCES_OUTPUT_DIR}"                # Output folder 
    DEPTHAI_BOOTLOADER_RESOURCE_LIST                # List of output resources
    "${DEPTHAI_BOOTLOADER_MATURITY}"                # Maturity
    "${DEPTHAI_BOOTLOADER_VERSION}"                 # if maturity == snapshot -> hash else version                  
)
list(APPEND RESOURCE_COMPILED_FILES ${DEPTHAI_BOOTLOADER_RESOURCE_LIST})

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
    target_link_libraries(${TARGET_NAME} PRIVATE "${DEPTHAI_RESOURCE_LIBRARY_NAME}")

    # Set define that binaries are resource compiled
    target_compile_definitions(${TARGET_NAME} PRIVATE DEPTHAI_RESOURCE_COMPILED_BINARIES)

else()
    # TODO
    # Don't add RC and don't resource compile the binaries

endif()


# Configure build information (version, ...)
configure_file("${CMAKE_CURRENT_LIST_DIR}/cmake/version.hpp.in" "${CMAKE_CURRENT_LIST_DIR}/include/depthai/build/version.hpp")

# Add include directories
target_include_directories(${TARGET_NAME}   
    PUBLIC
        # Relative path to include directories after installed
        "$<INSTALL_INTERFACE:include>"
        "$<INSTALL_INTERFACE:include/${DEPTHAI_3RDPARTY_HEADERS_PATH}>"
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

target_include_directories(${TARGET_NAME} SYSTEM 
    PUBLIC
        "$<BUILD_INTERFACE:${DEPTHAI_SHARED_3RDPARTY_INCLUDE}>"
        "$<BUILD_INTERFACE:${DEPTHAI_BOOTLOADER_SHARED_3RDPARTY_INCLUDE}>"
)


# Add clang format after specifying include directories
if(DEPTHAI_CLANG_FORMAT)
    # HEADER DIRECTORIES
    set(header_dirs "${CMAKE_CURRENT_LIST_DIR}/include;${DEPTHAI_SHARED_PUBLIC_INCLUDE};${DEPTHAI_SHARED_INCLUDE}")
    include(cmake/clang-format.cmake)
    target_clangformat_setup(${TARGET_NAME} "${header_dirs}")
endif()



# link libraries
target_link_libraries(${TARGET_NAME}
    PUBLIC
        nlohmann_json::nlohmann_json
        XLink
    PRIVATE
        Threads::Threads
        BZip2::bz2
        FP16::fp16
        archive_static
        spdlog::spdlog
)

# Add compile definitions
target_compile_definitions(${TARGET_NAME} 
    PRIVATE 
        # XLink required define
        __PC__
        # Add depthai-device version
        DEPTHAI_DEVICE_VERSION="${DEPTHAI_DEVICE_SIDE_COMMIT}"
        # Add if using depthai device FWP
        DEPTHAI_RESOURCES_TAR_XZ
        # Add depthai-bootloader version
        DEPTHAI_BOOTLOADER_VERSION="${DEPTHAI_BOOTLOADER_VERSION}"
)

# Add patch only mode definition
if(DEPTHAI_USB2_PATCH_ONLY_MODE)
    target_compile_definitions(${TARGET_NAME} PRIVATE DEPTHAI_PATCH_ONLY_MODE)
endif()


########################
# Testing
########################
if(DEPTHAI_BUILD_TESTS OR DEPTHAI_TEST_EXAMPLES)
    include(CTest)
    enable_testing()
endif()


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


########################
# INSTALL steps
########################
include(GNUInstallDirs)
install(
    TARGETS 
        ${TARGET_NAME} "${DEPTHAI_RESOURCE_LIBRARY_NAME}" cmrc-base
    EXPORT 
        "${TARGET_NAME}Config"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)

# Install depthai public headers
install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
# Install depthai-shared public headers
install(DIRECTORY "${DEPTHAI_SHARED_PUBLIC_INCLUDE}/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
# Install depthai-shared 3rdparty headers
install(DIRECTORY "${DEPTHAI_SHARED_3RDPARTY_INCLUDE}/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${DEPTHAI_SHARED_3RDPARTY_HEADERS_PATH}")
# Install depthai-bootloader-shared public headers
install(DIRECTORY "${DEPTHAI_BOOTLOADER_SHARED_PUBLIC_INCLUDE}/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
# Install depthai-bootloader-shared 3rdparty headers (Currently no 3rdparty)
#install(DIRECTORY "${DEPTHAI_BOOTLOADER_SHARED_3RDPARTY_INCLUDE}/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/depthai-bootloader-shared/3rdparty")

# # Install public 3rdparty dependency header (Hunter) to include/${depthai/3rdparty
# XLink
get_target_property(XLINK_INCLUDE_DIRS XLink INTERFACE_INCLUDE_DIRECTORIES)
if(XLINK_INCLUDE_DIRS)
    file(GLOB XLINK_HEADERS ${XLINK_INCLUDE_DIRS}/XLink*.h)
    install(FILES ${XLINK_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${DEPTHAI_3RDPARTY_HEADERS_PATH}")
endif()
# nlohmann json
get_target_property(NLOHMANN_JSON_INCLUDE_DIRS nlohmann_json::nlohmann_json INTERFACE_INCLUDE_DIRECTORIES)
if(NLOHMANN_JSON_INCLUDE_DIRS)
    list(GET NLOHMANN_JSON_INCLUDE_DIRS 0 JSON_DIR)
    install(FILES ${JSON_DIR}/nlohmann/json.hpp DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${DEPTHAI_3RDPARTY_HEADERS_PATH}/nlohmann")
endif()

# Add resource installation if not RC'd
if(NOT DEPTHAI_BINARIES_RESOURCE_COMPILE)
    install(DIRECTORY "${DEPTHAI_RESOURCES_OUTPUT_DIR}/" DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/depthai")
endif()

set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET_NAME}")
export(TARGETS
    ${TARGET_NAME} "${DEPTHAI_RESOURCE_LIBRARY_NAME}" cmrc-base
    FILE "${config_install_dir}/${TARGET_NAME}Config.cmake"
)
install(EXPORT
    "${TARGET_NAME}Config"
    DESTINATION "${config_install_dir}"
)
