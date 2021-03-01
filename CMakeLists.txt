cmake_minimum_required(VERSION 3.4) # For Hunter

# MSVC variable isn't available before 'project' call
# Generalize to Win32 platform for now 
if(NOT WIN32)
    set(HUNTER_CONFIGURATION_TYPES "Release" CACHE STRING "Hunter dependencies list of build configurations")
endif()

# Read the BUILD_SHARED_LIBS option and select PIC toolchain
if(BUILD_SHARED_LIBS)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain/pic.cmake" CACHE STRING "")
    # Specify exporting all symbols on Windows
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON CACHE BOOL "")
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

# Create depthai project
project(depthai VERSION "2.0.0" LANGUAGES CXX C)
set(DEPTHAI_VERSION ${PROJECT_VERSION} PARENT_SCOPE)

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
option(DEPTHAI_CLANG_FORMAT "Enable clang-format target"                  ON )
option(DEPTHAI_CLANG_TIDY "Enable clang-tidy checks during compilation" OFF)
option(DEPTHAI_SANITIZE "Enable Address and Undefined sanitizers for library, examples and tests" OFF)

# Should install depthai core libraries
option(DEPTHAI_INSTALL   "Enable install target for depthai-core targets" ON)

# Debug option
set(DEPTHAI_XLINK_LOCAL "" CACHE STRING "Path to local XLink source to use instead of Hunter")

### Constants
set(PROJECT_EXPORT_GROUP "${PROJECT_NAME}Targets")

## Check if cloned or sources
find_package(Git)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --is-inside-work-tree
        WORKING_DIRECTORY "${DEPTHAI_SHARED_FOLDER}"
        RESULT_VARIABLE DEPTHAI_DOWNLOADED_SOURCES
        OUTPUT_QUIET ERROR_QUIET
    )
endif()

### Get and find dependencies

# Include project dependencies
include(depthaiDependencies)

# Add threads preference
set(THREADS_PREFER_PTHREAD_FLAG ON)

# Add depthai-shared, and definitions that it is PC side
include(${CMAKE_CURRENT_LIST_DIR}/shared/depthai-shared.cmake)

# Add depthai-bootloader-shared
include(${CMAKE_CURRENT_LIST_DIR}/shared/depthai-bootloader-shared.cmake)

# Add flags helpers
include(Flags)

### End of dependencies

set(TARGET_CORE_NAME ${PROJECT_NAME}-core)
set(TARGET_CORE_ALIAS core)

# Create core library
add_library(${TARGET_CORE_NAME}
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
    src/pipeline/node/DetectionNetwork.cpp
    src/pipeline/node/SystemLogger.cpp
    src/pipeline/datatype/Buffer.cpp
    src/pipeline/datatype/ImgFrame.cpp
    src/pipeline/datatype/ImageManipConfig.cpp
    src/pipeline/datatype/CameraControl.cpp
    src/pipeline/datatype/NNData.cpp
    src/pipeline/datatype/ImgDetections.cpp
    src/pipeline/datatype/SystemInformation.cpp
    src/pipeline/datatype/StreamPacketParser.cpp
    src/utility/Initialization.cpp
    src/utility/Resources.cpp
    src/xlink/XLinkConnection.cpp
    src/xlink/XLinkStream.cpp
    src/openvino/OpenVINO.cpp
    src/openvino/BlobReader.cpp
    src/bspatch/bspatch.c
)
add_library("${PROJECT_NAME}::${TARGET_CORE_ALIAS}" ALIAS ${TARGET_CORE_NAME})

# Add default flags to core
add_default_flags(${TARGET_CORE_NAME})

# And clang-tidy and format
if(DEPTHAI_CLANG_TIDY)
    include(ClangTidy)
    target_clangtidy_setup(${TARGET_CORE_NAME})
endif()

# Set compiler features (c++11), and disables extensions (g++11)
set_property(TARGET ${TARGET_CORE_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${TARGET_CORE_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${TARGET_CORE_NAME} PROPERTY CXX_EXTENSIONS OFF)

# First specify options
option(DEPTHAI_BUILD_TESTS "Build tests" OFF)
option(DEPTHAI_BUILD_EXAMPLES "Build examples - Requires OpenCV library to be installed" OFF)
option(DEPTHAI_BUILD_DOCS "Build documentation - requires doxygen to be installed" OFF)
option(DEPTHAI_OPENCV_SUPPORT "Enable optional OpenCV support" ON)


option(DEPTHAI_BINARIES_RESOURCE_COMPILE "Compile Depthai device side binaries into library" ON)
option(DEPTHAI_USB2_PATCH_ONLY_MODE "Use patch file and full depthai.cmd binary for usb2 mode" ON)
option(DEPTHAI_CMD_PATH "Use local path for depthai.cmd instead of downloading" OFF)
if(DEPTHAI_USB2_PATCH_ONLY_MODE)
    option(DEPTHAI_USB2_PATCH_PATH "Use local path for depthai-usb2-patch.patch instead of downloading" OFF)
else()
    option(DEPTHAI_USB2_CMD_PATH "Use local path for depthai-usb2.cmd instead of downloading" OFF)
endif()

if(DEPTHAI_USB2_PATCH_ONLY_MODE)
    message(STATUS "Compiling ${TARGET_CORE_NAME} resources in PATCH_ONLY mode")
else()
    message(STATUS "Compiling ${TARGET_CORE_NAME} depthai and depthai-usb2 resources")
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
    target_link_libraries(${TARGET_CORE_NAME} PRIVATE "${DEPTHAI_RESOURCE_LIBRARY_NAME}")

    # Set define that binaries are resource compiled
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_RESOURCE_COMPILED_BINARIES)

else()
    # TODO
    # Don't add RC and don't resource compile the binaries

endif()


# Configure build information (version, ...)
configure_file("${CMAKE_CURRENT_LIST_DIR}/cmake/version.hpp.in" "${CMAKE_CURRENT_LIST_DIR}/include/depthai/build/version.hpp")

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
        "$<BUILD_INTERFACE:${DEPTHAI_BOOTLOADER_SHARED_3RDPARTY_INCLUDE}>"
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
        XLink
    PRIVATE
        Threads::Threads
        BZip2::bz2
        FP16::fp16
        archive_static
        spdlog::spdlog
)

# Add compile definitions
target_compile_definitions(${TARGET_CORE_NAME} 
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
    target_compile_definitions(${TARGET_CORE_NAME} PRIVATE DEPTHAI_PATCH_ONLY_MODE)
endif()

########################
# OpenCV Support
########################
set(TARGET_OPENCV_NAME ${PROJECT_NAME}-opencv)   
set(TARGET_OPENCV_ALIAS opencv)
if(DEPTHAI_OPENCV_SUPPORT)
    # Check if required libraries are available
    set(REQUIRED_OPENCV_LIBRARIES "opencv_core" "opencv_imgproc")
    set(OPENCV_SUPPORT_AVAILABLE ${OpenCV_FOUND})
    foreach(lib ${REQUIRED_OPENCV_LIBRARIES})
        if(NOT (lib IN_LIST OpenCV_LIBS))
            set(OPENCV_SUPPORT_AVAILABLE FALSE)
        endif()
    endforeach()

    if(OPENCV_SUPPORT_AVAILABLE)

        # Add depthai-core-opencv library and depthai::core::opencv alias
        add_library(${TARGET_OPENCV_NAME} src/opencv/ImgFrame.cpp)
        add_library("${PROJECT_NAME}::${TARGET_OPENCV_ALIAS}" ALIAS ${TARGET_OPENCV_NAME})

        # Add default flags
        add_default_flags(${TARGET_OPENCV_NAME})
        add_flag(${TARGET_OPENCV_NAME} -Wno-switch-enum) 
        
        # Link to OpenCV (publically)
        target_link_libraries(${TARGET_OPENCV_NAME} PUBLIC ${REQUIRED_OPENCV_LIBRARIES})
    
        # Add public compile definition indicating that OpenCV support is available
        target_compile_definitions(${TARGET_OPENCV_NAME} PUBLIC DEPTHAI_OPENCV_SUPPORT)

        # Add public dependency to depthai::core library
        target_link_libraries(${TARGET_OPENCV_NAME} PUBLIC ${TARGET_CORE_NAME})

        # Add to clangformat target
        target_clangformat_setup(${TARGET_OPENCV_NAME} "")

        # Install
        # Export targets to export group
        include(GNUInstallDirs)
        install(
            TARGETS ${TARGET_OPENCV_NAME} EXPORT ${PROJECT_EXPORT_GROUP}
            RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" 
            LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" 
            ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        )

    else()
        message(STATUS "OpenCV or required libraries (${REQUIRED_OPENCV_LIBRARIES}) not found. OpenCV Support disabled")
    endif()
endif()

########################
# Sanitizers
########################
if(DEPTHAI_SANITIZE)
    set(SANITIZE_ADDRESS ON CACHE BOOL "Enable AddressSanitizer for sanitized targets." FORCE)
    set(SANITIZE_UNDEFINED ON CACHE BOOL "Enable UndefinedBehaviorSanitizer for sanitized targets." FORCE)
endif()
find_package(Sanitizers)
add_sanitizers(${TARGET_CORE_NAME} ${TARGET_OPENCV_NAME})

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

########################
# Documentation
########################
if (DEPTHAI_BUILD_DOCS)
    add_subdirectory(docs)
endif()

########################
# INSTALL steps
########################
if(DEPTHAI_INSTALL)
    include(GNUInstallDirs)

    # Export targets to export group
    if(BUILD_SHARED_LIBS)
        set(targets_to_install ${TARGET_CORE_NAME})
    else()
        set(targets_to_install ${TARGET_CORE_NAME} ${DEPTHAI_RESOURCE_LIBRARY_NAME} cmrc-base)
    endif()
    install(
        TARGETS ${targets_to_install}
        EXPORT ${PROJECT_EXPORT_GROUP}
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" 
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

    # Install Hunter dependencies
    if(BUILD_SHARED_LIBS)
        # Install only PUBLIC dependencies
        install(DIRECTORY "${HUNTER_INSTALL_PREFIX}/include/XLink" DESTINATION include/XLink)
        install(DIRECTORY "${HUNTER_INSTALL_PREFIX}/include/nlohmann" DESTINATION include/nlohmann)
        install(DIRECTORY "${HUNTER_INSTALL_PREFIX}/lib/cmake/XLink" DESTINATION lib/cmake/XLink)
        install(DIRECTORY "${HUNTER_INSTALL_PREFIX}/lib/cmake/nlohmann_json" DESTINATION lib/cmake/nlohmann_json)
    else()
        # Install all dependencies
        install(DIRECTORY "${HUNTER_INSTALL_PREFIX}/" DESTINATION .)
    endif()

    # Add resource installation if not RC'd
    if(NOT DEPTHAI_BINARIES_RESOURCE_COMPILE)
        install(DIRECTORY "${DEPTHAI_RESOURCES_OUTPUT_DIR}/" DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/depthai")
    endif()

    # Write project version
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        ${PROJECT_NAME}ConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    # Install export group (targets)
    install(EXPORT ${PROJECT_EXPORT_GROUP}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
    )

    # Dependencies file
    install(FILES 
        "cmake/${PROJECT_NAME}Dependencies.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
    )

    # Configure config file
    set(INSTALL_LIB_CMAKE_DIR "lib/cmake/${PROJECT_NAME}")
    file(RELATIVE_PATH RELATIVE_INSTALLTION_PATH "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_PREFIX}/${INSTALL_LIB_CMAKE_DIR}" "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_PREFIX}")
    configure_file(cmake/${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake @ONLY)
    install(FILES 
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${INSTALL_LIB_CMAKE_DIR}
    )

endif()
