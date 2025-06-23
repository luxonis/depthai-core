cmake_minimum_required(VERSION 3.16)
project(uvc-gadget VERSION 0.4.0 LANGUAGES C CXX)

# Version generation
execute_process(
    COMMAND ${CMAKE_SOURCE_DIR}/scripts/gen-version.sh ${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE uvc_gadget_git_version
    RESULT_VARIABLE _gen_version_result
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT uvc_gadget_git_version)
    set(uvc_gadget_git_version ${PROJECT_VERSION})
endif()

string(REGEX REPLACE "^([^+]+).*" "\\1" uvc_gadget_version ${uvc_gadget_git_version})

if(NOT uvc_gadget_version STREQUAL PROJECT_VERSION)
    message(WARNING "The sources disagree about the version: ${uvc_gadget_version} != ${PROJECT_VERSION}")
    # For simplicity, we're keeping the full git version in uvc_gadget_git_version.
endif()

message(STATUS "UVC Gadget Version: ${uvc_gadget_git_version}")

# Configure the build environment
include(GNUInstallDirs)
# find_package(libcamera QUIET)
# find_package(JPEG QUIET)
# find_package(Threads QUIET)

# Configure header
# set(CONF_HEADERS config.h.in)
# configure_file(${CMAKE_SOURCE_DIR}/config.h.in ${CMAKE_BINARY_DIR}/config.h @ONLY)
# list(APPEND PROJECT_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR})

# Subdirectories
add_subdirectory(lib)

# Include directories
# target_include_directories(uvcgadget
#     PUBLIC
#     # this lets the .c files in lib/ find their headers:
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
#     # your public API headers:
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/uvcgadget>
#     # if you generate config.h into the build tree:
#     $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
#     # for installs:
#     $<INSTALL_INTERFACE:include>
# )
# target_include_directories(uvc-gadget
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/uvcgadget>
# )

