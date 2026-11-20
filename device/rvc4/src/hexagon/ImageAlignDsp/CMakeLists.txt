cmake_minimum_required(VERSION 3.14.3)

project(ImageAlignDsp C CXX ASM)
enable_language(ASM)

# Set DSP to be used
set(DSP_TYPE CDSP)

# Use if-else logic
if(DSP_TYPE STREQUAL "CDSP")
    message("-- CDSP is used")
    set(DOMAIN_ID 3)
elseif(DSP_TYPE STREQUAL "ADSP")
    message("-- ADSP is used")
    set(DOMAIN_ID 0)
elseif(DSP_TYPE STREQUAL "SDSP")
    message("-- SDSP is used")
    set(DOMAIN_ID 1)
elseif(DSP_TYPE STREQUAL "MDSP")
    message("-- MDSP is used")
    set(DOMAIN_ID 2)
else()
    message(FATAL_ERROR "-- No DSP selected.")
endif()

set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
set(DSP_ARCH v73)
set(PREBUILT_LIB_DIR android_aarch64)

cmake_policy(SET CMP0115 OLD)

# Interface, shared headers and make fragments: the skeleton is rebuilt when they change
file(GLOB _ImageAlignDsp_extra_deps CONFIGURE_DEPENDS inc/*.idl shared/*.h dsp/*.min)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/hexagon_build/libImageAlignDsp_skel.so
    COMMAND ${CMAKE_COMMAND} -E env "DEPTHAI_DEVICE_ROOT=${DEPTHAI_DEVICE_SOURCE_DIR}" "DEPTHAI_CORE_ROOT=${DEPTHAI_DEVICE_KB_CORE_PATH}" ./build.sh ${DSP_ARCH} ${CMAKE_CURRENT_BINARY_DIR}/hexagon_build/
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dsp
    DEPENDS dsp/src/ImageAlignDsp.cpp
    DEPENDS dsp/build.sh
    DEPENDS dsp/hexagon.min dsp/Makefile ${_ImageAlignDsp_extra_deps})

add_custom_target(dsp_ImageAlignDsp DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/hexagon_build/libImageAlignDsp_skel.so)

set(CMAKE_CXX_FLAGS -DDOMAIN_ID=${DOMAIN_ID})

include(${HEXAGON_SDK_ROOT}/build/cmake/hexagon_fun.cmake)

# Base Include dirs for the Project. The CPU side implements the C interface vendor/dai_device_dsp.h (found through
# the src/ root ../..) and includes only the C interface, the standard library, the Hexagon SDK and the generated IDL
# headers: the prebuilt dsp/ folder of the vendor module has this library.
set(common_incs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    ${CMAKE_CURRENT_SOURCE_DIR}/shared/
    ${CMAKE_CURRENT_BINARY_DIR}/
    ${HEXAGON_SDK_ROOT}/incs/
    ${HEXAGON_SDK_ROOT}/incs/stddef/
    ${HEXAGON_SDK_ROOT}/rtos/qurt/
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${GST_INCLUDE_DIRS}
)

# ################################################
# HLOS Dynamic Stub Library
# ################################################
# No object file is in both DSP libraries: the common objects (utilities/DspCommon.cpp with the log and
# dai_device_dsp_set_log, and utilities/utilities.cpp) are in StereoDepthDsp, and ImageAlignDsp links it.
set(ImageAlignDsp_srcs
    ${CMAKE_CURRENT_BINARY_DIR}/ImageAlignDsp_stub
    cpu/ImageAlignDspInterface.cpp
)

add_library(ImageAlignDsp STATIC ${CMAKE_CURRENT_BINARY_DIR}/ImageAlignDsp_stub ${ImageAlignDsp_srcs})
target_link_libraries(ImageAlignDsp PRIVATE StereoDepthDsp cdsprpc)

add_dependencies(ImageAlignDsp dsp_ImageAlignDsp)

# Add common HLOS-specific linker options
link_options(${TARGET_DEVICE_KB_NAME})

# Add include directories to the build
include_directories(${common_incs})

# Build interface files using user IDL
build_idl(inc/ImageAlignDsp.idl dsp_ImageAlignDsp)

# Find the DSP from provided domain value, default is CDSP
choose_dsprpc(${DSP_TYPE} dsprpc)

# Copy binaries from ship to CMAKE_CURRENT_BINARY_DIR folder
copy_binaries(ImageAlignDsp)
