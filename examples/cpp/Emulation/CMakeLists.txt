project(emulator_examples)
cmake_minimum_required(VERSION 3.10)

set(EMULATION_BUILD_DIR "/home/aljaz/work-luxonis/holoscan-sensor-bridge/build" CACHE PATH "Path to holoscan-sensor-bridge build directory")
set(EMULATION_INCLUDE_DIR
    "/home/aljaz/work-luxonis/holoscan-sensor-bridge/src/hololink/emulation"
    CACHE PATH "Path to holoscan-sensor-bridge emulation headers"
)
set(EMULATION_ROOT_INCLUDE_DIR
    "/home/aljaz/work-luxonis/holoscan-sensor-bridge/src"
    CACHE PATH "Path to holoscan-sensor-bridge source root (for hololink/core includes)"
)
set(EMULATION_DLPACK_INCLUDE_DIR
    "${EMULATION_BUILD_DIR}/_deps/dlpack/include"
    CACHE PATH "Optional path to external dlpack headers used by emulation headers"
)

set(EMULATION_LIB "${EMULATION_BUILD_DIR}/libemulation.a")
set(EMULATION_ROCE_LIB "${EMULATION_BUILD_DIR}/libemulationroce.a")

if(NOT EXISTS "${EMULATION_LIB}")
    message(FATAL_ERROR "Missing emulation library: ${EMULATION_LIB}")
endif()

if(NOT EXISTS "${EMULATION_INCLUDE_DIR}/hsb_emulator.hpp")
    message(FATAL_ERROR "Missing header: ${EMULATION_INCLUDE_DIR}/hsb_emulator.hpp")
endif()
if(NOT EXISTS "${EMULATION_ROOT_INCLUDE_DIR}/hololink/core/hololink.hpp")
    message(FATAL_ERROR "Missing header: ${EMULATION_ROOT_INCLUDE_DIR}/hololink/core/hololink.hpp")
endif()

dai_add_example(emulator emulator.cpp ON OFF)

target_include_directories(emulator PRIVATE
    "${EMULATION_INCLUDE_DIR}"
    "${EMULATION_ROOT_INCLUDE_DIR}"
)

if(EXISTS "${EMULATION_DLPACK_INCLUDE_DIR}/dlpack/dlpack.h")
    target_include_directories(emulator PRIVATE "${EMULATION_DLPACK_INCLUDE_DIR}")
endif()

if(EXISTS "${EMULATION_ROCE_LIB}")
    find_package(ZLIB REQUIRED)
    find_package(CUDAToolkit QUIET)
    if(NOT TARGET CUDA::cudart)
        message(FATAL_ERROR "Found ${EMULATION_ROCE_LIB}, but CUDA::cudart is unavailable. Install CUDAToolkit or rebuild emulation without RoCE.")
    endif()

    # Keep dependent static libraries ordered for one-pass linkers.
    target_link_libraries(emulator PRIVATE
        "${EMULATION_ROCE_LIB}"
        "${EMULATION_LIB}"
        ZLIB::ZLIB
        CUDA::cudart
    )
else()
    target_link_libraries(emulator PRIVATE
        "${EMULATION_LIB}"
    )
endif()
