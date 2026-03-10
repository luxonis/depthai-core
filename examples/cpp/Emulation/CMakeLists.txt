cmake_minimum_required(VERSION 3.10)
project(emulator_examples)

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(DEPTHAI_TARGET depthai::core)
if(TARGET depthai::opencv)
    set(DEPTHAI_TARGET depthai::opencv)
endif()

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

set(EMULATION_LIB_CANDIDATES
    "${EMULATION_BUILD_DIR}/libemulation.a"
    "${EMULATION_BUILD_DIR}/src/hololink/emulation/libemulation.a"
)
set(EMULATION_ROCE_LIB_CANDIDATES
    "${EMULATION_BUILD_DIR}/libemulationroce.a"
    "${EMULATION_BUILD_DIR}/src/hololink/emulation/libemulationroce.a"
)

unset(EMULATION_LIB)
foreach(candidate IN LISTS EMULATION_LIB_CANDIDATES)
    if(EXISTS "${candidate}")
        set(EMULATION_LIB "${candidate}")
        break()
    endif()
endforeach()

unset(EMULATION_ROCE_LIB)
foreach(candidate IN LISTS EMULATION_ROCE_LIB_CANDIDATES)
    if(EXISTS "${candidate}")
        set(EMULATION_ROCE_LIB "${candidate}")
        break()
    endif()
endforeach()

if(NOT EMULATION_LIB)
    message(FATAL_ERROR
        "Missing emulation library. Checked:\n"
        "  ${EMULATION_LIB_CANDIDATES}"
    )
endif()

if(NOT EXISTS "${EMULATION_INCLUDE_DIR}/hsb_emulator.hpp")
    message(FATAL_ERROR "Missing header: ${EMULATION_INCLUDE_DIR}/hsb_emulator.hpp")
endif()
if(NOT EXISTS "${EMULATION_ROOT_INCLUDE_DIR}/hololink/core/hololink.hpp")
    message(FATAL_ERROR "Missing header: ${EMULATION_ROOT_INCLUDE_DIR}/hololink/core/hololink.hpp")
endif()

add_executable(emulator emulator.cpp)
set_property(TARGET emulator PROPERTY CXX_STANDARD 17)
set_property(TARGET emulator PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET emulator PROPERTY CXX_EXTENSIONS OFF)

target_include_directories(emulator PRIVATE
    "${EMULATION_INCLUDE_DIR}"
    "${EMULATION_ROOT_INCLUDE_DIR}"
)

if(EXISTS "${EMULATION_DLPACK_INCLUDE_DIR}/dlpack/dlpack.h")
    target_include_directories(emulator PRIVATE "${EMULATION_DLPACK_INCLUDE_DIR}")
endif()

if(NOT EMULATION_ROCE_LIB)
    message(FATAL_ERROR
        "Missing emulation RoCE library. Checked:\n"
        "  ${EMULATION_ROCE_LIB_CANDIDATES}"
    )
endif()

find_package(ZLIB REQUIRED)
find_package(CUDAToolkit QUIET)

# Keep dependent static libraries ordered for one-pass linkers.
target_link_libraries(emulator PRIVATE
    ${DEPTHAI_TARGET}
    Threads::Threads
    ${OpenCV_LIBS}
    "${EMULATION_ROCE_LIB}"
    "${EMULATION_LIB}"
    ZLIB::ZLIB
)

if(TARGET CUDA::cudart)
    target_link_libraries(emulator PRIVATE CUDA::cudart)
endif()
