# Toolchain for the RVC4 firmware: a GNU cross compiler for aarch64 Linux (the public build image has GCC 11.4 of Ubuntu,
# aarch64-linux-gnu) with a plain sysroot
# directory, as in the public build container (docker/Dockerfile.sdk). No OpenEmbedded SDK environment is needed.
#
# Inputs, each read from the CMake variable first and from the environment variable of the same name second
# (CMakeLists.txt exports both, because the vcpkg port builds load this file too and see only the environment):
#   DEPTHAI_DEVICE_RVC4_SYSROOT            sysroot directory with usr/include, usr/lib and lib
#   DEPTHAI_DEVICE_RVC4_TOOLCHAIN_PREFIX   compiler prefix, for example /usr/bin/aarch64-linux-gnu-
#                                          default: aarch64-linux-gnu-gcc, then aarch64-none-linux-gnu-gcc, on PATH
#
# The flags are the ones of the Luxonis OS SDK (OpenEmbedded) build, so that both toolchains build the firmware the
# same way. They go into the *_FLAGS_INIT variables, so the cache keeps control of CMAKE_<LANG>_FLAGS.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

if(DEPTHAI_DEVICE_RVC4_SYSROOT)
    set(_rvc4_sysroot "${DEPTHAI_DEVICE_RVC4_SYSROOT}")
elseif(NOT "$ENV{DEPTHAI_DEVICE_RVC4_SYSROOT}" STREQUAL "")
    set(_rvc4_sysroot "$ENV{DEPTHAI_DEVICE_RVC4_SYSROOT}")
else()
    message(FATAL_ERROR "rvc4-gnu.cmake: no sysroot. Set DEPTHAI_DEVICE_RVC4_SYSROOT (CMake or environment variable) "
                        "to the RVC4 sysroot directory.")
endif()
if(NOT EXISTS "${_rvc4_sysroot}/usr/lib" OR NOT EXISTS "${_rvc4_sysroot}/usr/include")
    message(FATAL_ERROR "rvc4-gnu.cmake: '${_rvc4_sysroot}' is not a sysroot (no usr/lib or usr/include)")
endif()
set(CMAKE_SYSROOT "${_rvc4_sysroot}")

list(APPEND CMAKE_FIND_ROOT_PATH "${_rvc4_sysroot}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

if(DEPTHAI_DEVICE_RVC4_TOOLCHAIN_PREFIX)
    set(_rvc4_prefix "${DEPTHAI_DEVICE_RVC4_TOOLCHAIN_PREFIX}")
elseif(NOT "$ENV{DEPTHAI_DEVICE_RVC4_TOOLCHAIN_PREFIX}" STREQUAL "")
    set(_rvc4_prefix "$ENV{DEPTHAI_DEVICE_RVC4_TOOLCHAIN_PREFIX}")
else()
    find_program(_rvc4_gnu_gcc NAMES aarch64-linux-gnu-gcc aarch64-none-linux-gnu-gcc)
    if(NOT _rvc4_gnu_gcc)
        message(FATAL_ERROR "rvc4-gnu.cmake: no aarch64-linux-gnu-gcc or aarch64-none-linux-gnu-gcc on PATH. Install a GCC 11 "
                            "cross compiler (Ubuntu 22.04: g++-11-aarch64-linux-gnu) or set DEPTHAI_DEVICE_RVC4_TOOLCHAIN_PREFIX.")
    endif()
    string(REGEX REPLACE "gcc$" "" _rvc4_prefix "${_rvc4_gnu_gcc}")
endif()
if(NOT EXISTS "${_rvc4_prefix}gcc" OR NOT EXISTS "${_rvc4_prefix}g++")
    message(FATAL_ERROR "rvc4-gnu.cmake: '${_rvc4_prefix}gcc' or '${_rvc4_prefix}g++' not found")
endif()

set(CMAKE_C_COMPILER "${_rvc4_prefix}gcc")
set(CMAKE_CXX_COMPILER "${_rvc4_prefix}g++")
set(CMAKE_ASM_COMPILER "${_rvc4_prefix}gcc")
set(CMAKE_AR "${_rvc4_prefix}ar" CACHE FILEPATH "Archiver")
set(CMAKE_RANLIB "${_rvc4_prefix}ranlib" CACHE FILEPATH "Archive indexer")
set(CMAKE_OBJCOPY "${_rvc4_prefix}objcopy" CACHE FILEPATH "objcopy")
set(CMAKE_OBJDUMP "${_rvc4_prefix}objdump" CACHE FILEPATH "objdump")
set(CMAKE_STRIP "${_rvc4_prefix}strip" CACHE FILEPATH "strip")
set(CMAKE_NM "${_rvc4_prefix}nm" CACHE FILEPATH "nm")
set(CMAKE_READELF "${_rvc4_prefix}readelf" CACHE FILEPATH "readelf")

# CMake folds CC, CFLAGS, LDFLAGS and the like into the compiler and flag variables; this file defines them itself.
foreach(_rvc4_var CC CXX CPP ASM CFLAGS CXXFLAGS CPPFLAGS ASMFLAGS LDFLAGS)
    unset(ENV{${_rvc4_var}})
endforeach()

# Generic armv8-a, as the OS build: no -march or -mcpu. The cross compiler of Ubuntu turns on stack clash protection by
# default; the compiler of the OS build does not, so it is turned off.
set(_rvc4_compile_flags
    -mbranch-protection=standard -fstack-protector-strong -fno-stack-clash-protection -fPIE -D_FORTIFY_SOURCE=2 -Wa,--noexecstack
    -Wformat -Wformat-security -Werror=format-security
    -O2 -g -fexpensive-optimizations -frename-registers -ftree-vectorize -finline-functions -finline-limit=64
    -Wno-error=maybe-uninitialized -Wno-error=unused-result)
set(_rvc4_link_flags -Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed -Wl,-z,relro,-z,now,-z,noexecstack -Wl,--build-id=sha1)
# The GCC runtime objects of the device OS (crtbegin*.o, crtend*.o, libgcc.a, libgcc_eh.a in usr/lib/<target>/<gcc version>/)
# carry the AArch64 BTI/PAC property note that the objects of the cross compiler lack. The linker enables branch
# protection only when every input has the note, so -B makes the compiler driver take the objects of the OS.
file(GLOB _rvc4_gcc_runtime_dirs LIST_DIRECTORIES true "${_rvc4_sysroot}/usr/lib/aarch64-*-linux/*")
set(_rvc4_gcc_runtime_flag "")
foreach(_rvc4_dir IN LISTS _rvc4_gcc_runtime_dirs)
    if(EXISTS "${_rvc4_dir}/crtbeginS.o" AND EXISTS "${_rvc4_dir}/libgcc.a")
        set(_rvc4_gcc_runtime_flag "-B${_rvc4_dir}/")
    endif()
endforeach()
string(JOIN " " _rvc4_compile_flags ${_rvc4_compile_flags})
string(JOIN " " _rvc4_link_flags ${_rvc4_link_flags})

# The flags of the vcpkg triplet (VCPKG_C_FLAGS and the like) are not applied, as in OEToolchainConfig.cmake: the
# rvc4-tsan and rvc4-asan-ubsan triplets set the sanitizer flags there, and both toolchains build the dependencies
# without them. CMakeLists.txt gives the sanitizer flags to the firmware targets.
set(CMAKE_C_FLAGS_INIT "${_rvc4_compile_flags} ${_rvc4_gcc_runtime_flag}")
set(CMAKE_CXX_FLAGS_INIT "${_rvc4_compile_flags} ${_rvc4_gcc_runtime_flag}")
set(CMAKE_ASM_FLAGS_INIT "${_rvc4_compile_flags}")
# Executables are position independent, as with the SDK compiler (it defaults to -pie)
set(CMAKE_EXE_LINKER_FLAGS_INIT "-pie ${_rvc4_link_flags}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_rvc4_link_flags}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_rvc4_link_flags}")

# try_compile projects load this file again and need the same inputs
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES DEPTHAI_DEVICE_RVC4_SYSROOT DEPTHAI_DEVICE_RVC4_TOOLCHAIN_PREFIX)
