# CMake toolchain for the RVC4 Yocto (OpenEmbedded) SDK.
# Derived from OpenEmbedded-core's OEToolchainConfig.cmake (MIT). The SDK environment
# script (environment-setup-aarch64-oe-linux) provides CC/CXX, CFLAGS/CXXFLAGS/LDFLAGS
# and OECORE_TARGET_SYSROOT.
set(CMAKE_SYSTEM_NAME Linux)

# Initial flags only: a -DCMAKE_<LANG>_FLAGS given on the command line is kept.
set(CMAKE_C_FLAGS_INIT "$ENV{CFLAGS}")
set(CMAKE_CXX_FLAGS_INIT "$ENV{CXXFLAGS}")
set(CMAKE_ASM_FLAGS_INIT "$ENV{CFLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "$ENV{LDFLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "$ENV{LDFLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "$ENV{LDFLAGS}")

set(CMAKE_SYSROOT "$ENV{OECORE_TARGET_SYSROOT}")

list(APPEND CMAKE_FIND_ROOT_PATH "$ENV{OECORE_TARGET_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_FIND_LIBRARY_CUSTOM_LIB_SUFFIX "$ENV{OE_CMAKE_FIND_LIBRARY_CUSTOM_LIB_SUFFIX}")

# Set CMAKE_SYSTEM_PROCESSOR from the sysroot name (assuming processor-distro-os).
if("$ENV{SDKTARGETSYSROOT}" MATCHES "/sysroots/([a-zA-Z0-9_-]+)-.+-.+")
    set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_MATCH_1})
endif()

# Include the toolchain configuration subscripts
file(GLOB toolchain_config_files "${CMAKE_CURRENT_LIST_FILE}.d/*.cmake")
foreach(config ${toolchain_config_files})
    include(${config})
endforeach()
