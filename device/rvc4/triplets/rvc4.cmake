set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

set(VCPKG_CMAKE_SYSTEM_NAME Linux)

set(VCPKG_BUILD_TYPE release)

# Add ffmpeg after the shared libraries become relocatable
if(PORT MATCHES "libusb")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()

# CMakeLists.txt exports the toolchain that it selected, because vcpkg reads this file in its own process.
if("$ENV{DEPTHAI_DEVICE_RVC4_TOOLCHAIN}" STREQUAL "gnu")
    set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../toolchains/rvc4-gnu.cmake")
else()
    set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../toolchains/OEToolchainConfig.cmake")
endif()
