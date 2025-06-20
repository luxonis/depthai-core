set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

set(VCPKG_CMAKE_SYSTEM_NAME Linux)

set(VCPKG_BUILD_TYPE release)

# Add ffmpeg after the shared libraries become relocatable
if(PORT MATCHES "libusb")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()

set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE /work/toolchains/OEToolchainConfig.cmake)
