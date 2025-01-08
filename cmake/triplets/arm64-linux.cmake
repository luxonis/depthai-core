set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_BUILD_TYPE release)
set(PORT_DEBUG ON)

if(PORT MATCHES "libusb|ffmpeg")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
    set(VCPKG_FIXUP_ELF_RPATH ON)
endif()
if(PORT MATCHES "vulkan-loader")
    # set env variable for pkg config
    set(ENV{PKG_CONFIG_DEBUG_SPEW} 1)
    set(CMAKE_FIND_DEBUG_MODE ON)
    set(VCPKG_CMAKE_CONFIGURE_OPTIONS "-DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config")

endif()

set(VCPKG_CMAKE_SYSTEM_NAME Linux)
