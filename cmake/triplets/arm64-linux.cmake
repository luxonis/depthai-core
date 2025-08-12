set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(PORT_DEBUG ON)

if(PORT MATCHES "libusb|ffmpeg")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
    set(VCPKG_FIXUP_ELF_RPATH ON)
endif()
if(PORT MATCHES "vulkan-loader")
    # When building in GH Actions environment, vcpkg's pkg-conf can't find XCB/X11 libraries
    set(VCPKG_CMAKE_CONFIGURE_OPTIONS "-DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config")

endif()

set(VCPKG_CMAKE_SYSTEM_NAME Linux)
