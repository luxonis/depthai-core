set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE static)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_BUILD_TYPE release)

if(PORT MATCHES "libusb")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()

