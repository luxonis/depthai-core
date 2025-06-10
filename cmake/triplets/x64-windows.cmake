set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

if(PORT MATCHES "libusb|ffmpeg")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()

