set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_BUILD_TYPE release)

if(PORT MATCHES "libusb|ffmpeg")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
    set(VCPKG_FIXUP_ELF_RPATH ON)
endif()

set(VCPKG_CMAKE_SYSTEM_NAME Linux)
set(VCPKG_LINKER_FLAGS "-Wl,-z,noseparate-code")
