set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE static)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_BUILD_TYPE release)

if(PORT MATCHES "libusb|ffmpeg")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()

set(VCPKG_CXX_FLAGS "-fsanitize=thread")
set(VCPKG_C_FLAGS "-fsanitize=thread")
set(VCPKG_LINKER_FLAGS "-fsanitize=thread")

