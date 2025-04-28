set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

set(VCPKG_CMAKE_SYSTEM_NAME Darwin)
set(VCPKG_OSX_ARCHITECTURES x86_64)
set(VCPKG_OSX_DEPLOYMENT_TARGET "11")

set(VCPKG_BUILD_TYPE release)

# Add ffmpeg after the shared libraries become relocatable
if(PORT MATCHES "libusb")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()

set(VCPKG_CXX_FLAGS "-fsanitize=address,undefined")
set(VCPKG_C_FLAGS "-fsanitize=address,undefined")
set(VCPKG_LINKER_FLAGS "-fsanitize=address,undefined")
