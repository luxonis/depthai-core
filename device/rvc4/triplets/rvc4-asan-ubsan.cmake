set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

set(VCPKG_CMAKE_SYSTEM_NAME Linux)

set(VCPKG_BUILD_TYPE release)

# Add ffmpeg after the shared libraries become relocatable
if(PORT MATCHES "libusb")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()

set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../toolchains/OEToolchainConfig.cmake")

set(VCPKG_ASAN_UBSAN_FLAGS
    "-fno-omit-frame-pointer"
    "-fno-optimize-sibling-calls"
    "-fsanitize=address,undefined,leak"
    "-fsanitize-address-use-after-scope"
    "-fsanitize=float-divide-by-zero"
    "-fno-sanitize=pointer-overflow"
    "-fno-sanitize-recover=all"
)
string(JOIN " " VCPKG_ASAN_UBSAN_FLAGS ${VCPKG_ASAN_UBSAN_FLAGS})

set(VCPKG_CXX_FLAGS "${VCPKG_ASAN_UBSAN_FLAGS}")
set(VCPKG_C_FLAGS "${VCPKG_ASAN_UBSAN_FLAGS}")
set(VCPKG_LINKER_FLAGS "${VCPKG_ASAN_UBSAN_FLAGS}")
