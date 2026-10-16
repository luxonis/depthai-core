vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO libarchive/libarchive
    REF "v${VERSION}"
    SHA512 c5d85564b70e3af24edc69f34829c70ba3abcaf042ba444e9344e54e594ac88a9cc22dcd21c806e2650f1ffde71e16aeaf70e9748d7ba124207ee832938656da
    HEAD_REF master
    PATCHES
        fix-buildsystem.patch
        fix-deps.patch
)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        bzip2   ENABLE_BZip2
        bzip2   CMAKE_REQUIRE_FIND_PACKAGE_BZip2
        libxml2 ENABLE_LIBXML2
        libxml2 CMAKE_REQUIRE_FIND_PACKAGE_LibXml2
        lz4     ENABLE_LZ4
        lz4     CMAKE_REQUIRE_FIND_PACKAGE_lz4
        lzma    ENABLE_LZMA
        lzma    CMAKE_REQUIRE_FIND_PACKAGE_LibLZMA
        lzo     ENABLE_LZO
        zstd    ENABLE_ZSTD
)
# Default crypto backend is OpenSSL, but it is ignored for DARWIN
set(WRAPPER_ENABLE_OPENSSL OFF)
if(NOT "crypto" IN_LIST FEATURES)
    list(APPEND FEATURE_OPTIONS
        -DLIBMD_FOUND=FALSE
        -DENABLE_OPENSSL=OFF
    )
elseif(VCPKG_TARGET_IS_OSX)
    list(APPEND FEATURE_OPTIONS
        -DENABLE_MBEDTLS=ON
        -DENABLE_OPENSSL=OFF
        -DCMAKE_REQUIRE_FIND_PACKAGE_MbedTLS=ON
    )
else()
    set(WRAPPER_ENABLE_OPENSSL ON)
    list(APPEND FEATURE_OPTIONS
        -DCMAKE_REQUIRE_FIND_PACKAGE_OpenSSL=ON
    )
endif()

set(ENABLE_ZLIB ON)

# archive_random.c and archive_util.c always call BCrypt* on Windows, bcrypt must be linked.
# ref. https://github.com/libarchive/libarchive/issues/3375
if(VCPKG_TARGET_IS_WINDOWS)
    set(ENABLE_CNG ON)
else()
    set(ENABLE_CNG OFF)
endif()

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DENABLE_ACL=OFF
        -DENABLE_BZip2=OFF
        -DENABLE_CAT=OFF
        -DENABLE_CAT_SHARED=OFF
        -DENABLE_CNG=${ENABLE_CNG}
        -DENABLE_COVERAGE=OFF
        -DENABLE_CPIO=OFF
        -DENABLE_CPIO_SHARED=OFF
        -DENABLE_EXPAT=OFF
        -DENABLE_ICONV=OFF
        -DENABLE_INSTALL=ON
        -DENABLE_LIBB2=OFF
        -DENABLE_LIBXML2=OFF
        -DENABLE_LZ4=OFF
        -DENABLE_LZMA=ON
        -DENABLE_LZO=OFF
        -DENABLE_LibGCC=OFF
        -DENABLE_MBEDTLS=OFF
        -DENABLE_NETTLE=OFF
        -DENABLE_OPENSSL=OFF
        -DENABLE_PCREPOSIX=OFF
        -DENABLE_SAFESEH=AUTO
        -DENABLE_TAR=OFF
        -DENABLE_TAR_SHARED=OFF
        -DENABLE_TEST=OFF
        -DENABLE_UNZIP=OFF
        -DENABLE_UNZIP_SHARED=OFF
        -DENABLE_WERROR=OFF
        -DENABLE_WIN32_XMLLITE=OFF
        -DENABLE_XATTR=OFF
        -DENABLE_ZLIB=${ENABLE_ZLIB}
        -DENABLE_ZSTD=OFF
        -DZLIB_WINAPI=OFF
    MAYBE_UNUSED_VARIABLES
        ZLIB_WINAPI
)

vcpkg_cmake_install()

vcpkg_fixup_pkgconfig()

vcpkg_copy_pdbs()

configure_file("${CMAKE_CURRENT_LIST_DIR}/vcpkg-cmake-wrapper.cmake.in" "${CURRENT_PACKAGES_DIR}/share/${PORT}/vcpkg-cmake-wrapper.cmake" @ONLY)

file(REMOVE_RECURSE
      "${CURRENT_PACKAGES_DIR}/debug/include"
      "${CURRENT_PACKAGES_DIR}/debug/share"
      "${CURRENT_PACKAGES_DIR}/share/man"
)

foreach(header "include/archive.h" "include/archive_entry.h")
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/${header}" "(!defined LIBARCHIVE_STATIC)" "0")
endforeach()

file(INSTALL "${CURRENT_PORT_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
vcpkg_install_copyright(FILE_LIST
    "${SOURCE_PATH}/COPYING"
    "${SOURCE_PATH}/libarchive/archive_blake2.h"
)
