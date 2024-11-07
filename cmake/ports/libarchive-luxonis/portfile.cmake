vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/libarchive
    REF 45baa3a3e57104519e1165bcd5ac29c3bd8c9f3a
    SHA512 8963cd567f471907cce941d5e6af7808b71cd289799b690fca84e7d7bb37d63816df387f5f4086fe47c33c80379b23cf3efcc455181214383ba7633202c5e783
    HEAD_REF hunter-3.5.2
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DENABLE_ACL=OFF
        -DENABLE_BZip2=OFF
        -DENABLE_CAT=OFF
        -DENABLE_CAT_SHARED=OFF
        -DENABLE_CNG=OFF
        -DENABLE_COVERAGE=OFF
        -DENABLE_CPIO=OFF
        -DENABLE_CPIO_SHARED=OFF
        -DENABLE_EXPAT=OFF
        -DENABLE_ICONV=OFF
        -DENABLE_INSTALL=ON
        -DENABLE_LIBB2=OFF
        -DENABLE_LIBXML2=OFF
        -DENABLE_LZ4=OFF
		-DENABLE_LZMA=OFF
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
        -DENABLE_WERROR=OFF
        -DENABLE_XATTR=OFF
        -DENABLE_ZLIB=OFF
        -DENABLE_ZSTD=OFF
)

vcpkg_cmake_install()

