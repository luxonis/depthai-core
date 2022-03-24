message(STATUS "---------------------iNITIIIIIIII inititing Hunter ${HUNTER_ROOT}")

hunter_config(
    nlohmann_json
    VERSION "3.9.1"
    URL "${HUNTER_ROOT}/_Base/Download/nlohmann_json/3.9.1/f8a20a7/v3.9.1.tar.gz"
    SHA1 "f8a20a7e19227906d77de0ede97468fbcfea03e7"
)

hunter_config(
    XLink
    VERSION "luxonis-2021.4.2-develop"
    URL "${HUNTER_ROOT}/_Base/Download/XLink/luxonis-2021.4.2-develop/58885ff/6210f8da430b8dec0257cda93110c57657ce2d15.tar.gz"
    SHA1 "58885ff0609ed5291039e512361bf7161b69910e"
)

hunter_config(
    BZip2
    VERSION "1.0.8-p0"
    URL "${HUNTER_ROOT}/_Base/Download/BZip2/1.0.8-p0/9125bd6/v1.0.8-p0.tar.gz"
    SHA1 "9125bd674fbe7c8169c8ea6a2a15a414a7dc2f86"
)

hunter_config(
    spdlog
    VERSION "1.8.2"
    URL "${HUNTER_ROOT}/_Base/Download/spdlog/1.8.2/4437f35/v1.8.2.tar.gz"
    SHA1 "4437f350ca7fa89a0cd8faca1198afb36823f775"
    CMAKE_ARGS
        SPDLOG_BUILD_EXAMPLE=OFF
        SPDLOG_FMT_EXTERNAL=OFF
        CMAKE_CXX_VISIBILITY_PRESET=hidden
        CMAKE_C_VISIBILITY_PRESET=hidden
)

hunter_config(
    lzma
    VERSION "5.2.3-p4"
    URL "${HUNTER_ROOT}/_Base/Download/lzma/5.2.3-p4/09d7d8c/v5.2.3-p4.tar.gz"
    SHA1 "09d7d8c8c8f1f488b3ccb739760c2092aae62441"
)

# libarchive, luxonis fork
hunter_config(
    libarchive-luxonis
    VERSION "3.4.2-p2"
    URL "${HUNTER_ROOT}/_Base/Download/libarchive-luxonis/3.4.2-p2/e99477d/cf2caf0588fc5e2af22cae37027d3ff6902e096f.tar.gz"
    SHA1 "e99477d32ce14292fe652dc5f4f460d3af8fbc93"
    CMAKE_ARGS
        ENABLE_ACL=OFF
        ENABLE_BZip2=OFF
        ENABLE_CAT=OFF
        ENABLE_CAT_SHARED=OFF
        ENABLE_CNG=OFF
        ENABLE_COVERAGE=OFF
        ENABLE_CPIO=OFF
        ENABLE_CPIO_SHARED=OFF
        ENABLE_EXPAT=OFF
        ENABLE_ICONV=OFF
        ENABLE_INSTALL=ON
        ENABLE_LIBB2=OFF
        ENABLE_LIBXML2=OFF
        ENABLE_LZ4=OFF
        ENABLE_LZMA=ON
        ENABLE_LZO=OFF
        ENABLE_LibGCC=OFF
        ENABLE_MBEDTLS=OFF
        ENABLE_NETTLE=OFF
        ENABLE_OPENSSL=OFF
        ENABLE_PCREPOSIX=OFF
        ENABLE_SAFESEH=AUTO
        ENABLE_TAR=OFF
        ENABLE_TAR_SHARED=OFF
        ENABLE_TEST=OFF
        ENABLE_WERROR=OFF
        ENABLE_XATTR=OFF
        ENABLE_ZLIB=OFF
        ENABLE_ZSTD=OFF
)

# Luxonis FP16 fork which doesn't use git cloning for its dependencies
hunter_config(
    FP16
    VERSION "luxonis-0.0.0"
    URL "${HUNTER_ROOT}/_Base/Download/FP16/luxonis-0.0.0/40e9723/c911175d2717e562976e606c6e5f799bf40cf94e.tar.gz"
    SHA1 "40e9723c87c2fe21781132c0f2f8b90338500e32"
    CMAKE_ARGS
        FP16_BUILD_BENCHMARKS=OFF
        FP16_BUILD_TESTS=OFF
)

# Backward - Stacktrace printer
hunter_config(
    Backward
    VERSION "1.6"
    URL "${HUNTER_ROOT}/_Base/Download/Backward/1.6/4ecb711/v1.6.tar.gz"
    SHA1 "4ecb711eabfd15bc88ff9dd9342907fc5da46b62"
    CMAKE_ARGS
        BACKWARD_TESTS=OFF
)

# libnop - Serialization
hunter_config(
    libnop
    VERSION "1.0-ec8f75a"
    URL "${HUNTER_ROOT}/_Base/Download/libnop/1.0-ec8f75a/261b8bf/ec8f75aa4df3454f73b5d7a7fe0680f3701b1744.tar.gz"
    SHA1 "261b8bfe734aff0456793416bbe4c4503dafbfdb"
)

# Specific Catch2 version
hunter_config(
    Catch2
    VERSION "2.13.7"
    URL "https://github.com/catchorg/Catch2/archive/refs/tags/v2.13.7.tar.gz"
    SHA1 "fa8f14ccf852413d3c6d3999145ada934d37d773"
)

# ZLib - Luxonis fix for alias on imported target for old CMake versions
hunter_config(
    ZLIB
    VERSION "1.2.11-p2"
    URL "${HUNTER_ROOT}/_Base/Download/ZLIB/1.2.11-p2/fb8b648/v1.2.11-p2.tar.gz"
    SHA1 "fb8b6486183b13a86040f793a939b128f6d27095"
)



# hunter_final_config(
#     PACKAGE "lzma"
#     VERSION "5.2.3-p4"
# )