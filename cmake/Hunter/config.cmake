hunter_config(
    nlohmann_json
    VERSION "3.9.1"
    URL "https://github.com/nlohmann/json/archive/v3.9.1.tar.gz"
    SHA1 "f8a20a7e19227906d77de0ede97468fbcfea03e7"
)

hunter_config(
    XLink
    VERSION "luxonis-2021.4-master"
    URL "https://github.com/luxonis/XLink/archive/8a2ea2741b70dc5179952ad52aa80e126240e852.tar.gz"
    SHA1 "86c683f72092e4830b960cf5d7d3de079b1549e4"
)

hunter_config(
    BZip2
    VERSION "1.0.8-p0"
)

hunter_config(
    spdlog
    VERSION "1.8.2"
    URL "https://github.com/gabime/spdlog/archive/v1.8.2.tar.gz"
    SHA1 "4437f350ca7fa89a0cd8faca1198afb36823f775"
    CMAKE_ARGS
        SPDLOG_BUILD_EXAMPLE=OFF
        SPDLOG_FMT_EXTERNAL=OFF
        CMAKE_CXX_VISIBILITY_PRESET=hidden
        CMAKE_C_VISIBILITY_PRESET=hidden
)

# libarchive, luxonis fork
hunter_config(
    libarchive-luxonis
    VERSION "3.4.2-p2"
    URL "https://github.com/luxonis/libarchive/archive/cf2caf0588fc5e2af22cae37027d3ff6902e096f.tar.gz"
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
    URL "https://github.com/luxonis/FP16/archive/c911175d2717e562976e606c6e5f799bf40cf94e.tar.gz"
    SHA1 "40e9723c87c2fe21781132c0f2f8b90338500e32"
    CMAKE_ARGS
        FP16_BUILD_BENCHMARKS=OFF
        FP16_BUILD_TESTS=OFF
)

# Backward - Stacktrace printer
hunter_config(
    Backward
    VERSION "1.6"
    URL "https://github.com/bombela/backward-cpp/archive/refs/tags/v1.6.tar.gz"
    SHA1 "4ecb711eabfd15bc88ff9dd9342907fc5da46b62"
    CMAKE_ARGS
        BACKWARD_TESTS=OFF
)

# libnop - Serialization
hunter_config(
    libnop
    VERSION "1.0-72fcaaf"
    URL "https://github.com/luxonis/libnop/archive/72fcaaf119708cbaad2e04c087eadc88bbdefdef.tar.gz"
    SHA1 "fe13e463ed1f8ad045e0faa33f9bc5d581892fd5"
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
    URL "https://github.com/luxonis/zlib/archive/refs/tags/v1.2.11-p2.tar.gz"
    SHA1 "fb8b6486183b13a86040f793a939b128f6d27095"
)