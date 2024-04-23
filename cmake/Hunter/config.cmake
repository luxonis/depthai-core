# hunter_config(
#     nlohmann_json
#     VERSION "3.9.1"
#     URL "https://github.com/nlohmann/json/archive/v3.9.1.tar.gz"
#     SHA1 "f8a20a7e19227906d77de0ede97468fbcfea03e7"
# )
hunter_config(
    nlohmann_json
    VERSION "3.9.1"
)

hunter_config(
    XLink
    VERSION "luxonis-2021.4.3-develop"
    URL "https://github.com/luxonis/XLink/archive/061709357b06a74510b5b9e3e2832d9d80efd3ff.tar.gz"
    SHA1 "affa1bdd7487cf70c9caf40365844967ae761ea3"
    CMAKE_ARGS
        XLINK_ENABLE_LIBUSB=${DEPTHAI_ENABLE_LIBUSB}
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
    VERSION "hunter-3.5.2"
    URL "https://github.com/luxonis/libarchive/archive/45baa3a3e57104519e1165bcd5ac29c3bd8c9f3a.tar.gz"
    SHA1 "ca5cd0f1c31b9c187d7119cb1aa7467f8c231d29"
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

if(NOT EMSCRIPTEN)
    # Backward - Stacktrace printer
    hunter_config(
      Backward
      VERSION "1.6"
      URL "https://github.com/bombela/backward-cpp/archive/refs/tags/v1.6.tar.gz"
      SHA1 "4ecb711eabfd15bc88ff9dd9342907fc5da46b62"
      CMAKE_ARGS
          BACKWARD_TESTS=OFF
    )
endif()

# libnop - Serialization
hunter_config(
    libnop
    VERSION "1.0-ec8f75a"
    URL "https://github.com/luxonis/libnop/archive/ab842f51dc2eb13916dc98417c2186b78320ed10.tar.gz"
    SHA1 "32f40f084615ba7940ce9d29f05f3294371aabeb"
)

# Specific Catch2 version
hunter_config(
    Catch2
    VERSION "3.4.0"
    URL "https://github.com/catchorg/Catch2/archive/refs/tags/v3.4.0.tar.gz"
    SHA1 "4c308576c856a43dc88949a8f64ef90ebf94ae1b"
)

# ZLib - Luxonis fix for alias on imported target for old CMake versions
hunter_config(
    ZLIB
    VERSION "1.2.11-p2"
    URL "https://github.com/luxonis/zlib/archive/refs/tags/v1.2.11-p2.tar.gz"
    SHA1 "fb8b6486183b13a86040f793a939b128f6d27095"
)

# LZ4 for mcap
hunter_config(
    lz4-luxonis
    VERSION "1.9.4-p2"
    URL "https://github.com/luxonis/lz4/archive/ba358ed311d125333d245e4c284464a72a168983.tar.gz"
    SHA1 "43ae0d2343147e32cdd8b85cefb5a311c3ee5504"
    CMAKE_ARGS
        BUILD_SHARED_LIBS=ON
)

# TMP, could be read from XLink
# libusb without udev
hunter_config(
    libusb-luxonis
    VERSION "1.0.24-cmake"
    URL "https://github.com/luxonis/libusb/archive/b7e4548958325b18feb73977163ad44398099534.tar.gz"
    SHA1 "2d79573d57628fe56d2868d2f6ce756d40906cf4"
    CMAKE_ARGS
        WITH_UDEV=OFF
        # Build shared libs by default to not cause licensing issues
        BUILD_SHARED_LIBS=ON
)

# cpp-httplib
hunter_config(
    httplib
    VERSION "0.11.2"
    URL "https://github.com/luxonis/cpp-httplib/archive/3ba99c06f655a52e701c9a7ae5dc48850582d95b.tar.gz"
    SHA1 "84ddd5d58a210b6203c50760d2ebde75b0ff6376"
    CMAKE_ARGS
        HTTPLIB_USE_OPENSSL_IF_AVAILABLE=OFF
        HTTPLIB_USE_BROTLI_IF_AVAILABLE=OFF
)

# # g2o

# hunter_config(
#     g2o
#     VERSION "20230806_git"
#     URL "https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20230806_git.tar.gz"
#     SHA1 "d73df7d192a9f5d4f6c84f4b70d7cbd0564a0d53"
#     CMAKE_ARGS
#         BUILD_WITH_MARCH_NATIVE=OFF
#         G2O_BUILD_APPS=OFF
#         G2O_BUILD_EXAMPLES=OFF
#         G2O_USE_OPENGL=OFF
# )

# RTABMap
hunter_config(
    rtbmap
    # VERSION "0.21.4"
    URL "https://github.com/introlab/rtabmap/archive/refs/heads/master.tar.gz"
    SHA1 "b18fd4cf6811c4b5b4e925f9f8f0303ed11c55b1"
    CMAKE_ARGS
        CMAKE_BUILD_TYPE=RelWithDebInfo
        BUILD_APP=OFF
        WITH_UDEV=OFF
        BUILD_EXAMPLES=OFF
        BUILD_TOOLS=OFF
        # Build shared libs by default to not cause licensing issues
        BUILD_SHARED_LIBS=ON
        WITH_QT=ON
        WITH_ORB_OCTREE=OFF
        WITH_TORCH=OFF
        WITH_PYTHON=OFF
        WITH_PYTHON_THREADING=OFF
        WITH_PDAL=OFF
        WITH_FREENECT=OFF
        WITH_FREENECT2=OFF
        WITH_K4W2=OFF
        WITH_K4A=OFF
        WITH_OPENNI2=OFF
        WITH_DC1394=OFF
        WITH_G2O=ON
        WITH_GTSAM=ON
        WITH_TORO=ON
        WITH_CERES=OFF
        WITH_MRPT=OFF
        WITH_VERTIGO=ON
        WITH_CVSBA=OFF
        WITH_POINTMATCHER=ON
        WITH_CCCORELIB=OFF
        WITH_OPEN3D=OFF
        WITH_LOAM=OFF
        WITH_FLOAM=OFF
        WITH_FLYCAPTURE2=OFF
        WITH_ZED=OFF
        WITH_ZEDOC=OFF
        WITH_REALSENSE=OFF
        WITH_REALSENSE_SLAM=OFF
        WITH_REALSENSE2=OFF
        WITH_MYNTEYE=OFF
        WITH_DEPTHAI=OFF
        WITH_OCTOMAP=OFF
        WITH_GRIDMAP=OFF
        WITH_CPUTSDF=OFF
        WITH_OPENCHISEL=OFF
        WITH_ALICE_VISION=OFF
        WITH_FOVIS=OFF
        WITH_VISO2=OFF
        WITH_DVO=OFF
        WITH_ORB_SLAM=OFF
        WITH_OKVIS=OFF
        WITH_MSCKF_VIO=OFF
        WITH_VINS=OFF
        WITH_OPENVINS=OFF
        WITH_MADGWICK=OFF
        WITH_FASTCV=OFF
        WITH_OPENMP=OFF
        WITH_OPENGV=OFF
        PCL_OMP=OFF
)
# Pybind11 2.9.2
# # Pybind11 2.11.0-smart_holder
# hunter_config(
#     pybind11
#     VERSION "2.11.0-smart_holder"
#     URL "https://github.com/pybind/pybind11/archive/10283c2ef44a9100bc88d066a4972c4f51ded2b0.tar.gz"
#     SHA1 "0da09bdd6987a33feb800e4b7f129df5c9aa5aed"
# )

# Pybind11 2.9.2-smart_holder
hunter_config(
    pybind11
    VERSION "2.9.2-smart_holder"
    URL "https://github.com/luxonis/pybind11/archive/9fa714f524fdbaf8e81c1a1b1637c547fed4e18f.tar.gz"
    SHA1 "0cdb8fe12d33bac220b58aa85b219de02716f537"
)

hunter_config(
    mp4v2
    VERSION "2.1.3"
    URL "https://github.com/luxonis/mp4v2/archive/1dc9f4d24645ea43405582e5c813dec3eaa8fd3e.tar.gz"
    SHA1 "2ac9e4348c78d09a3b4fc6e147b9eeb356ba31f1"
    CMAKE_ARGS
        BUILD_SHARED=ON
        BUILD_UTILS=OFF
)


# hunter_config(
#     basalt
#     URL "https://gitlab.freedesktop.org/mateosss/basalt/-/archive/release-673cc5c6/basalt-release-673cc5c6.tar.gz"
#     SHA1 "60825d3e76f15896abc5011edaafffe4ff4b6c93"
#     CMAKE_ARGS
#         BASALT_INSTANTIATIONS_DOUBLE=off
#         BUILD_TESTS=off
# )

hunter_config(
    Sophus
    URL "https://github.com/strasdat/Sophus/archive/936265f5815dee546a3583ceba58c234c2ecb271.zip"
    SHA1 "7f54e06fb1680910c465e12ea85fc414ce7a9f9e" 
    CMAKE_ARGS
        CMAKE_BUILD_TYPE=RelWithDebInfo
)
hunter_config(
    magic_enum
    URL "https://github.com/Neargye/magic_enum/archive/3d1f6a5a2a3fbcba077e00ad0ccc2dd9fefc2ca7.zip"
    SHA1 "c9a27f6ff8311f0c6b2adb959d0598f079fcc9f3"
)