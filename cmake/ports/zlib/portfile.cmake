
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/zlib
    REF 0e5eb7e3ca8e2cd929148968dc0c8f54ed4c4ede
    SHA512 096f008157da402c72f74aedf6480aca755c9c4eb8e594437427b46d2b29f721b87443ac6de11466d3787e93ff57c767e8a8ecd030cc93ddc1885c1e6ab41a97
    HEAD_REF hunter-1.2.11
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
)

vcpkg_cmake_install()

