vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/mcap_builder
    REF a0d93edb81e3cca041b8be45f65e58e65c8d4e0a
    SHA512 e56824a96388069ee7dbaeb092a47bf3c73061c6e074230cc889987f1b957409bdcfe40ce2f6e4371b5b99574ecac544b4ca8479397c5b8cc4f2f9b2b9c903a6
    HEAD_REF main
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
vcpkg_cmake_config_fixup(PACKAGE_NAME mcap CONFIG_PATH "lib/cmake/mcap")
