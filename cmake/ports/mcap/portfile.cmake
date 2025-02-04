vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/mcap_builder
    REF 32c455dbe67df11f9e7f731e57aa9caa065af834
    SHA512 d0e6681238b76449c272543bd2cc8bb62bb40ffd0cfd89fc279e0619087dc617f67ea6a6d4e66fde4a1e1be594b5a92fc99eb4663ce31b3832b9592d7bc51004
    HEAD_REF main
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
vcpkg_cmake_config_fixup(PACKAGE_NAME mcap CONFIG_PATH "lib/cmake/mcap")
