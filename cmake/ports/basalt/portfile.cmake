
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/basalt
    REF ac3cd0fbef91e889db27a1a779a5105ba715fef7
    SHA512 1e7694168f92af5f48f462a793fff528c46a4bd01d3417daf2031b651acba70ee9d61d38a33a1be217cc6e97baa67425e1a8fcc4e57a46033954757112326206
    HEAD_REF deps_test
    PATCHES
        win.patch
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
		-DBASALT_SDK_ONLY=ON
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(PACKAGE_NAME basalt_sdk CONFIG_PATH "lib/cmake/basalt_sdk")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
