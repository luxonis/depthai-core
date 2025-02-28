vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/XLink
    REF 33f7a294d4b02f22257f9876f3b704fe3b0f3cf4
    SHA512 2ede385c77bd773c817e6b76554675c808a4c4fbbd8c8df4d8dc64bd57ced3775193e1c57a4badb988bd72651661960331b6652df077b4b768af58b3b992f797
    HEAD_REF develop_server
    PATCHES
        no-hunter.patch
)
vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        libusb          XLINK_ENABLE_LIBUSB
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
    # temporary
        ${FEATURE_OPTIONS}
		-DCMAKE_CXX_STANDARD=17
		-DCMAKE_C_STANDARD=11
)

vcpkg_cmake_install()

vcpkg_fixup_pkgconfig()
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
vcpkg_cmake_config_fixup(PACKAGE_NAME xlink CONFIG_PATH "lib/cmake/XLink")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")
