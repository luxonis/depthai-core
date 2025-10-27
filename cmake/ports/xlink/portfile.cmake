vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/XLink
    REF 7f5633ab542df632acaf4ccaf5e98d30f984e6e4
    SHA512 ab9a918caf8ab7c1c0328ca7eeef2fc61f81cd1d248bb23d90b7f0428c5b9a7c92e243851b8d99288eaecc8d665805527c28e309de20d79a4aba31d8fbde56b8
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
