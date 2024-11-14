vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/XLink
    REF 6615e0721021c136dc50a3af5a5bf16919ffef87
    SHA512 35fb1c1ee8dea69a6bf577fb374fe6dd3cf815e7baef70671732fa8062d81fc34c17439fbe95f3bfe1a4266e0bc4e47b0d033c8abf586bbd7e0713891ffe6c28
    HEAD_REF develop_server
    PATCHES
        hunter.patch
        windows.patch
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
