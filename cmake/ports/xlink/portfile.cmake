vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/XLink
    REF 585a38fe4707e5f023de277135d8cb6ff9c4e0c4
    SHA512 e27834680374f348ffb590e181b3592df77c33d2de2c3b78061ff20d3315be5d4688da4ecfca8b3f914955367713ed53d50e61049b2f4eb95d6606b2426de7be
    HEAD_REF master
    PATCHES
        werror-switch.patch
)
vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        libusb          DXLINK_ENABLE_LIBUSB
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

