vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/XLink
    REF 2b517e1cb1ca77bea17679f9fdeb739812431174
    SHA512 f4c48411115a7708e60e73bda35350eaf2dfc23d84c7d5516dde8253ab06f2f226ff8529e567f26a135f9d909861f660336e4d6676326224c63d3be8828940cc
    HEAD_REF master
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

