
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/basalt
    REF 454dccbe7c7e13b9d1e1124f19d0ecbe808994ad
    SHA512 e0335d28b6466c521a3562338d77552f27e129934b2d364d9ac26a3e8e3a087c200836ddfd5cbba438a2290c2c3f819d377a3505fb8210bcaf590c7738d93b05
    HEAD_REF ros_tests
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
