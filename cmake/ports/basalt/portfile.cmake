
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/basalt
    REF bf3d2c33685b4315fa2f1407619c301c3135e891
    SHA512 346104eee47e08afd82d134b89c44163dee4528ed98875afd8bd6bd153d28f185145c5f938d0ff52423edca1434391ae6478ab3646f014c7598075babeadc054
    HEAD_REF depthai
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
