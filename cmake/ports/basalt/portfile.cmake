
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/basalt
    REF ef61684ca32d0f827981270f780830c5f84c8e85
    SHA512 be82d5f8971265178b8ab1f784fecb498963d3d8b641a47c541416dcea34f49e4138b2b191c930a5e1d9ca5b2ebcd1c1872cc0a2bb6bced683e9d44240f6eed3
    HEAD_REF depthai)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
		-DBASALT_SDK_ONLY=ON
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(PACKAGE_NAME basalt_sdk CONFIG_PATH "lib/cmake/basalt_sdk")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
