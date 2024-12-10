

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/mp4v2
    REF 1dc9f4d24645ea43405582e5c813dec3eaa8fd3e
    SHA512 d1acef4d4fc03154f55dd04f0319c5ade8fa85ffc4d7f8240f68ff9f053079c337945e4c2a860d520b442b05cea8c873aede6ef104957bf64ee548020d7b641c
    HEAD_REF master
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
		-DBUILD_SHARED=OFF
		-DBUILD_UTILS=OFF
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/mp4v2")
vcpkg_fixup_pkgconfig()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/COPYING")
