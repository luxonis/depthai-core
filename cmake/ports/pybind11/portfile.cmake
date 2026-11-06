vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO pybind/pybind11
    REF fdec8c350aa0c7fbb552ec70d8176f8a13eb0561
    SHA512 6b98b6d7d2aaafc070ea0d3247d145c581eb4fa4d3bc7390c1127d093df550ce78ad0c8cd59bba1aa99fd5b4fca0c6929ce5efd6f56e8abe29a1c6e184d25aaa
    HEAD_REF master
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DPYBIND11_TEST=OFF
        # Disable all Python searching, Python required only for tests
        -DPYBIND11_NOPYTHON=ON
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH "share/cmake/pybind11")
vcpkg_fixup_pkgconfig()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/")

file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
