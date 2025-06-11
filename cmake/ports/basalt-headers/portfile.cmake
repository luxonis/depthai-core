vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/basalt-headers
    REF 5392967dc6825838a52fa6d6ed38188a55a6acf7
    SHA512 7b22090aacf609e9b8417dd852607d09f0d0db144980852ec1cd73ae2e5fa0f095ef450011a8d7bf6b630f9af8a8ca5870d590de25b8c0783d843344aa48866b
    HEAD_REF vcpkg_deps
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/basalt-headers")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/lib" "${CURRENT_PACKAGES_DIR}/lib")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
