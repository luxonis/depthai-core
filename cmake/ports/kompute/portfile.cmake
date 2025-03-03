vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO KomputeProject/kompute
    REF 48c127d37750ed7033bf0f74972adad0ce099716
    SHA512 d46984e2f49b6b0c3eb313c03a935e2a1e3d480e40295bb2016f82ea1a23b484323f5bb534bffc309e1b09d219a7da8cf6dcea6e0b5ab8ee9f7aa65b6440b87d
    HEAD_REF master
)

vcpkg_cmake_configure(
    SOURCE_PATH ${SOURCE_PATH}
    OPTIONS
        -DKOMPUTE_OPT_USE_BUILT_IN_VULKAN_HEADER=OFF
        -DKOMPUTE_OPT_USE_BUILT_IN_FMT=OFF
        -DKOMPUTE_OPT_DISABLE_VULKAN_VERSION_CHECK=ON # Tmp fix for mac
        -DKOMPUTE_OPT_INSTALL=ON
        -DKOMPUTE_OPT_LOG_LEVEL=Warn
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(PACKAGE_NAME "kompute" CONFIG_PATH "lib/cmake/kompute")

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

file(INSTALL "${SOURCE_PATH}/LICENSE" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}" RENAME copyright)

configure_file("${CMAKE_CURRENT_LIST_DIR}/usage" "${CURRENT_PACKAGES_DIR}/share/${PORT}/usage" COPYONLY)
