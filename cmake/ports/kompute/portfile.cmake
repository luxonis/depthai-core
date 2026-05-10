vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO KomputeProject/kompute
    # Pinned 48c127d fails against current vcpkg Vulkan-Hpp (DispatchLoaderDynamic / debug types).
    # 6160e788 is current master; includes VK_VERSION_1_4 dispatch-loader fixes.
    REF 6160e788daec0c8dc74b1f184d94809bf89872a9
    SHA512 565da26aca4d228be3dd1d56acba5bbf5bf72c531f0ca37b62056fb472175cba7b60bcc08accf41fcb336be6d9484af58469244964a0f924a7bea80ec960f3c2
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
