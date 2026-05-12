
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/libusb
    REF 745c272a2f0305c4ab162b7fb1b2edc7578c6b77
    SHA512 1ddf278e9f82061f77043400164d98e8e24a6627d1bcf9a1a8e57e9b032b3a183730b2651de1b66ab953cac8b49d1a399e1d3476ee9c07922be9e5a17ee396ff
    HEAD_REF bugfix/win_claimed_if_corruption
)

if(VCPKG_TARGET_IS_WINDOWS)
  set(CMAKE_CONFIGURE_OPTIONS_DEBUG "-DCMAKE_DEBUG_POSTFIX=d")
endif()

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DWITH_UDEV=OFF
        # Build shared libs by default to not cause licensing issues
        -DBUILD_SHARED_LIBS=ON
        ${CMAKE_CONFIGURE_OPTIONS_DEBUG}
)

vcpkg_cmake_install()

vcpkg_fixup_pkgconfig()

vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/usb-1.0 PACKAGE_NAME usb-1.0)

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")
