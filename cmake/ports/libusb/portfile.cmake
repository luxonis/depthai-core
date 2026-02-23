
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/libusb
    REF d631db2d91ce72f79ac296e3ff724eee98ad0c46
    SHA512 9525b73217335b37c4634a7f03af5478975c901e40bd661ccf2375d02a172702347dbc60035d5c46f17d75084371b239333a9b4adfe8e9b8d5257ac65193d48f
    HEAD_REF cmake-android-mainline
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
