
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/libusb
    REF abc5b67c289df64aa56f7af736ba9164b96bc215
    SHA512 52ef84780305d274660e7906dfacb783de7f06444189085495d44fb84b43003f58804ae30ab8a955ddb0ce0b84ffb7c12bae4c6e3946e54e6ef0571fb06e27ec
    HEAD_REF luxonis-v1.0.30
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
