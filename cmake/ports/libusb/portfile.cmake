
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/libusb
    REF 6a2cebf82866b593ce97d79b1b70ad08ea19c02b
    SHA512 875bf257521a9a532dce4c58d0b29db5d4bbc4789eec0b3528ad828e9f6d30726c15253fa56bc5902b86a7dde3480c3cb698f16ec58c30a8b7f59f7c40bbdf5b
    HEAD_REF fix/macos26-usb-port-number
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
