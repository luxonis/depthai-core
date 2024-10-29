
# Specific Catch2 version
hunter_config(
    Catch2
    VERSION "3.4.0"
    URL "https://github.com/catchorg/Catch2/archive/refs/tags/v3.4.0.tar.gz"
    SHA1 "4c308576c856a43dc88949a8f64ef90ebf94ae1b"
)



# TMP, could be read from XLink
# libusb without udev
hunter_config(
    libusb-luxonis
    VERSION "1.0.24-cmake"
    URL "https://github.com/luxonis/libusb/archive/b7e4548958325b18feb73977163ad44398099534.tar.gz"
    SHA1 "2d79573d57628fe56d2868d2f6ce756d40906cf4"
    CMAKE_ARGS
        WITH_UDEV=OFF
        # Build shared libs by default to not cause licensing issues
        BUILD_SHARED_LIBS=ON
)


# Pybind11 2.9.2-smart_holder
hunter_config(
    pybind11
    VERSION "luxonis_smart_holder"
    URL "https://github.com/luxonis/pybind11/archive/f760e2b984b66be2cfa202c93da9d341a557fb5d.tar.gz"
    SHA1 "d53247e4d1af52be040b647de0c25eb336bc85c7"
)


hunter_config(
    semver
    VERSION "v0.3.1"
    URL "https://github.com/Neargye/semver/archive/v0.3.1.tar.gz"
    SHA1 "c9ac79025cc259d8cca454be0865e88f154402be"
)

