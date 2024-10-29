vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/basalt-headers
    REF 9172c4e6d8c8533d5ebae8289bba2a299f30eb50
    SHA512 39b4b88b147f4d8fb9b2423559c8f07b6e7e43a5136ff461058ab2fe33edfbd1790e6e8684abc2cbe647457752f4ff835c4ed0ed4a370141e345d1e529af2369
    HEAD_REF vcpkg_deps
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/basalt-headers")
