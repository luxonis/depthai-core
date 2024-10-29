

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/lz4
    REF ba358ed311d125333d245e4c284464a72a168983
    SHA512 33955bf214c59ed094ea6961e28b6f59284e8e409ff57d9ba333b1ff12911b392cbb9594a0e88d5d151356471e7297597b445950b977bdca79c21fb69d0291c9
    HEAD_REF dev
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

