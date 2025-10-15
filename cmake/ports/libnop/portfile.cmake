vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/libnop
    REF 00938ba06ad513e4e0a12f9f2ec41cecad7e40d1
    SHA512 1377cc955148b9106e173d5877623df4f18400234ad2a3c0e6ab12970c53886478520f7bc591f4e8d89ae1a325c4cd030063043795e34f99ec6efe77457c6664
    HEAD_REF develop
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

