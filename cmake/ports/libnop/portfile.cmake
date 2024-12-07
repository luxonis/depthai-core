vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/libnop
    REF ab842f51dc2eb13916dc98417c2186b78320ed10
    SHA512 6353765ac8c489e66e0c91dc1da899e5a89200eb3834c95a1697b5e22e4d7186d920c24a67272ff92fdf22a0fac5b6b838345ad077e67693f35815c1cc9dbf17
    HEAD_REF develop
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

