
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/cpp-httplib
    REF 3ba99c06f655a52e701c9a7ae5dc48850582d95b
    SHA512 282b4fd205d2de8ffc9ee901c2f2628ce10d992e5c9979de8e7509ac5fca94e1dae4cd6205b91885b9113465c9e5b64a78ae87dcc951eedcebaf27d0ee80050d
    HEAD_REF master
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        HTTPLIB_USE_OPENSSL_IF_AVAILABLE=OFF
        HTTPLIB_USE_BROTLI_IF_AVAILABLE=OFF
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/httplib")
