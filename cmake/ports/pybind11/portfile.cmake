vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/pybind11
    REF f760e2b984b66be2cfa202c93da9d341a557fb5d
    SHA512 8baad5c326553075926901b9e2d5fcb574d298b382f17ba8ae0abf022fe3c92f4704b4a3fdce9b63d12ef31a436afc04d11a2050639f254be886a88d201a8b85
    HEAD_REF luxonis-smart-holder
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

