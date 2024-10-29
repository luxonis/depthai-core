
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
	REPO Serafadam/cpr
    REF c898870a8992c1a679c04b8da3979b8cbcb91a6d
    SHA512 efa68d4e41668345988ce2403e4ef1f98d0e7f6bdca797174f8c6f5eb5dbdd818faa75aa583ce89391154e2cfe528e0128f1657421a2c8a0ddc71fe43b559a84
    HEAD_REF vcpkg
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
	OPTIONS
	  -DBUILD_CPR_TESTS=OFF
)

vcpkg_cmake_install()

