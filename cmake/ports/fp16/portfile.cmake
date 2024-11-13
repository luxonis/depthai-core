

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
	REPO luxonis/FP16
    REF c911175d2717e562976e606c6e5f799bf40cf94e
    SHA512 4b892070e43dbaac3a457996981e4a9b6a0faaf7703d99f5f515604d8dcb2ff90c037c2660e36f68bdbb3481cb0fad5d2d7adf564ab19cefa420ee3f84bda4b6
    HEAD_REF master
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
		-DFP16_BUILD_BENCHMARKS=OFF
		-DFP16_BUILD_TESTS=OFF
)

vcpkg_cmake_install()

