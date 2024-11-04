
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/basalt
    REF b4ec0d98a9b09781958573cc9c3c04f89fb773ca
    SHA512 1291995a57b9a509aa0e70afe840f9231240b3bc0ba025f92461be06227cd10fc76e09aed63dbee8eaa28e7787220e613016c23d4c86ccd79dfabd3cc9401b2b
    HEAD_REF deps_test
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
		-DBUILD_SHARED_LIBS=OFF
		-DBASALT_SDK_ONLY=ON
)

vcpkg_cmake_install()

