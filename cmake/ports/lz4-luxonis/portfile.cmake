

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
	REPO Serafadam/lz4
    REF 4067f26aa0f6054cb8a92642518e50ad94591b37
    SHA512 9dc1f48442cd41039dda8eeed1e8fccbc0531983473c7484c121823415ed432f1677182b19533a3d88891dab74ab00295e402333c1513226e974b13fe57dcb0d
    HEAD_REF flag_cleanup
)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

