
vcpkg_from_github(
	OUT_SOURCE_PATH SOURCE_PATH
	REPO oneapi-src/oneTBB
	REF v2021.12.0
	SHA512 64022bcb61cf7b2030a1bcc11168445ef9f0d69b70290233a7febb71cc7a12cc2282dddc045f84e30893efe276342f02fd78d176706268eeaefe9aac7446d4e9
	HEAD_REF master
	PATCHES
		overflow-warning.patch
)
vcpkg_cmake_configure(
	SOURCE_PATH "${SOURCE_PATH}"
	OPTIONS
		-DTBB_TEST=OFF
		-DCMAKE_CXX_VISIBILITY_PRESET=hidden
		-DCMAKE_C_VISIBILITY_PRESET=hidden
		-DCMAKE_CXX_STANDARD=17
		-DCMAKE_C_STANDARD=11
)

vcpkg_cmake_install()

