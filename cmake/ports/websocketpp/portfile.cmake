#header-only library

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO amini-allight/websocketpp
    REF ee8cf4257e001d939839cff5b1766a835b749cd6 # 0.8.2
    SHA512 b8af1874655b29786e03ffd4bc91bcfdcad8a2a83cb26544269ec7f5a6464e83a36c2015a997d6128a088b92d22594ca47022cb250558f4bf97cb793e81f045d
    HEAD_REF develop
)

file(MAKE_DIRECTORY ${CURRENT_PACKAGES_DIR}/share/${PORT})

# Copy the header files
file(COPY "${SOURCE_PATH}/websocketpp" DESTINATION "${CURRENT_PACKAGES_DIR}/include" FILES_MATCHING PATTERN "*.hpp")

set(PACKAGE_INSTALL_INCLUDE_DIR "\${CMAKE_CURRENT_LIST_DIR}/../../include")
set(WEBSOCKETPP_VERSION 0.8.2)
set(PACKAGE_INIT "
macro(set_and_check)
  set(\${ARGV})
endmacro()
")
configure_file(${SOURCE_PATH}/websocketpp-config.cmake.in "${CURRENT_PACKAGES_DIR}/share/${PORT}/websocketpp-config.cmake" @ONLY)
configure_file(${SOURCE_PATH}/COPYING ${CURRENT_PACKAGES_DIR}/share/${PORT}/copyright COPYONLY)
