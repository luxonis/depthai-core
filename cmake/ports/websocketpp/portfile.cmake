#header-only library

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/websocketpp
    REF 6f878782807398aa9fbf5784eacd0753aafaf54f
    SHA512 cb4d8c7cb1886c94e510cd64df72e051d94eb504a86961394dccc93c676d6d034a4677b0369ec692fad23b1cfcbbc186a3f61f7abc64146cc0dd62c6d9b12242
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
