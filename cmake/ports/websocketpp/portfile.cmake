#header-only library

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO luxonis/websocketpp
    REF cebc62c393f07e664121f9b801ee7e8a1b273cee
    SHA512 ca48ebf3c7811e6194eeb97ba86b72c49d442f831549995c67edb4827b1b859c30b636ddd6478a5c9814a5438a40a59e59d3c4a76bdc9330d7cf28e3d1a5070b
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
