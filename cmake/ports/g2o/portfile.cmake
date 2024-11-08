vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO RainerKuemmerle/g2o
    REF 11440dc477508329775dc79e4cdb572f51336d82
    SHA512 da88b0900e11e60df33dceb00737d2425579601c025e7deec93f5cf3616f36f043799226356d786731d75106da0026786f0cf91feca21b36e31c20107bfb126b
    HEAD_REF master
)

string(COMPARE EQUAL "${VCPKG_LIBRARY_LINKAGE}" "dynamic" BUILD_LGPL_SHARED_LIBS)
file(REMOVE "${SOURCE_PATH}/cmake_modules/FindBLAS.cmake")

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_LGPL_SHARED_LIBS=${BUILD_LGPL_SHARED_LIBS}
        -DG2O_BUILD_EXAMPLES=OFF
        -DG2O_BUILD_APPS=OFF
		-DG2O_USE_OPENGL=OFF
)

vcpkg_cmake_install()

vcpkg_copy_pdbs()

vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/g2o)

if(VCPKG_LIBRARY_LINKAGE STREQUAL dynamic)
    file(GLOB_RECURSE HEADERS "${CURRENT_PACKAGES_DIR}/include/*")
    foreach(HEADER ${HEADERS})
        file(READ ${HEADER} HEADER_CONTENTS)
        string(REPLACE "#ifdef G2O_SHARED_LIBS" "#if 1" HEADER_CONTENTS "${HEADER_CONTENTS}")
        file(WRITE ${HEADER} "${HEADER_CONTENTS}")
    endforeach()
endif()

file(GLOB EXE "${CURRENT_PACKAGES_DIR}/bin/*.exe")
file(GLOB DEBUG_EXE "${CURRENT_PACKAGES_DIR}/debug/bin/*.exe")
if(EXE OR DEBUG_EXE)
    file(REMOVE ${EXE} ${DEBUG_EXE})
endif()
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")

file(INSTALL "${SOURCE_PATH}/doc/license-bsd.txt" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}" RENAME copyright)
