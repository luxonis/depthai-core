if(DEPTHAI_SHARED_LOCAL)
    set(DEPTHAI_SHARED_FOLDER ${DEPTHAI_SHARED_LOCAL})
else()
    set(DEPTHAI_SHARED_FOLDER ${CMAKE_CURRENT_LIST_DIR}/depthai-shared)
endif()

set(DEPTHAI_SHARED_3RDPARTY_HEADERS_PATH "depthai-shared/3rdparty")

set(DEPTHAI_SHARED_SOURCES
    ${DEPTHAI_SHARED_FOLDER}/src/datatype/DatatypeEnum.cpp
    ${DEPTHAI_SHARED_FOLDER}/src/utility/Checksum.cpp
    ${DEPTHAI_SHARED_FOLDER}/src/utility/matrixOps.cpp
    ${DEPTHAI_SHARED_FOLDER}/src/common/ImgTransformations.cpp
)

set(DEPTHAI_SHARED_PUBLIC_INCLUDE
    ${DEPTHAI_SHARED_FOLDER}/include
)

set(DEPTHAI_SHARED_3RDPARTY_INCLUDE
    ${DEPTHAI_SHARED_FOLDER}/3rdparty
)

set(DEPTHAI_SHARED_INCLUDE
    ${DEPTHAI_SHARED_FOLDER}/src
)

# Make sure files exist
foreach(source_file ${DEPTHAI_SHARED_SOURCES})
    message(STATUS "Checking file: ${source_file}")
    if(NOT EXISTS ${source_file})
        message(FATAL_ERROR "depthai-shared submodule files missing. Make sure to download prepackaged release instead of \"Source code\" on GitHub. Example: depthai-core-vX.Y.Z.tar.gz")
    endif()
endforeach()