if(CONFIG_MODE)
    set(_CMAKE_PREFIX_PATH_ORIGINAL ${CMAKE_PREFIX_PATH})
    set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/${_IMPORT_PREFIX}" ${CMAKE_PREFIX_PATH})
    set(_QUIET "QUIET")
else()
    set(depthai_SHARED_LIBS ${BUILD_SHARED_LIBS})
    hunter_add_package(nlohmann_json)
    if(NOT DEPTHAI_XLINK_LOCAL)
        hunter_add_package(XLink)
    endif()
    hunter_add_package(BZip2)
    hunter_add_package(FP16)
    hunter_add_package(libarchive-luxonis)
    hunter_add_package(spdlog)
endif()

# If library was build as static, find all dependencies
if(NOT CONFIG_MODE OR (CONFIG_MODE AND NOT depthai_SHARED_LIBS))
        
    # BZip2 (for bspatch)
    find_package(BZip2 ${_QUIET} CONFIG REQUIRED)

    # FP16 for conversions
    find_package(FP16 ${_QUIET} CONFIG REQUIRED)

    # libarchive for firmware packages
    find_package(archive_static ${_QUIET} CONFIG REQUIRED)
    find_package(lzma ${_QUIET} CONFIG REQUIRED)

    # spdlog for library and device logging
    find_package(spdlog ${_QUIET} CONFIG REQUIRED)
        
endif()

# Add threads (c++)
find_package(Threads ${_QUIET} REQUIRED)

# Nlohmann JSON
find_package(nlohmann_json ${_QUIET} CONFIG REQUIRED)

# XLink
if(DEPTHAI_XLINK_LOCAL)
    add_subdirectory("${DEPTHAI_XLINK_LOCAL}" ${CMAKE_CURRENT_BINARY_DIR}/XLink EXCLUDE_FROM_ALL)
else()
    find_package(XLink ${_QUIET} CONFIG REQUIRED)
endif()

# OpenCV 4 - (optional, quiet always)
find_package(OpenCV 4 QUIET CONFIG)

# Cleanup
if(CONFIG_MODE)
    set(CMAKE_PREFIX_PATH ${_CMAKE_PREFIX_PATH_ORIGINAL})
    set(_CMAKE_PREFIX_PATH_ORIGINAL)
    set(_QUIET)
else()
    set(depthai_SHARED_LIBS)
endif()
