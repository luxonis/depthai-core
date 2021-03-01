if(CONFIG_MODE)
    set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/${_IMPORT_PREFIX}" ${CMAKE_PREFIX_PATH})
    set(QUIET "QUIET")
else()
    hunter_add_package(nlohmann_json)
    hunter_add_package(XLink)
    hunter_add_package(BZip2)
    hunter_add_package(FP16)
    hunter_add_package(libarchive)
    hunter_add_package(spdlog)
endif()

# If library was build as static, find all dependencies
if(NOT CONFIG_MODE OR (CONFIG_MODE AND NOT BUILD_SHARED_LIBS))
        
    # BZip2 (for bspatch)
    find_package(BZip2 ${QUIET} CONFIG REQUIRED)

    # FP16 for conversions
    find_package(FP16 ${QUIET} REQUIRED)

    # libarchive for firmware packages
    find_package(archive_static ${QUIET} REQUIRED)
    find_package(lzma ${QUIET} REQUIRED)

    # spdlog for library and device logging
    find_package(spdlog ${QUIET} CONFIG REQUIRED)

    # Add threads (c++)
    find_package(Threads ${QUIET} REQUIRED)
        
endif()

# Nlohmann JSON
find_package(nlohmann_json ${QUIET} CONFIG REQUIRED)

# XLink
if(DEPTHAI_XLINK_LOCAL)
    add_subdirectory("${DEPTHAI_XLINK_LOCAL}" ${CMAKE_CURRENT_BINARY_DIR}/XLink EXCLUDE_FROM_ALL)
else()
    find_package(XLink ${QUIET} CONFIG REQUIRED)
endif()

# OpenCV - (optional, quiet always)
find_package(OpenCV QUIET)
