cmake_minimum_required(VERSION 3.20)

# Set policies
# CMP0074 dictates that find_package searches environment variable "[packageName]_ROOT" along with regular variable [packageName]_ROOT
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW) # Only introduced in 3.12
endif()

if(POLICY CMP0028)
  cmake_policy(SET CMP0028 NEW)
endif()

# Specify path separator
set(SYS_PATH_SEPARATOR ";")
if(UNIX)
    set(SYS_PATH_SEPARATOR ":")
endif()

# Move binary dir if windows, to shorten the path
if(WIN32)
    set(HUNTER_BINARY_DIR "${HUNTER_GATE_ROOT}/_bin" CACHE STRING "Hunter binary directory")
endif()

# Pybindings project
set(TARGET_NAME depthai)
project(depthai VERSION "0") # revision of bindings [depthai-core].[rev]

# Add module paths
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

# Constants
set(DOCSTRINGS_INCLUDE_PLACEHOLDER_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/include)
set(DOCSTRINGS_INCLUDE_PLACEHOLDER_PATH ${DOCSTRINGS_INCLUDE_PLACEHOLDER_DIR}/docstring.hpp)
set(DEFAULT_DOCSTRINGS_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated/depthai_python_docstring.hpp)

# First specify options
# option(DEPTHAI_PYTHON_USE_FIND_PACKAGE "Use find_package for depthai-core" OFF)
option(DEPTHAI_PYTHON_ENABLE_TESTS "Enable tests" OFF)
option(DEPTHAI_PYTHON_ENABLE_EXAMPLES "Enable examples" OFF)
option(DEPTHAI_PYTHON_BUILD_DOCSTRINGS "Generate docstrings from header files if module 'pybind11_mkdoc' available" ON)
option(DEPTHAI_PYTHON_EMBEDDED_MODULE "Create an embeddable module" OFF)

# Add pybind11 dependency
#add_subdirectory(pybind11-2.5.0)

# Disable LTO if MINGW compiler
if(MINGW)
    set(PYBIND11_LTO_CXX_FLAGS "" CACHE STRING "" FORCE)
endif()

# Add external dependencies
add_subdirectory(external)

set(SOURCE_LIST
    src/py_bindings.cpp
    src/XLinkBindings.cpp
    src/DeviceBindings.cpp
    src/CalibrationHandlerBindings.cpp
    src/DeviceBootloaderBindings.cpp
    src/DatatypeBindings.cpp
    src/MessageQueueBindings.cpp
    src/pipeline/PipelineBindings.cpp
    src/pipeline/CommonBindings.cpp
    src/pipeline/AssetManagerBindings.cpp
    src/openvino/OpenVINOBindings.cpp
    src/nn_archive/NNArchiveBindings.cpp
    src/log/LogBindings.cpp
    src/VersionBindings.cpp
    src/common/ModelTypeBindings.cpp

    src/pipeline/node/NodeBindings.cpp

    src/pipeline/node/ColorCameraBindings.cpp
    src/pipeline/node/CameraBindings.cpp
    src/pipeline/node/MonoCameraBindings.cpp
    src/pipeline/node/StereoDepthBindings.cpp
    src/pipeline/node/NeuralNetworkBindings.cpp
    src/pipeline/node/VideoEncoderBindings.cpp
    src/pipeline/node/ImageManipBindings.cpp
    src/pipeline/node/SPIOutBindings.cpp
    src/pipeline/node/SPIInBindings.cpp
    src/pipeline/node/DetectionNetworkBindings.cpp
    src/pipeline/node/SystemLoggerBindings.cpp
    src/pipeline/node/ScriptBindings.cpp
    src/pipeline/node/SpatialLocationCalculatorBindings.cpp
    src/pipeline/node/SpatialDetectionNetworkBindings.cpp
    src/pipeline/node/ObjectTrackerBindings.cpp
    src/pipeline/node/IMUBindings.cpp
    src/pipeline/node/EdgeDetectorBindings.cpp
    src/pipeline/node/FeatureTrackerBindings.cpp
    src/pipeline/node/ThermalBindings.cpp
    src/pipeline/node/ToFBindings.cpp
    src/pipeline/node/AprilTagBindings.cpp
    src/pipeline/node/DetectionParserBindings.cpp
    src/pipeline/node/WarpBindings.cpp
    src/pipeline/node/SyncBindings.cpp
    src/pipeline/node/BenchmarkBindings.cpp
    src/pipeline/node/UVCBindings.cpp
    src/pipeline/node/ToFBindings.cpp
    src/pipeline/node/PointCloudBindings.cpp
    src/pipeline/node/SyncBindings.cpp
    src/pipeline/node/MessageDemuxBindings.cpp
    src/pipeline/node/HostNodeBindings.cpp
    src/pipeline/node/RecordBindings.cpp
    src/pipeline/node/ReplayBindings.cpp
    src/pipeline/node/ImageAlignBindings.cpp
    src/pipeline/node/RGBDBindings.cpp

    src/pipeline/datatype/ADatatypeBindings.cpp
    src/pipeline/datatype/AprilTagConfigBindings.cpp
    src/pipeline/datatype/AprilTagsBindings.cpp
    src/pipeline/datatype/BufferBindings.cpp
    src/pipeline/datatype/CameraControlBindings.cpp
    src/pipeline/datatype/EdgeDetectorConfigBindings.cpp
    src/pipeline/datatype/FeatureTrackerConfigBindings.cpp
    src/pipeline/datatype/ThermalConfigBindings.cpp
    src/pipeline/datatype/ToFConfigBindings.cpp
    src/pipeline/datatype/ImageManipConfigBindings.cpp
    src/pipeline/datatype/ImgDetectionsBindings.cpp
    src/pipeline/datatype/ImgFrameBindings.cpp
    src/pipeline/datatype/EncodedFrameBindings.cpp
    src/pipeline/datatype/IMUDataBindings.cpp
    src/pipeline/datatype/MessageGroupBindings.cpp
    src/pipeline/datatype/NNDataBindings.cpp
    src/pipeline/datatype/RGBDDataBindings.cpp
    src/pipeline/datatype/SpatialImgDetectionsBindings.cpp
    src/pipeline/datatype/SpatialLocationCalculatorConfigBindings.cpp
    src/pipeline/datatype/SpatialLocationCalculatorDataBindings.cpp
    src/pipeline/datatype/StereoDepthConfigBindings.cpp
    src/pipeline/datatype/SystemInformationBindings.cpp
    src/pipeline/datatype/TrackedFeaturesBindings.cpp
    src/pipeline/datatype/TrackletsBindings.cpp
    src/pipeline/datatype/BenchmarkReportBindings.cpp
    src/pipeline/datatype/PointCloudConfigBindings.cpp
    src/pipeline/datatype/ObjectTrackerConfigBindings.cpp
    src/pipeline/datatype/PointCloudDataBindings.cpp
    src/pipeline/datatype/TransformDataBindings.cpp
    src/pipeline/datatype/ImageAlignConfigBindings.cpp
    src/pipeline/datatype/ImgAnnotationsBindings.cpp
    src/remote_connection/RemoteConnectionBindings.cpp

    src/capabilities/CapabilityBindings.cpp
    src/capabilities/CapabilityRangeBindings.cpp
    src/capabilities/ImgFrameCapabilityBindings.cpp

    src/modelzoo/ZooBindings.cpp

    src/remote_connection/RemoteConnectionBindings.cpp
    src/utility/EventsManagerBindings.cpp
)
if(DEPTHAI_MERGED_TARGET)
    list(APPEND SOURCE_LIST
        external/pybind11_opencv_numpy/ndarray_converter.cpp
    )
endif()

if(DEPTHAI_BASALT_SUPPORT)
    list(APPEND SOURCE_LIST
        src/pipeline/node/BasaltVIOBindings.cpp
    )
endif()

if(DEPTHAI_RTABMAP_SUPPORT)
    list(APPEND SOURCE_LIST
        src/pipeline/node/RTABMapVIOBindings.cpp
        src/pipeline/node/RTABMapSLAMBindings.cpp
    )
endif()

# Add files for python module
if(DEPTHAI_PYTHON_EMBEDDED_MODULE)
    add_library(${TARGET_NAME} ${SOURCE_LIST})
else()
    pybind11_add_module(${TARGET_NAME} ${SOURCE_LIST})
endif()

if(WIN32)
    # Copy dlls to target directory - Windows only
    # TARGET_RUNTIME_DLLS generator expression available since CMake 3.21
    if(CMAKE_VERSION VERSION_LESS "3.21")
        file(GLOB depthai_dll_libraries "${HUNTER_INSTALL_PREFIX}/bin/*.dll")
    else()
        set(depthai_dll_libraries "$<TARGET_RUNTIME_DLLS:${TARGET_NAME}>")
    endif()
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD COMMAND
        "$<$<BOOL:${depthai_dll_libraries}>:${CMAKE_COMMAND};-E;copy_if_different;${depthai_dll_libraries};$<TARGET_FILE_DIR:${TARGET_NAME}>>"
        COMMAND_EXPAND_LISTS
        VERBATIM
    )

    # Disable "d" postfix, so python can import the library as is
    set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "")
endif()

# Add stubs (pyi) generation step after building bindings
execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c" "from mypy import api" RESULT_VARIABLE error OUTPUT_QUIET ERROR_QUIET)
if(error OR CMAKE_CROSSCOMPILING)
    message(WARNING "Mypy not available or cross compiling - stubs won't be generated or checked")
else()
    get_target_property(bindings_directory ${TARGET_NAME} LIBRARY_OUTPUT_DIRECTORY)
    if(NOT bindings_directory)
        set(bindings_directory ${CMAKE_CURRENT_BINARY_DIR})
    endif()
    message(STATUS "Mypy available, creating and checking stubs. Running with generate_stubs.py ${TARGET_NAME} ${bindings_directory}")
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD COMMAND
        ${CMAKE_COMMAND} -E env
        # Python path (to find compiled module)
        "PYTHONPATH=$<TARGET_FILE_DIR:${TARGET_NAME}>${SYS_PATH_SEPARATOR}$ENV{PYTHONPATH}"
        ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/generate_stubs.py" "${TARGET_NAME}" "$<TARGET_FILE_DIR:${TARGET_NAME}>"
        DEPENDS "${CMAKE_CURRENT_LIST_DIR}/generate_stubs.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
endif()

# Docstring options
if(DEPTHAI_PYTHON_DOCSTRINGS_INPUT AND DEPTHAI_PYTHON_DOCSTRINGS_OUTPUT)
    message(FATAL_ERROR "DEPTHAI_PYTHON_DOCSTRINGS_INPUT and DEPTHAI_PYTHON_DOCSTRINGS_OUTPUT are mutually exclusive")
endif()

if(DEPTHAI_PYTHON_DOCSTRINGS_OUTPUT)
    # If output is specified set both input and output to same the path
    set(docstring_input_path ${DEPTHAI_PYTHON_DOCSTRINGS_OUTPUT})
    set(docstring_output_path ${DEPTHAI_PYTHON_DOCSTRINGS_OUTPUT})
else()
    # If input docstrings explicitly specified, use those and disable building
    if(DEPTHAI_PYTHON_DOCSTRINGS_INPUT)
        set(docstring_input_path ${DEPTHAI_PYTHON_DOCSTRINGS_INPUT})
        message(STATUS "Disabled building of docstrings - using docstrings specified by DEPTHAI_PYTHON_DOCSTRINGS_INPUT (${DEPTHAI_PYTHON_DOCSTRINGS_INPUT})")
        set(DEPTHAI_PYTHON_BUILD_DOCSTRINGS OFF CACHE BOOL "Generate docstrings from header files if module 'pybind11_mkdoc' available" FORCE)
    else()
        # Otherwise set default location as input
        set(docstring_input_path ${DEFAULT_DOCSTRINGS_OUTPUT})
    endif()

    # Set default output location
    set(docstring_output_path ${DEFAULT_DOCSTRINGS_OUTPUT})
endif()

if(DEPTHAI_PYTHON_BUILD_DOCSTRINGS)
    option(DEPTHAI_PYTHON_FORCE_DOCSTRINGS "Force that docstrings are generated, module 'pybind11_mkdoc' required" OFF)
endif()

# Configure include placeholder with INPUT path
configure_file(cmake/docstring.hpp.in ${DOCSTRINGS_INCLUDE_PLACEHOLDER_PATH})
# Add target to generate docstrings
if (DEPTHAI_PYTHON_BUILD_DOCSTRINGS)
    include(pybind11-mkdoc)
    set(DEPTHAI_MKDOC_TARGETS depthai::core)
    if(DEPTHAI_BASALT_SUPPORT)
        list(APPEND DEPTHAI_MKDOC_TARGETS depthai::basalt)
    endif()
    if(DEPTHAI_RTABMAP_SUPPORT)
        list(APPEND DEPTHAI_MKDOC_TARGETS depthai::rtabmap)
    endif()
    # Set up docstrings for combined targets
    target_pybind11_mkdoc_setup_combined(${docstring_output_path} ${DEPTHAI_PYTHON_FORCE_DOCSTRINGS} ${DEPTHAI_MKDOC_TARGETS})

    if(TARGET pybind11_mkdoc)
        # Add dependency to mkdoc target (makes sure that mkdoc is executed, and docstrings available)
        add_dependencies(${TARGET_NAME} pybind11_mkdoc)
    else()
        # Generate default docstrings to OUTPUT path
        configure_file(cmake/default_docstring.hpp.in ${docstring_output_path} COPYONLY)
    endif()
else()
    # Generate default docstrings to OUTPUT path
    configure_file(cmake/default_docstring.hpp.in ${docstring_output_path} COPYONLY)
endif()

# Add include directory
target_include_directories(${TARGET_NAME} PRIVATE src ${DOCSTRINGS_INCLUDE_PLACEHOLDER_DIR} "${CMAKE_CURRENT_LIST_DIR}/../../src")
if(DEPTHAI_MERGED_TARGET)
    target_include_directories(${TARGET_NAME} PRIVATE external/pybind11_opencv_numpy)
endif()

set(DEPTHAI_LINK_TARGET depthai::core)
if(NOT DEPTHAI_MERGED_TARGET)
    list(APPEND DEPTHAI_LINK_TARGET depthai::opencv)
endif()

if(DEPTHAI_BASALT_SUPPORT)
    list(APPEND DEPTHAI_LINK_TARGET depthai::basalt)
endif()
if(DEPTHAI_RTABMAP_SUPPORT AND DEPTHAI_PCL_SUPPORT)
    list(APPEND DEPTHAI_LINK_TARGET depthai::rtabmap)
endif()

# Link with libraries
target_link_libraries(${TARGET_NAME}
    PUBLIC
        # pybind11
        pybind11::pybind11
        ${DEPTHAI_LINK_TARGET}
        hedley
        pybind11_json
        xtensor-python
        spdlog::spdlog
)

# Add embedded module option, otherwise link to pybind11 as usual
if(DEPTHAI_PYTHON_EMBEDDED_MODULE)
    target_compile_definitions(${TARGET_NAME} PRIVATE DEPTHAI_PYTHON_EMBEDDED_MODULE)
endif()

if(DEPTHAI_ENABLE_REMOTE_CONNECTION)
    target_compile_definitions(${TARGET_NAME} PRIVATE DEPTHAI_ENABLE_REMOTE_CONNECTION)
endif()
if(DEPTHAI_ENABLE_EVENTS_MANAGER)
    target_compile_definitions(${TARGET_NAME} PRIVATE DEPTHAI_ENABLE_EVENTS_MANAGER)
endif()

# Add the clang-format target
if(COMMAND target_clangformat_setup)
    set(header_dirs "${CMAKE_CURRENT_LIST_DIR}/src")
    target_clangformat_setup(${TARGET_NAME} "${header_dirs}")
endif()

# Find Git
find_package(Git)

# Add build information (commit, date)
set(BUILD_COMMIT "dev")
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE BUILD_COMMIT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} show -s --format=%ci ${BUILD_COMMIT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE BUILD_COMMIT_DATETIME
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()

# Add local commit hash (or 'dev' if unable to retrieve) if not build by CI
if(NOT DEFINED ENV{CI} AND NOT DEPTHAI_PYTHON_COMMIT_HASH)
    set(DEPTHAI_PYTHON_COMMIT_HASH ${BUILD_COMMIT})
endif()

# Get version to use
set(version_command "import find_version as v; print(v.get_package_version())")
if(DEPTHAI_PYTHON_COMMIT_HASH)
    set(version_command "import find_version as v; print(v.get_package_dev_version('${DEPTHAI_PYTHON_COMMIT_HASH}'))")
endif()
execute_process(COMMAND ${PYTHON_EXECUTABLE} "-c" "${version_command}"
    OUTPUT_VARIABLE DEPTHAI_PYTHON_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)


string(TIMESTAMP BUILD_DATETIME "%Y-%m-%d %H:%M:%S +0000" UTC)
target_compile_definitions(${TARGET_NAME}
    PRIVATE
        DEPTHAI_PYTHON_VERSION="${DEPTHAI_PYTHON_VERSION}"
        DEPTHAI_PYTHON_COMMIT_HASH="${BUILD_COMMIT}"
        DEPTHAI_PYTHON_COMMIT_DATETIME="${BUILD_COMMIT_DATETIME}"
        DEPTHAI_PYTHON_BUILD_DATETIME="${BUILD_DATETIME}"
)

# ASAN Settings as we are building and using shared library
if(SANITIZE_ADDRESS OR SANITIZE_MEMORY OR SANITIZE_THREAD OR SANITIZE_UNDEFINED)
    # Get asan library to preload
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libclang_rt.asan-${CMAKE_HOST_SYSTEM_PROCESSOR}.so OUTPUT_VARIABLE LIBASAN_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libasan.so OUTPUT_VARIABLE LIBASAN_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
    # Set preload env variable
    if(APPLE)
        set(ASAN_ENVIRONMENT_VARS "DYLD_INSERT_LIBRARIES=${LIBASAN_PATH}" "ASAN_OPTIONS=leak_check_at_exit=0")
    elseif(UNIX)
        set(ASAN_ENVIRONMENT_VARS "LD_PRELOAD=${LIBASAN_PATH}" "ASAN_OPTIONS=leak_check_at_exit=0")
    endif()
    message(STATUS "ASAN environment variables: ${ASAN_ENVIRONMENT_VARS}")
endif()

########################
# Testing
########################
if(DEPTHAI_PYTHON_ENABLE_TESTS OR DEPTHAI_PYTHON_ENABLE_EXAMPLES)
    include(CTest)
    enable_testing()
endif()

########################
# Tests
########################
if (DEPTHAI_PYTHON_ENABLE_TESTS)
    add_subdirectory(tests)
endif()

########################
# Examples (can also act as tests)
########################
if (DEPTHAI_PYTHON_ENABLE_EXAMPLES)
    add_subdirectory(../../examples/python ${CMAKE_BINARY_DIR}/examples/python)
endif()
