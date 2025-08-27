# tests/unit/CMakeLists.txt

# Must come first
find_package(Catch2 3 REQUIRED)

# Add a test executable
add_executable(unit_tests
    DynamicCalibration/UtilsTest.cpp
    DynamicCalibration/DynamicCalibrationTest.cpp
)

# Link test dependencies
target_link_libraries(unit_tests
    PRIVATE
    Catch2::Catch2WithMain
    dynamic_calibration_imported
    ${TARGET_CORE_NAME}
)

# Enable C++17
target_compile_features(unit_tests PRIVATE cxx_std_17)

# Add include directories
target_include_directories(unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

include(CTest)
include(Catch)
catch_discover_tests(unit_tests)
