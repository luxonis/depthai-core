# tests/unit/CMakeLists.txt

# Must come first
find_package(GTest CONFIG REQUIRED)

# Add a test executable
add_executable(unit_tests
    DynamicCalibration/DynamicCalibrationTest.cpp
)

# Link test dependencies
target_link_libraries(unit_tests
    PRIVATE
    GTest::gtest_main
    ${TARGET_CORE_NAME}  # Or replace with ${} if needed
)

# Enable C++17
target_compile_features(unit_tests PRIVATE cxx_std_17)

# Add include directories
target_include_directories(unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

# Register with CTest
include(GoogleTest)
gtest_discover_tests(unit_tests)
