#include <catch2/catch_test_macros.hpp>

// This executable must contain only skipped cases: Catch2 then returns 4,
// which the shared dai_add_test helper must report as Skipped in CTest.
TEST_CASE("CTest recognizes an entirely skipped Catch2 executable", "[ctest_skip]") {
    SKIP("Intentional skip to verify Catch2/CTest integration without hardware");
}
