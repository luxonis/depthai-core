#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>

#include "depthai/utility/Path.hpp"
#include "utility/Platform.hpp"

TEST_CASE("Test existence of files with non-ascii characters") {
    dai::Path tempPath = dai::platform::getTempPath();
    dai::Path path = dai::platform::joinPaths(tempPath, "+ěščřžýáíé%=§_mačča.txt");

    REQUIRE(!std::filesystem::exists(path));

    // Create it
    std::filesystem::create_directories(path);
    REQUIRE(std::filesystem::exists(path));

    // Remove it
    std::filesystem::remove(path);
    REQUIRE(!std::filesystem::exists(path));
}

TEST_CASE("Test existence of directories with non-ascii characters") {
    dai::Path tempPath = dai::platform::getTempPath();
    dai::Path path = dai::platform::joinPaths(tempPath, "+ěščřžýáíé%=§_mačča");

    REQUIRE(!std::filesystem::exists(path));

    // Create it
    std::filesystem::create_directories(path);
    REQUIRE(std::filesystem::exists(path));

    dai::Path path2 = dai::platform::joinPaths(path, "soupy_mačča");
    REQUIRE(!std::filesystem::exists(path2));

    // Create it
    std::filesystem::create_directories(path2);
    REQUIRE(std::filesystem::exists(path2));

    // Remove it
    std::filesystem::remove(path2);
    std::filesystem::remove(path);
}