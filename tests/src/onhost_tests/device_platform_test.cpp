#include <catch2/catch_test_macros.hpp>
#include <stdexcept>

#include "depthai/device/Platform.hpp"

static_assert(static_cast<int>(dai::Platform::RVC2) == 0);
static_assert(static_cast<int>(dai::Platform::RVC3) == 1);
static_assert(static_cast<int>(dai::Platform::RVC4) == 2);

TEST_CASE("Device platform names include RVC2NG", "[platform]") {
    REQUIRE(dai::platform2string(dai::Platform::RVC2) == "RVC2");
    REQUIRE(dai::platform2string(dai::Platform::RVC3) == "RVC3");
    REQUIRE(dai::platform2string(dai::Platform::RVC4) == "RVC4");
    REQUIRE(dai::platform2string(dai::Platform::RVC2NG) == "RVC2NG");
    REQUIRE(dai::string2platform("RVC2NG") == dai::Platform::RVC2NG);
    REQUIRE(dai::string2platform("rvc2ng") == dai::Platform::RVC2NG);
    REQUIRE(dai::string2platform("RvC2Ng") == dai::Platform::RVC2NG);
    REQUIRE(dai::string2platform("rvc2") == dai::Platform::RVC2);
    REQUIRE(dai::string2platform("rvc3") == dai::Platform::RVC3);
    REQUIRE(dai::string2platform("rvc4") == dai::Platform::RVC4);
    REQUIRE_THROWS_AS(dai::string2platform("unknown"), std::invalid_argument);
    REQUIRE_THROWS_AS(dai::platform2string(static_cast<dai::Platform>(99)), std::invalid_argument);
}
