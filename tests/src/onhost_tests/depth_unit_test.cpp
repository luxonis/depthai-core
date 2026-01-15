#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <depthai/common/DepthUnit.hpp>

namespace {

float toUnit(float meters, dai::DepthUnit unit) {
    return meters * dai::getDepthUnitMultiplier(unit);
}

float toMeters(float value, dai::DepthUnit unit) {
    return value / dai::getDepthUnitMultiplier(unit);
}

}  // namespace

TEST_CASE("DepthUnit multipliers", "[DepthUnit]") {
    REQUIRE(dai::getDepthUnitMultiplier(dai::DepthUnit::METER) == Catch::Approx(1.0f));
    REQUIRE(dai::getDepthUnitMultiplier(dai::DepthUnit::CENTIMETER) == Catch::Approx(100.0f));
    REQUIRE(dai::getDepthUnitMultiplier(dai::DepthUnit::MILLIMETER) == Catch::Approx(1000.0f));
    REQUIRE(dai::getDepthUnitMultiplier(dai::DepthUnit::INCH) == Catch::Approx(39.3701f));
    REQUIRE(dai::getDepthUnitMultiplier(dai::DepthUnit::FOOT) == Catch::Approx(3.28084f));
    REQUIRE(dai::getDepthUnitMultiplier(dai::DepthUnit::CUSTOM) == Catch::Approx(1.0f));
}

TEST_CASE("DepthUnit conversions", "[DepthUnit]") {
    constexpr float depthMeters = 2.5f;

    REQUIRE(toUnit(depthMeters, dai::DepthUnit::CENTIMETER) == Catch::Approx(250.0f));
    REQUIRE(toUnit(depthMeters, dai::DepthUnit::MILLIMETER) == Catch::Approx(2500.0f));
    REQUIRE(toUnit(depthMeters, dai::DepthUnit::FOOT) == Catch::Approx(8.2021f));

    constexpr float depthMm = 750.0f;
    REQUIRE(toMeters(depthMm, dai::DepthUnit::MILLIMETER) == Catch::Approx(0.75f));
}
