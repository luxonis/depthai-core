#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <depthai/common/DepthUnit.hpp>

namespace {

constexpr float kPi = 3.14159265358979323846f;

float toUnit(float meters, dai::DepthUnit unit) {
    return meters * dai::getLengthUnitMultiplier(unit);
}

float toMeters(float value, dai::DepthUnit unit) {
    return value / dai::getLengthUnitMultiplier(unit);
}

float toAngleUnit(float radians, dai::AngleUnit unit) {
    return radians * dai::getAngleUnitToRadianMultiplier(unit);
}

float toRadians(float value, dai::AngleUnit unit) {
    return value / dai::getAngleUnitToRadianMultiplier(unit);
}

}  // namespace

TEST_CASE("DepthUnit multipliers", "[DepthUnit]") {
    REQUIRE(dai::getLengthUnitMultiplier(dai::DepthUnit::METER) == Catch::Approx(1.0f));
    REQUIRE(dai::getLengthUnitMultiplier(dai::DepthUnit::CENTIMETER) == Catch::Approx(100.0f));
    REQUIRE(dai::getLengthUnitMultiplier(dai::DepthUnit::MILLIMETER) == Catch::Approx(1000.0f));
    REQUIRE(dai::getLengthUnitMultiplier(dai::DepthUnit::INCH) == Catch::Approx(39.3701f));
    REQUIRE(dai::getLengthUnitMultiplier(dai::DepthUnit::FOOT) == Catch::Approx(3.28084f));
    REQUIRE(dai::getLengthUnitMultiplier(dai::DepthUnit::CUSTOM) == Catch::Approx(1.0f));
}

TEST_CASE("DepthUnit conversions", "[DepthUnit]") {
    constexpr float depthMeters = 2.5f;

    REQUIRE(toUnit(depthMeters, dai::DepthUnit::CENTIMETER) == Catch::Approx(250.0f));
    REQUIRE(toUnit(depthMeters, dai::DepthUnit::MILLIMETER) == Catch::Approx(2500.0f));
    REQUIRE(toUnit(depthMeters, dai::DepthUnit::FOOT) == Catch::Approx(8.2021f));

    constexpr float depthMm = 750.0f;
    REQUIRE(toMeters(depthMm, dai::DepthUnit::MILLIMETER) == Catch::Approx(0.75f));
}

TEST_CASE("AngleUnit multipliers", "[AngleUnit]") {
    REQUIRE(dai::getAngleUnitToRadianMultiplier(dai::AngleUnit::RADIAN) == Catch::Approx(1.0f));
    REQUIRE(dai::getAngleUnitToRadianMultiplier(dai::AngleUnit::DEGREE) == Catch::Approx(180.0f / kPi));
}

TEST_CASE("AngleUnit conversions", "[AngleUnit]") {
    constexpr float radians = kPi / 2.0f;

    REQUIRE(toAngleUnit(radians, dai::AngleUnit::DEGREE) == Catch::Approx(90.0f));
    REQUIRE(toRadians(180.0f, dai::AngleUnit::DEGREE) == Catch::Approx(kPi));
}
