#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>

TEST_CASE("Stitching serializes as a device node", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::node::Stitching>()->build(2);
    stitching->setRunOnHost(false);
    stitching->setMode(dai::node::Stitching::Mode::PLANAR_PROJECTION);
    stitching->setPlane({1.0f, 2.0f, 3.0f}, {0.0f, 1.0f, 0.0f}, dai::LengthUnit::METER);
    stitching->setMaxViewSize(1280, 720);
    stitching->setMaxRange(4.0f, dai::LengthUnit::METER);
    stitching->setMinIncidenceAngle(12.0f);

    REQUIRE_FALSE(stitching->runOnHost());

    const auto serialized = dai::utility::serialize(stitching->properties);
    dai::StitchingProperties properties;
    REQUIRE(dai::utility::deserialize(serialized, properties));
    REQUIRE(properties.mode == dai::node::Stitching::Mode::PLANAR_PROJECTION);
    REQUIRE(properties.plane.has_value());
    REQUIRE(properties.plane->point.x == 1.0f);
    REQUIRE(properties.plane->unit == dai::LengthUnit::METER);
    REQUIRE(properties.maxViewWidth == 1280);
    REQUIRE(properties.maxViewHeight == 720);
    REQUIRE(properties.maxRange == 400.0f);
    REQUIRE(properties.minIncidenceAngle == 12.0f);
}

TEST_CASE("Stitching runs on the host by default", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::node::Stitching>();

    REQUIRE(stitching->runOnHost());
    stitching->setRunOnHost(false);
    REQUIRE_FALSE(stitching->runOnHost());
    stitching->setRunOnHost(true);
    REQUIRE(stitching->runOnHost());
}
