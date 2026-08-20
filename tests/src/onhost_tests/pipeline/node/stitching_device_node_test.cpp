#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>
#include <type_traits>
#include <vector>

#include "beta/node/StitchingPlatform.hpp"

TEST_CASE("Stitching device execution is RVC4-only", "[Stitching]") {
    STATIC_REQUIRE(std::is_base_of_v<dai::beta::BetaNode, dai::beta::node::Stitching>);
    REQUIRE_FALSE(dai::beta::node::stitching::isDevicePlatformSupported(dai::Platform::RVC2));
    REQUIRE_FALSE(dai::beta::node::stitching::isDevicePlatformSupported(dai::Platform::RVC3));
    REQUIRE(dai::beta::node::stitching::isDevicePlatformSupported(dai::Platform::RVC4));
}

TEST_CASE("Stitching accepts deserialized properties", "[Stitching]") {
    auto properties = std::make_unique<dai::beta::StitchingProperties>();
    properties->mode = dai::beta::StitchingProperties::Mode::PLANAR_PROJECTION;
    properties->maxViewWidth = 1280;
    properties->numInputs = 2;

    auto stitching = std::make_shared<dai::beta::node::Stitching>(std::move(properties));

    REQUIRE(stitching->properties.mode == dai::beta::StitchingProperties::Mode::PLANAR_PROJECTION);
    REQUIRE(stitching->properties.maxViewWidth == 1280);
    REQUIRE(stitching->getNumInputs() == 2);
    REQUIRE(stitching->inputs.has("input0"));
    REQUIRE(stitching->inputs.has("input1"));
}

TEST_CASE("Stitching validates every source before building", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>();
    const std::vector<dai::Node::Output*> sources = {nullptr, nullptr};

    REQUIRE_THROWS(stitching->build(sources));
    REQUIRE(stitching->getNumInputs() == 0);
    REQUIRE_NOTHROW(stitching->build(2));
}

TEST_CASE("Stitching serializes as a device node", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(2);
    stitching->setRunOnHost(false);
    stitching->setMode(dai::beta::node::Stitching::Mode::PLANAR_PROJECTION);
    stitching->setPlane({1.0f, 2.0f, 3.0f}, {0.0f, 1.0f, 0.0f}, dai::LengthUnit::METER);
    stitching->setMaxViewSize(1280, 720);
    stitching->setMaxRange(4.0f, dai::LengthUnit::METER);
    stitching->setMinIncidenceAngle(12.0f);

    REQUIRE_FALSE(stitching->runOnHost());

    const auto serialized = dai::utility::serialize(stitching->properties);
    dai::beta::StitchingProperties properties;
    REQUIRE(dai::utility::deserialize(serialized, properties));
    REQUIRE(properties.mode == dai::beta::node::Stitching::Mode::PLANAR_PROJECTION);
    REQUIRE(properties.numInputs == 2);
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
    auto stitching = pipeline.create<dai::beta::node::Stitching>();

    REQUIRE(stitching->runOnHost());
    stitching->setRunOnHost(false);
    REQUIRE_FALSE(stitching->runOnHost());
    stitching->setRunOnHost(true);
    REQUIRE(stitching->runOnHost());
}
