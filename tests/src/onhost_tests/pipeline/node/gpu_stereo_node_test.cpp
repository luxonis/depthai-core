#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/node/GPUStereo.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>
#include <depthai/properties/RectificationProperties.hpp>

TEST_CASE("GPUStereo rectification toggle updates Rectification subnode properties", "[GPUStereo]") {
    auto node = dai::node::GPUStereo::create();

    node->setRectification(false);
    {
        auto& rectProps = (*node->rectification)->properties;
        REQUIRE(rectProps.enableRectification == false);
    }

    node->setRectification(true);
    {
        auto& rectProps = (*node->rectification)->properties;
        REQUIRE(rectProps.enableRectification == true);
    }
}

TEST_CASE("GPUStereo confidence threshold is clamped to [0, 255]", "[GPUStereo]") {
    auto node = dai::node::GPUStereo::create();

    node->initialConfig->setConfidenceThreshold(-1);
    REQUIRE(node->initialConfig->confidenceThreshold == static_cast<std::uint8_t>(0));

    node->initialConfig->setConfidenceThreshold(0);
    REQUIRE(node->initialConfig->confidenceThreshold == static_cast<std::uint8_t>(0));

    node->initialConfig->setConfidenceThreshold(255);
    REQUIRE(node->initialConfig->confidenceThreshold == static_cast<std::uint8_t>(255));

    node->initialConfig->setConfidenceThreshold(256);
    REQUIRE(node->initialConfig->confidenceThreshold == static_cast<std::uint8_t>(255));
}
