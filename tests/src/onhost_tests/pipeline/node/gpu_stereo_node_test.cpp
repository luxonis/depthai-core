#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/node/GPUStereo.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>
#include <depthai/properties/RectificationProperties.hpp>

TEST_CASE("Rectification backend selection survives host-device serialization", "[Rectification]") {
    auto node = dai::node::Rectification::create();
    REQUIRE(node->properties.backend == dai::Backend::AUTO);
    for(auto backend : {dai::Backend::CPU, dai::Backend::HW, dai::Backend::GPU, dai::Backend::AUTO}) {
        REQUIRE(&node->setBackend(backend) == node.get());
        for(auto format : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
            std::vector<uint8_t> data;
            node->getProperties().serialize(data, format);
            dai::RectificationProperties decoded;
            REQUIRE(dai::utility::deserialize(data, decoded, format));
            REQUIRE(decoded.backend == backend);
        }
    }
}

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
