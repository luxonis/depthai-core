#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/node/GPUStereo.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>
#include <type_traits>

static_assert(!std::is_same_v<dai::node::StereoDepth::Backend, dai::Backend>, "StereoDepth must have its own backend enum");

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
    REQUIRE(node->properties.rectification == true);

    node->setRectification(false);
    REQUIRE(node->properties.rectification == false);

    node->setRectification(true);
    REQUIRE(node->properties.rectification == true);
}

TEST_CASE("GPUStereo compute mode defaults to GPU_CPU and is settable", "[GPUStereo]") {
    auto node = dai::node::GPUStereo::create();
    REQUIRE(node->initialConfig->getComputeMode() == dai::GPUStereoConfig::ComputeMode::GPU_CPU);

    node->initialConfig->setComputeMode(dai::GPUStereoConfig::ComputeMode::GPU_ONLY);
    REQUIRE(node->initialConfig->getComputeMode() == dai::GPUStereoConfig::ComputeMode::GPU_ONLY);
}

TEST_CASE("GPUStereo temporal filter defaults to on and is settable", "[GPUStereo]") {
    auto node = dai::node::GPUStereo::create();
    REQUIRE(node->initialConfig->getTemporalFilter() == true);

    node->initialConfig->setTemporalFilter(false);
    REQUIRE(node->initialConfig->getTemporalFilter() == false);
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

TEST_CASE("StereoDepth backend selection survives host-device serialization", "[StereoDepth][GPUStereo]") {
    auto node = dai::node::StereoDepth::create();
    REQUIRE(node->getBackend() == dai::node::StereoDepth::Backend::AUTO);
    for(auto backend : {dai::node::StereoDepth::Backend::AUTO,
                        dai::node::StereoDepth::Backend::HW,
                        dai::node::StereoDepth::Backend::GPU,
                        dai::node::StereoDepth::Backend::GPU_CPU}) {
        REQUIRE(&node->setBackend(backend) == node.get());
        REQUIRE(node->getBackend() == backend);
        for(auto format : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
            std::vector<uint8_t> data;
            static_cast<dai::DeviceNode&>(*node).getProperties().serialize(data, format);
            dai::StereoDepthProperties decoded;
            REQUIRE(dai::utility::deserialize(data, decoded, format));
            REQUIRE(decoded.backend == backend);
        }
    }
    REQUIRE_THROWS_AS(node->setBackend(static_cast<dai::node::StereoDepth::Backend>(0)), std::invalid_argument);
    REQUIRE_THROWS_AS(node->setBackend(static_cast<dai::node::StereoDepth::Backend>(255)), std::invalid_argument);
    REQUIRE(node->getBackend() == dai::node::StereoDepth::Backend::GPU_CPU);
}
