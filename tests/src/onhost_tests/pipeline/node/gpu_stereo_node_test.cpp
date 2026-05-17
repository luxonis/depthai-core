#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/node/GPUStereo.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>
#include <depthai/properties/RectificationProperties.hpp>

namespace {
bool isLinked(dai::Node::Output& out, dai::Node::Input& in) {
    for(const auto& c : out.getConnections()) {
        if(c.in == &in) return true;
    }
    return false;
}
}  // namespace

TEST_CASE("GPUStereo rectification toggle updates Rectification subnode properties", "[GPUStereo]") {
    auto node = dai::node::GPUStereo::create();

    node->setRectification(false);
    {
        auto& rectProps = static_cast<dai::RectificationProperties&>(node->rectification->getProperties());
        REQUIRE(rectProps.enableRectification == false);
    }
    REQUIRE(isLinked(node->messageDemux->outputs["left"], node->leftInternal));
    REQUIRE(isLinked(node->messageDemux->outputs["right"], node->rightInternal));
    REQUIRE_FALSE(isLinked(node->messageDemux->outputs["left"], node->rectification->input1));
    REQUIRE_FALSE(isLinked(node->messageDemux->outputs["right"], node->rectification->input2));

    node->setRectification(true);
    {
        auto& rectProps = static_cast<dai::RectificationProperties&>(node->rectification->getProperties());
        REQUIRE(rectProps.enableRectification == true);
    }
    REQUIRE_FALSE(isLinked(node->messageDemux->outputs["left"], node->leftInternal));
    REQUIRE_FALSE(isLinked(node->messageDemux->outputs["right"], node->rightInternal));
    REQUIRE(isLinked(node->messageDemux->outputs["left"], node->rectification->input1));
    REQUIRE(isLinked(node->messageDemux->outputs["right"], node->rectification->input2));
}

TEST_CASE("GPUStereo confidence threshold is clamped to [0, 255]", "[GPUStereo]") {
    auto node = dai::node::GPUStereo::create();

    node->setConfidenceThreshold(-1);
    REQUIRE(node->initialConfig->confidenceThreshold == 0);

    node->setConfidenceThreshold(0);
    REQUIRE(node->initialConfig->confidenceThreshold == 0);

    node->setConfidenceThreshold(255);
    REQUIRE(node->initialConfig->confidenceThreshold == 255);

    node->setConfidenceThreshold(256);
    REQUIRE(node->initialConfig->confidenceThreshold == 255);
}
