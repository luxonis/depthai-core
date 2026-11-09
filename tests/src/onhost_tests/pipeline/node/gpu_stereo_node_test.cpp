#include <algorithm>
#include <catch2/catch_all.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/node/GPUStereo.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>
#include <type_traits>

static_assert(!std::is_same_v<dai::node::StereoDepth::Backend, dai::Backend>, "StereoDepth must have its own backend enum");

TEST_CASE("Rectification backend selection survives host-device serialization", "[Rectification]") {
    auto node = dai::node::Rectification::create();
    REQUIRE(node->properties.backend == dai::Backend::AUTO);
    node->properties.mesh.meshLeftUri = "asset:/node/1/meshLeft";
    node->properties.mesh.meshRightUri = "asset:/node/1/meshRight";
    node->properties.mesh.meshSize = 256;
    node->properties.mesh.stepWidth = 32;
    node->properties.rectifyEdgeFillColor = -1;
    node->properties.useHomographyRectification = true;
    node->properties.rectificationUseSpecTranslation = false;
    node->properties.alphaScaling = 0.5f;
    node->properties.depthAlign = dai::StereoDepthProperties::DepthAlign::RECTIFIED_RIGHT;
    node->properties.depthAlignCamera = dai::CameraBoardSocket::CAM_C;
    node->properties.numFramesPool = 7;
    for(auto backend : {dai::Backend::CPU, dai::Backend::HW, dai::Backend::GPU, dai::Backend::AUTO}) {
        REQUIRE(&node->setBackend(backend) == node.get());
        for(auto format : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
            std::vector<uint8_t> data;
            node->getProperties().serialize(data, format);
            dai::RectificationProperties decoded;
            REQUIRE(dai::utility::deserialize(data, decoded, format));
            REQUIRE(decoded.backend == backend);
            REQUIRE(decoded.mesh.meshLeftUri == node->properties.mesh.meshLeftUri);
            REQUIRE(decoded.mesh.meshRightUri == node->properties.mesh.meshRightUri);
            REQUIRE(decoded.mesh.meshSize == 256);
            REQUIRE(decoded.mesh.stepWidth == 32);
            REQUIRE(decoded.rectifyEdgeFillColor == -1);
            REQUIRE(decoded.useHomographyRectification == true);
            REQUIRE(decoded.rectificationUseSpecTranslation == false);
            REQUIRE(decoded.alphaScaling == 0.5f);
            REQUIRE(decoded.depthAlign == dai::StereoDepthProperties::DepthAlign::RECTIFIED_RIGHT);
            REQUIRE(decoded.depthAlignCamera == dai::CameraBoardSocket::CAM_C);
            REQUIRE(decoded.numFramesPool == 7);
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

TEST_CASE("StereoDepth follows Rectification then Sync, with a true bypass when disabled", "[StereoDepth][Rectification]") {
    const bool enabled = GENERATE(true, false);
    const auto backend = GENERATE(dai::node::StereoDepth::Backend::HW, dai::node::StereoDepth::Backend::GPU, dai::node::StereoDepth::Backend::GPU_CPU);
    dai::Pipeline pipeline(false);
    struct HostStereoDepth : dai::node::StereoDepth {
        bool runOnHost() const override {
            return true;
        }
    };
    auto stereo = std::make_shared<HostStereoDepth>();
    pipeline.add(stereo);
    stereo->buildInternal();
    stereo->rectification->setRunOnHost(true);
    for(auto& child : stereo->getNodeMap()) {
        if(auto sync = std::dynamic_pointer_cast<dai::node::Sync>(child)) sync->setRunOnHost(true);
    }
    stereo->setBackend(backend);
    const bool configureBeforeLinking = GENERATE(true, false);
    if(configureBeforeLinking) stereo->setRectification(enabled);
    auto left = stereo->left.createInputQueue();
    auto right = stereo->right.createInputQueue();
    auto syncedLeft = stereo->syncedLeft.createOutputQueue();
    auto syncedRight = stereo->syncedRight.createOutputQueue();
    auto sourceOf = [&](dai::Node::Input& input) -> dai::Node::Output* {
        for(auto& node : pipeline.getAllNodes()) {
            for(auto* output : node->getOutputRefs()) {
                for(auto& connection : output->getConnections()) {
                    if(connection.in == &input) return output;
                }
            }
        }
        return nullptr;
    };
    auto* leftSource = sourceOf(stereo->left);
    auto* rightSource = sourceOf(stereo->right);
    REQUIRE(leftSource != nullptr);
    REQUIRE(rightSource != nullptr);
    // Changing the route after producer links exist must preserve those links.
    if(!configureBeforeLinking) {
        stereo->setRectification(!enabled);
        stereo->setRectification(enabled);
    }
    REQUIRE(stereo->rectification->properties.enableRectification == enabled);
    // StereoDepth owns the rectification toggle, including when the subnode was configured directly.
    stereo->rectification->enableRectification(false);
    stereo->left.setMaxSize(7);
    stereo->left.setBlocking(false);

    pipeline.build();
    std::shared_ptr<dai::node::Sync> sync;
    for(const auto& child : stereo->getNodeMap()) {
        if(auto candidate = std::dynamic_pointer_cast<dai::node::Sync>(child)) sync = candidate;
        REQUIRE(std::string(child->getName()) != "MessageDemux");
    }
    REQUIRE(sync != nullptr);
    REQUIRE(sync->getSyncThreshold() == std::chrono::nanoseconds(16'666'666));
    REQUIRE(stereo->getNodeMap().size() == (enabled ? 2 : 1));
    REQUIRE(sync->out.getConnections().at(0).in == stereo->getInputRef("inputSynced"));
    REQUIRE_FALSE(stereo->properties.enableRectification);
    REQUIRE(stereo->syncedRight.getQueueConnections().at(0).queue == syncedRight);
    auto& leftInput = enabled ? stereo->rectification->input1 : sync->inputs["left"];
    auto& rightInput = enabled ? stereo->rectification->input2 : sync->inputs["right"];
    REQUIRE(leftSource->getConnections().at(0).in == &leftInput);
    REQUIRE(rightSource->getConnections().at(0).in == &rightInput);
    REQUIRE(leftInput.getMaxSize() == 7);
    REQUIRE_FALSE(leftInput.getBlocking());
    auto leftFrame = std::make_shared<dai::ImgFrame>();
    auto rightFrame = std::make_shared<dai::ImgFrame>();
    leftSource->send(leftFrame);
    rightSource->send(rightFrame);
    REQUIRE(leftInput.tryGet<dai::ImgFrame>() == leftFrame);
    REQUIRE(rightInput.tryGet<dai::ImgFrame>() == rightFrame);
    if(enabled) {
        REQUIRE(stereo->rectification->properties.enableRectification);
        REQUIRE(stereo->rectification->output1.getConnections().at(0).in == &sync->inputs["left"]);
        REQUIRE(stereo->rectification->output2.getConnections().at(0).in == &sync->inputs["right"]);
        REQUIRE(stereo->rectification->passthrough1.getConnections().at(0).in == &sync->inputs["originalLeft"]);
        REQUIRE(stereo->rectification->passthrough2.getConnections().at(0).in == &sync->inputs["originalRight"]);
    } else {
        REQUIRE(sync->inputs.size() == 2);
        REQUIRE_FALSE(stereo->rectification->input1.isConnected());
        REQUIRE_FALSE(stereo->rectification->input2.isConnected());
    }
}
