#include <catch2/catch_all.hpp>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "depthai/beta/node/ToFStereoFusion.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/Rectification.hpp"

using namespace dai;

namespace {

bool deviceHasTofSensor(const std::shared_ptr<Device>& device) {
    for(const auto& features : device->getConnectedCameraFeatures()) {
        for(const auto type : features.supportedTypes) {
            if(type == CameraSensorType::TOF) return true;
        }
    }
    return false;
}

template <typename T>
size_t countNodes(const Node& parent) {
    size_t count = 0;
    for(const auto& node : parent.getAllNodes()) {
        if(std::dynamic_pointer_cast<T>(node)) ++count;
        count += countNodes<T>(*node);
    }
    return count;
}

template <typename T>
std::shared_ptr<T> findNode(const Node& parent) {
    for(const auto& node : parent.getAllNodes()) {
        if(auto match = std::dynamic_pointer_cast<T>(node)) return match;
        if(auto match = findNode<T>(*node)) return match;
    }
    return nullptr;
}

}  // namespace

TEST_CASE("ToFStereoFusion wires rectification and the ToF fusion mode instead of NeuralDepth", "[ToFStereoFusion]") {
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) SKIP("needs an RVC4 device with a ToF sensor");

    Pipeline pipeline(device);
    auto left = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<beta::node::ToFStereoFusion>();
    // The network shipped with the firmware is selected at creation, so a later setModelPath() replaces it.
    REQUIRE(fusion->neuralNetwork->properties.backendProperties.at("firmware_model") == "TOF_STEREO_FUSION_1024x640.dlc");
    fusion->build(left, right);
    REQUIRE_THROWS_AS(fusion->build(left, right), std::logic_error);

    REQUIRE(countNodes<node::NeuralDepth>(*fusion) == 0);
    REQUIRE(countNodes<node::Rectification>(*fusion) == 1);
    const auto rectification = findNode<node::Rectification>(*fusion);
    REQUIRE(rectification != nullptr);
    REQUIRE(rectification->properties.backend == node::Rectification::Backend::HW);
    REQUIRE(rectification->properties.alphaScaling == std::optional<float>(0.0f));
    // A linked phase output selects the ToF fusion depth mode.
    REQUIRE_FALSE(fusion->tof->tofBaseNode.phase.getConnections().empty());
    REQUIRE_FALSE(fusion->neuralNetwork->properties.deviceModel.has_value());
    REQUIRE(fusion->neuralNetwork->properties.backendProperties.at("output_tensors") == "disp_low,head9,conf");
    REQUIRE(fusion->neuralNetwork->properties.backendProperties.at("output_memory") == "dma_buf");
    REQUIRE(fusion->neuralNetwork->properties.backendProperties.at("firmware_model") == "TOF_STEREO_FUSION_1024x640.dlc");
}
