#include <catch2/catch_all.hpp>
#include <memory>
#include <stdexcept>
#include <string>

#include "depthai/beta/node/ToFStereoFusion.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/Rectification.hpp"

using namespace dai;
using Model = beta::ToFStereoFusionProperties::Model;

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

}  // namespace

TEST_CASE("ToFStereoFusion STEREO_NETWORK wires rectification and the ToF fusion mode instead of NeuralDepth", "[ToFStereoFusion]") {
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) SKIP("needs an RVC4 device with a ToF sensor");

    Pipeline pipeline(device);
    auto left = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<beta::node::ToFStereoFusion>();
    REQUIRE_THROWS_AS(fusion->setNetworkInputSize(1000, 640), std::invalid_argument);
    REQUIRE_THROWS_AS(fusion->setNetworkInputSize(1024, 650), std::invalid_argument);
    fusion->setModel(Model::STEREO_NETWORK).setNetworkInputSize(1024, 640);
    REQUIRE(fusion->getModel() == Model::STEREO_NETWORK);
    fusion->build(left, right);

    REQUIRE(countNodes<node::NeuralDepth>(*fusion) == 0);
    REQUIRE(countNodes<node::Rectification>(*fusion) == 1);
    REQUIRE(fusion->tof->tofBaseNode.initialConfig->depthMode == ToFConfig::DepthMode::FUSION);
    REQUIRE_FALSE(fusion->neuralNetwork->properties.deviceModel.has_value());
    REQUIRE(fusion->neuralNetwork->properties.backendProperties.at("output_tensors") == "disp_low,head9,conf");
    REQUIRE(fusion->neuralNetwork->properties.backendProperties.at("output_memory") == "dma_buf");
}

TEST_CASE("ToFStereoFusion STEREO_NETWORK with the vendor ToF confidence keeps the standard ToF mode", "[ToFStereoFusion]") {
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) SKIP("needs an RVC4 device with a ToF sensor");

    Pipeline pipeline(device);
    auto left = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<beta::node::ToFStereoFusion>();
    fusion->setModel(Model::STEREO_NETWORK).setToFConfidenceScale(beta::ToFStereoFusionProperties::ToFConfidenceScale::VENDOR);
    fusion->build(left, right);

    REQUIRE(fusion->tof->tofBaseNode.initialConfig->depthMode == ToFConfig::DepthMode::STANDARD);
}
