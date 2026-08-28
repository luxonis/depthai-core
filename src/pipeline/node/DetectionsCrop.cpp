#include "depthai/pipeline/node/DetectionsCrop.hpp"

namespace dai {
namespace node {

DetectionsCrop::DetectionsCrop()
    : DeviceNodeGroup(nullptr), inputDetections{cropConfigGenerator->inputDetections}, inputImage{cropConfigGenerator->inputImage}, out{imageManip->out} {}

DetectionsCrop::DetectionsCrop(const std::shared_ptr<Device>& device)
    : DeviceNodeGroup(device), inputDetections{cropConfigGenerator->inputDetections}, inputImage{cropConfigGenerator->inputImage}, out{imageManip->out} {}

DetectionsCrop::~DetectionsCrop() = default;

std::shared_ptr<DetectionsCrop> DetectionsCrop::create() {
    auto node = std::make_shared<DetectionsCrop>();
    node->buildInternal();
    return node;
}

std::shared_ptr<DetectionsCrop> DetectionsCrop::create(const std::shared_ptr<Device>& device) {
    auto node = std::make_shared<DetectionsCrop>(device);
    node->buildInternal();
    return node;
}

void DetectionsCrop::buildInternal() {
    cropConfigGenerator->outConfig.link(imageManip->inputConfig);
    cropConfigGenerator->outImage.link(imageManip->inputImage);

    // Runtime configs and repeated images are emitted in matching FIFO order.
    // Waiting for config makes ImageManip consume exactly one of each per crop.
    imageManip->inputConfig.setWaitForMessage(true);
}

DetectionsCrop& DetectionsCrop::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
    cropConfigGenerator->setRunOnHost(runOnHost);
    imageManip->setRunOnHost(runOnHost);
    return *this;
}

bool DetectionsCrop::runOnHost() const {
    return runOnHostVar;
}

}  // namespace node
}  // namespace dai
