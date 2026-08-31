#include "depthai/pipeline/node/DetectionsCrop.hpp"

namespace dai {
namespace node {

DetectionsCrop::DetectionsCrop(const std::shared_ptr<Device>& device)
    : DeviceNodeGroup(device), inputDetections{cropConfigGenerator->inputDetections}, inputImage{cropConfigGenerator->inputImage}, out{imageManip->out} {}

DetectionsCrop::~DetectionsCrop() = default;

void DetectionsCrop::buildInternal() {
    cropConfigGenerator->outConfig.link(imageManip->inputConfig);
    cropConfigGenerator->outImage.link(imageManip->inputImage);

    // Runtime configs and repeated images are emitted in matching FIFO order.
    // Waiting for config makes ImageManip consume exactly one of each per crop.
    imageManip->inputConfig.setWaitForMessage(true);
}

}  // namespace node
}  // namespace dai
