#include "depthai/pipeline/node/UVC.hpp"

namespace dai {
namespace node {

UVC::UVC(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, UVC, UVCProperties>(std::move(props)) {}

UVC::~UVC() = default;

void UVC::setGpiosOnInit(const std::unordered_map<int, int>& list) {
    properties.gpioInit = list;
}

void UVC::setGpiosOnStreamOn(const std::unordered_map<int, int>& list) {
    properties.gpioStreamOn = list;
}

void UVC::setGpiosOnStreamOff(const std::unordered_map<int, int>& list) {
    properties.gpioStreamOff = list;
}

}  // namespace node
}  // namespace dai
