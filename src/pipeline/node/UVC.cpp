#include "depthai/pipeline/node/UVC.hpp"

namespace dai {
namespace node {

#if defined(__clang__)
UVC::~UVC() = default;
#endif

UVC::UVC(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, UVC, UVCProperties>(std::move(props)) {}

void UVC::setGpiosOnInit(std::unordered_map<int, int> list) {
    properties.gpioInit = list;
}

void UVC::setGpiosOnStreamOn(std::unordered_map<int, int> list) {
    properties.gpioStreamOn = list;
}

void UVC::setGpiosOnStreamOff(std::unordered_map<int, int> list) {
    properties.gpioStreamOff = list;
}

}  // namespace node
}  // namespace dai
