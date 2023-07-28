#include "depthai/pipeline/node/UVC.hpp"

namespace dai {
namespace node {

UVC::UVC() : NodeCRTP<DeviceNode, UVC, UVCProperties>() {}
UVC::UVC(std::unique_ptr<Properties> props) : NodeCRTP<DeviceNode, UVC, UVCProperties>(std::move(props)) {}

void UVC::build() {
    ;
}

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
