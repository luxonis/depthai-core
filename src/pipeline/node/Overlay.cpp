#include "depthai/pipeline/node/Overlay.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

Overlay::Overlay(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, Overlay, OverlayProperties>(std::move(props)) {}

Overlay& Overlay::setOverlayAlpha(float alpha) {
    properties.alpha = alpha;
    return *this;
}

Overlay& Overlay::setInterpolationType(int interpolationType) {
    properties.interpolationType = interpolationType;
    return *this;
}

Overlay& Overlay::setOutputSize(int width, int height) {
    properties.outputWidth = width;
    properties.outputHeight = height;
    return *this;
}

}  // namespace node
}  // namespace dai
