#include "depthai/pipeline/node/ImageManipV2.hpp"
namespace dai {
namespace node {

ImageManipV2::ImageManipV2(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, ImageManipV2, ImageManipPropertiesV2>(std::move(props)) {}

ImageManipV2::Properties& ImageManipV2::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

void ImageManipV2::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void ImageManipV2::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

}  // namespace node
}  // namespace dai
