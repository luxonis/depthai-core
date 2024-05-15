#include "depthai/pipeline/node/ImageAlign.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ImageAlign::ImageAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : ImageAlign(par, nodeId, std::make_unique<ImageAlign::Properties>()) {}
ImageAlign::ImageAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, ImageAlign, ImageAlignProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawImageAlignConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &input, &inputAlignTo});
    setOutputRefs({&outputAligned, &passthroughInput});
}

ImageAlign::Properties& ImageAlign::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

ImageAlign& ImageAlign::setOutputSize(int alignWidth, int alignHeight) {
    properties.alignWidth = alignWidth;
    properties.alignHeight = alignHeight;
    return *this;
}

}  // namespace node
}  // namespace dai