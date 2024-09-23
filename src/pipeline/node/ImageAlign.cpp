#include "depthai/pipeline/node/ImageAlign.hpp"


namespace dai {
namespace node {

ImageAlign::ImageAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : ImageAlign(par, nodeId, std::make_unique<ImageAlignProperties>()) {}
ImageAlign::ImageAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<ImageAlignProperties> props)
//: NodeCRTP<Node,ImageAlign>(par, nodeId)
    {
    //: NodeCRTP<Node, ImageAlign>(par, nodeId, std::move(props)),
      //rawConfig(std::make_shared<RawImageAlignConfig>()),
      //initialConfig(rawConfig) {
    

    //this->create(par,nodeId);
    setInputRefs({&inputConfig, &input, &inputAlignTo});
    setOutputRefs({&outputAligned, &passthroughInput});
}

ImageAlignProperties& ImageAlign::getProperties() {
    //properties.initialConfig = *rawConfig;
    return properties;
}

ImageAlign& ImageAlign::setOutputSize(int alignWidth, int alignHeight) {
    properties.alignWidth = alignWidth;
    properties.alignHeight = alignHeight;
    return *this;
}
ImageAlign& ImageAlign::setOutKeepAspectRatio(bool keep) {
    properties.outKeepAspectRatio = keep;
    return *this;
}

ImageAlign& ImageAlign::setInterpolation(Interpolation interp) {
    properties.interpolation = interp;
    return *this;
}

ImageAlign& ImageAlign::setNumShaves(int numShaves) {
    properties.numShaves = numShaves;
    return *this;
}

ImageAlign& ImageAlign::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
    return *this;
}

};  // namespace node
};  // namespace dai