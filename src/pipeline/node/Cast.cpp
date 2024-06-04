#include "depthai/pipeline/node/Cast.hpp"

namespace dai {
namespace node {

Cast::Cast(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Cast(par, nodeId, std::make_unique<Cast::Properties>()) {}
Cast::Cast(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, Cast, CastProperties>(par, nodeId, std::move(props)) {
    setInputRefs({&input});
    setOutputRefs({&output, &passthroughInput});
}

Cast& Cast::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
    return *this;
}

Cast& Cast::setOutputFrameType(dai::RawImgFrame::Type outputType) {
    properties.outputType = outputType;
    return *this;
}

Cast& Cast::setScale(float scale) {
    properties.scale = scale;
    return *this;
}

Cast& Cast::setOffset(float offset) {
    properties.offset = offset;
    return *this;
}

}  // namespace node
}  // namespace dai
