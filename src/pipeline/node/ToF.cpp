#include "depthai/pipeline/node/ToF.hpp"

namespace dai {
namespace node {

ToF::ToF(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : ToF(par, nodeId, std::make_unique<ToF::Properties>()) {}
ToF::ToF(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, ToF, ToFProperties>(par, nodeId, std::move(props)), rawConfig(std::make_shared<RawToFConfig>()), initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &input});
    setOutputRefs({&depth, &amplitude, &intensity, &phase});
}

ToF::Properties& ToF::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

ToF& ToF::setNumShaves(int numShaves) {
    properties.numShaves = numShaves;
    return *this;
}

ToF& ToF::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
    return *this;
}

}  // namespace node
}  // namespace dai
