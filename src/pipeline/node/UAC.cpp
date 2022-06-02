#include "depthai/pipeline/node/UAC.hpp"

namespace dai {
namespace node {

UAC::UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : UAC(par, nodeId, std::make_unique<UAC::Properties>()) {}
UAC::UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, UAC, UACProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawAudioInConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &input});
}

UAC::Properties& UAC::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

}  // namespace node
}  // namespace dai
