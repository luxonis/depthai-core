#include "depthai/pipeline/node/DepthAlign.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

DepthAlign::DepthAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DepthAlign(par, nodeId, std::make_unique<DepthAlign::Properties>()) {}
DepthAlign::DepthAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, DepthAlign, DepthAlignProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawDepthAlignConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputDepth});
    setOutputRefs({&outputDepthAlign, &passthroughDepth});
}

DepthAlign::Properties& DepthAlign::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void DepthAlign::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool DepthAlign::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

}  // namespace node
}  // namespace dai