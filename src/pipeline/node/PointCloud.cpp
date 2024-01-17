#include "depthai/pipeline/node/PointCloud.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

PointCloud::PointCloud(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : PointCloud(par, nodeId, std::make_unique<PointCloud::Properties>()) {}
PointCloud::PointCloud(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, PointCloud, PointCloudProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawPointCloudConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputDepth, &inputColor});
    setOutputRefs({&out, &passthroughDepth});
}

PointCloud::Properties& PointCloud::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void PointCloud::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool PointCloud::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

}  // namespace node
}  // namespace dai
