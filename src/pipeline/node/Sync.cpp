#include "depthai/pipeline/node/Sync.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

Sync::Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Sync(par, nodeId, std::make_unique<Sync::Properties>()) {}
Sync::Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, Sync, SyncProperties>(par, nodeId, std::move(props)) {
    setInputRefs({&input1, &input2, &input3});
    setOutputRefs({&output1, &output2, &output3});
}

Sync::Properties& Sync::getProperties() {
    return properties;
}

}  // namespace node
}  // namespace dai
