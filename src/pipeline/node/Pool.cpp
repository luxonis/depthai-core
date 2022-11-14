#include "depthai/pipeline/node/Pool.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

Pool::Pool(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : NodeCRTP<DeviceNode, Pool, PoolProperties>(par, nodeId, std::make_unique<Pool::Properties>()) {
    setOutputRefs(&out);
}

Pool::Pool(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, Pool, PoolProperties>(par, nodeId, std::move(props)) {
    setOutputRefs(&out);
}

void Pool::setNumMessages(int num) {
    properties.numMessages = num;
}

int Pool::getNumMessages() const {
    return properties.numMessages;
}

void Pool::setMaxMessageSize(std::int64_t size) {
    properties.maxMessageSize = size;
}

std::int64_t Pool::getMaxMessageSize() const {
    return properties.maxMessageSize;
}

}  // namespace node
}  // namespace dai
