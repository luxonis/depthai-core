#include "depthai/pipeline/node/Pool.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

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
