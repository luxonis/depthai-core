#include "depthai/pipeline/node/Pool.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

void Pool::setNumMessages(std::optional<int> num) {
    properties.numMessages = num;
}

std::optional<int> Pool::getNumMessages() const {
    return properties.numMessages;
}

void Pool::setMaxMessageSize(std::optional<std::int64_t> size) {
    properties.maxMessageSize = size;
}

std::optional<std::int64_t> Pool::getMaxMessageSize() const {
    return properties.maxMessageSize;
}

void Pool::setDatatype(std::optional<DatatypeEnum> type) {
    properties.datatype = type;
}

std::optional<DatatypeEnum> Pool::getDatatype() const {
    return properties.datatype;
}

void Pool::setProcessor(std::optional<ProcessorType> proc) {
    properties.processor = proc;
}

std::optional<ProcessorType> Pool::getProcessor() const {
    return properties.processor;
}

}  // namespace node
}  // namespace dai
