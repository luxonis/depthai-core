#include "depthai/pipeline/node/Pool.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

void Pool::setNumMessages(tl::optional<int> num) {
    properties.numMessages = num;
}

tl::optional<int> Pool::getNumMessages() const {
    return properties.numMessages;
}

void Pool::setMaxMessageSize(tl::optional<std::int64_t> size) {
    properties.maxMessageSize = size;
}

tl::optional<std::int64_t> Pool::getMaxMessageSize() const {
    return properties.maxMessageSize;
}

void Pool::setDatatype(tl::optional<DatatypeEnum> type) {
    properties.datatype = type;
}

tl::optional<DatatypeEnum> Pool::getDatatype() const {
    return properties.datatype;
}

void Pool::setProcessor(tl::optional<ProcessorType> proc) {
    properties.processor = proc;
}

tl::optional<ProcessorType> Pool::getProcessor() const {
    return properties.processor;
}

}  // namespace node
}  // namespace dai
