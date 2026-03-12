#include "depthai/pipeline/node/MessageDemux.hpp"

namespace dai {
namespace node {

MessageDemux::~MessageDemux() = default;

void MessageDemux::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

ProcessorType MessageDemux::getProcessor() const {
    return properties.processor;
}

}  // namespace node
}  // namespace dai
