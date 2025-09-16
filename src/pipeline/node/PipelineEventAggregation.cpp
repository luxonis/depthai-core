#include "depthai/pipeline/node/PipelineEventAggregation.hpp"

namespace dai {
namespace node {

void PipelineEventAggregation::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool PipelineEventAggregation::runOnHost() const {
    return runOnHostVar;
}

void PipelineEventAggregation::run() {
    while(isRunning()) {

    }
}
}  // namespace node
}  // namespace dai
