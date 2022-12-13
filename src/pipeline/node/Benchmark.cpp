#include "depthai/pipeline/node/BenchmarkOut.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/BenchmarkIn.hpp"

namespace dai {
namespace node {

void BenchmarkOut::setNumMessagesToSend(int num) {
    properties.numMessages = num;
}

void BenchmarkOut::setFps(float fps) {
    properties.fps = fps;
}

void BenchmarkIn::setNumMessagesToGet(int num) {
    properties.numMessages = num;
}

}  // namespace node
}  // namespace dai
