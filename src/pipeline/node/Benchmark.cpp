#include "depthai/pipeline/node/BenchmarkIn.hpp"
#include "depthai/pipeline/node/BenchmarkOut.hpp"

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

void BenchmarkOut::buildInternal() {
    properties.numMessages = -1;  // By default send messages indefinitely
}

}  // namespace node
}  // namespace dai
