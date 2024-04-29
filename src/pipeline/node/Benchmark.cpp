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

std::shared_ptr<BenchmarkOut> BenchmarkOut::build() {
        properties.numMessages = -1;  // By default send messages indefinitely
        isBuild = true; 
        return std::static_pointer_cast<BenchmarkOut>(shared_from_this());
    }


}  // namespace node
}  // namespace dai
