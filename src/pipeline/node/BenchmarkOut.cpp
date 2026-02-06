#include "depthai/pipeline/node/BenchmarkOut.hpp"

#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {

void BenchmarkOut::setNumMessagesToSend(int num) {
    properties.numMessages = num;
}

void BenchmarkOut::setFps(float fps) {
    properties.fps = fps;
}

void BenchmarkOut::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool BenchmarkOut::runOnHost() const {
    return runOnHostVar;
}

void BenchmarkOut::run() {
    using namespace std::chrono;
    auto& logger = pimpl->logger;
    std::shared_ptr<ADatatype> inMessage = nullptr;
    {
        auto blockEvent = this->inputBlockEvent();

        logger->trace("Wait for the input message.");
        inMessage = input.get();
    }

    bool useTiming = (properties.fps > 0);

    auto frameDurationDouble = std::chrono::duration<double>(1.0 / static_cast<double>(properties.fps));
    auto frameDuration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(frameDurationDouble);

    auto nextFrameTime = steady_clock::now();
    for(int i = 0; (i < properties.numMessages || properties.numMessages == -1) && mainLoop(); i++) {
        auto imgMessage = std::dynamic_pointer_cast<dai::ImgFrame>(inMessage);
        if(imgMessage != nullptr) {
            logger->trace("Sending img message with id {}", i);

            // Copying metadata and pointing to same data
            auto newMessage = std::make_shared<dai::ImgFrame>();
            newMessage->setMetadata(imgMessage);
            newMessage->data = imgMessage->data;
            if(runOnHostVar) {
                newMessage->setTimestamp(steady_clock::now());
            } else {
                newMessage->setTimestampDevice(steady_clock::now());
            }
            {
                auto blockEvent = this->outputBlockEvent();
                out.send(newMessage);
            }
        } else {
            logger->trace("Sending message with id {}", i);
            {
                auto blockEvent = this->outputBlockEvent();
                out.send(inMessage);
            }
        }

        if(useTiming) {
            nextFrameTime += frameDuration;

            auto now = steady_clock::now();
            if(nextFrameTime > now) {
                auto sleepTime = nextFrameTime - now;
                std::this_thread::sleep_for(sleepTime);
            }
        }
    }

    logger->trace("Benchmark out sent all messages!");
}

}  // namespace node
}  // namespace dai
