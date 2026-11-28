#include "FpsRegulator.hpp"

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "utilities/ErrorMacros.hpp"

namespace dai {
namespace gate {

#ifdef ENALBE_DEBUG
    #define DEBUG(...) fprintf(stderr, ##__VA_ARGS__)
#else
    #define DEBUG(...)
#endif

FpsRegulator::FpsRegulator(float inputFps, float outputFps, Node::Output* output, std::shared_ptr<spdlog::async_logger> logger)
    : inputFps(inputFps), outputFps(outputFps), output(output), logger(logger) {}

FpsRegulator::FpsRegulator(float inputFps, float outputFps, MessageQueue* output, std::shared_ptr<spdlog::async_logger> logger)
    : inputFps(inputFps), outputFps(outputFps), outputQueue(output), logger(logger) {}

void FpsRegulator::run() {
    mainThread = std::thread(&FpsRegulator::mainLoop, this);
}

void FpsRegulator::mainLoop() {
    uint32_t frameCount = 0;
    constexpr auto GRANULARITY = 1000;
    std::array<bool, GRANULARITY> frameKeepArray{false};
    float keepInterval = (float)inputFps / (float)outputFps;
    float nextKeepFrame = 0.0f;
    uint32_t keptFrames = 0;
    auto targetKeepFrames = keepInterval * GRANULARITY;
    for(uint32_t i = 0; i < GRANULARITY; ++i) {
        if(i >= nextKeepFrame) {
            frameKeepArray[i] = true;
            nextKeepFrame += keepInterval;
            keptFrames++;
        }
    }
    while(isAlive) {
        std::shared_ptr<dai::ADatatype> msg = inQueue.get();
        if(msg == nullptr) {
            logger->error("Got null for FpsRegulator input message. Skipping this one.");
            continue;
        }
        if(frameKeepArray[frameCount % GRANULARITY]) {
            if(output != nullptr) {
                output->send(msg);
            } else if(outputQueue != nullptr) {
                outputQueue->send(msg);
            } else {
                DAI_CHECK_IN(false);
            }
        }
        frameCount++;
    }
}

void FpsRegulator::stop() {
    isAlive = false;
    if(mainThread.joinable()) {
        mainThread.join();
    }
}

}  // namespace gate
}  // namespace dai
