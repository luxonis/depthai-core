#include "depthai/pipeline/node/Sync.hpp"

#include <chrono>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {
namespace {

using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

SteadyTimePoint getSyncTimestamp(const std::shared_ptr<dai::Buffer>& buffer) {
    if(auto imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(buffer)) {
        return imgFrame->getTimestamp(dai::CameraExposureOffset::END);
    }

    return buffer->getTimestamp();
}

SteadyTimePoint getSyncTimestampDevice(const std::shared_ptr<dai::Buffer>& buffer) {
    if(auto imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(buffer)) {
        return imgFrame->getTimestampDevice(dai::CameraExposureOffset::END);
    }

    return buffer->getTimestampDevice();
}

}  // namespace

void Sync::setSyncThreshold(std::chrono::nanoseconds syncThreshold) {
    properties.syncThresholdNs = syncThreshold.count();
}

void Sync::setSyncAttempts(int syncAttempts) {
    properties.syncAttempts = syncAttempts;
}

void Sync::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

ProcessorType Sync::getProcessor() const {
    return properties.processor;
}

std::chrono::nanoseconds Sync::getSyncThreshold() const {
    return std::chrono::nanoseconds(properties.syncThresholdNs);
}

int Sync::getSyncAttempts() const {
    return properties.syncAttempts;
}

void Sync::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool Sync::runOnHost() const {
    return runOnHostVar;
}

void Sync::run() {
    using namespace std::chrono;
    auto& logger = pimpl->logger;
    const auto inputsName = inputs.name;

    if(inputs.empty()) {
        throw std::runtime_error("Sync node must have at least 1 input!");
    }
    std::vector<std::string> inputNames;
    for(auto& in : inputs) {
        auto inputName = in.first.second;
        inputNames.push_back(inputName);
    }

    auto syncThresholdNs = properties.syncThresholdNs;
    logger->trace("Sync threshold: {}", syncThresholdNs);

    time_point<steady_clock> tAfterMessageBeginning;

    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        std::unordered_map<std::string, std::shared_ptr<dai::Buffer>> inputFrames;
        {
            auto blockEvent = this->inputBlockEvent();

            for(auto name : inputNames) {
                logger->trace("Receiving input: {}", name);
                inputFrames[name] = inputs[name].get<dai::Buffer>();
                if(inputFrames[name] == nullptr) {
                    logger->error("Received nullptr from input {}, sync node only accepts messages inherited from Buffer on the inputs", name);
                    throw std::runtime_error("Received nullptr from input " + name);
                }
            }
            for(const auto& frame : inputFrames) {
                logger->debug("Starting input {} sync timestamp is {} ms",
                              frame.first,
                              static_cast<float>(getSyncTimestampDevice(frame.second).time_since_epoch().count()) / 1000000.f);
            }
            tAfterMessageBeginning = steady_clock::now();
            int attempts = 0;
            while(true) {
                logger->trace("There have been {} attempts to sync", attempts);
                if(attempts > 50) {
                    logger->warn("Sync node has been trying to sync for {} messages, but the messages are still not in sync.", attempts);
                    for(const auto& frame : inputFrames) {
                        logger->warn("Output {} sync timestamp is {} ms",
                                     frame.first,
                                     static_cast<float>(getSyncTimestampDevice(frame.second).time_since_epoch().count()) / 1000000.f);
                    }
                }
                if(attempts > properties.syncAttempts && properties.syncAttempts != -1) {
                    if(properties.syncAttempts != 0)
                        logger->warn(
                            "Sync node has been trying to sync for {} messages, but the messages are still not in sync. "
                            "The node will send the messages anyway.",
                            attempts);
                    break;
                }

                auto minTs = getSyncTimestampDevice(inputFrames.begin()->second);
                for(const auto& frame : inputFrames) {
                    const auto ts = getSyncTimestampDevice(frame.second);
                    if(ts < minTs) {
                        minTs = ts;
                    }
                }

                auto maxTs = getSyncTimestampDevice(inputFrames.begin()->second);
                for(const auto& frame : inputFrames) {
                    const auto ts = getSyncTimestampDevice(frame.second);
                    if(ts > maxTs) {
                        maxTs = ts;
                    }
                }
                logger->debug("Diff: {} ms", duration_cast<milliseconds>(maxTs - minTs).count());

                if(duration_cast<nanoseconds>(maxTs - minTs).count() < syncThresholdNs) {
                    break;
                }

                std::string minTsName;
                for(const auto& frame : inputFrames) {
                    if(getSyncTimestampDevice(frame.second) == minTs) {
                        minTsName = frame.first;
                        break;
                    }
                }
                logger->trace("Receiving input: {}", minTsName);
                inputFrames[minTsName] = inputs[minTsName].get<dai::Buffer>();
                attempts++;
            }
        }
        auto tBeforeSend = steady_clock::now();
        auto outputGroup = std::make_shared<dai::MessageGroup>();
        auto newestFrame = inputFrames.begin()->second;
        auto newestTimestamp = getSyncTimestamp(newestFrame);
        auto newestTimestampDevice = getSyncTimestampDevice(newestFrame);
        for(const auto& name : inputNames) {
            logger->trace("Sending output: {}", name);
            logger->trace("Sync timestamp: {} ms",
                          static_cast<float>(duration_cast<microseconds>(getSyncTimestampDevice(inputFrames[name]).time_since_epoch()).count()) /
                              1000.f);
            outputGroup->add(name, inputFrames[name]);

            const auto tsDevice = getSyncTimestampDevice(inputFrames[name]);
            if(tsDevice > newestTimestampDevice) {
                newestFrame = inputFrames[name];
                newestTimestamp = getSyncTimestamp(inputFrames[name]);
                newestTimestampDevice = tsDevice;
            }
        }
        outputGroup->setTimestamp(newestTimestamp);
        outputGroup->setTimestampDevice(newestTimestampDevice);
        outputGroup->setSequenceNum(newestFrame->getSequenceNum());
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(outputGroup);
        }
        auto tAbsoluteEnd = steady_clock::now();
        logger->debug("Sync total took {}ms, processing {}ms, getting_frames {}ms, sending_frames {}ms",
                      duration_cast<microseconds>(tAbsoluteEnd - tAbsoluteBeginning).count() / 1000,
                      duration_cast<microseconds>(tBeforeSend - tAfterMessageBeginning).count() / 1000,
                      duration_cast<microseconds>(tAfterMessageBeginning - tAbsoluteBeginning).count() / 1000,
                      duration_cast<microseconds>(tAbsoluteEnd - tBeforeSend).count() / 1000);
    }
}
}  // namespace node
}  // namespace dai
