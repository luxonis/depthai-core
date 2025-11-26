#include "depthai/pipeline/node/Sync.hpp"

#include <chrono>

#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {

void Sync::setSyncThreshold(std::chrono::nanoseconds syncThreshold) {
    properties.syncThresholdNs = syncThreshold.count();
}

void Sync::setSyncAttempts(int syncAttempts) {
    properties.syncAttempts = syncAttempts;
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
            // Print out the timestamps
            for(const auto& frame : inputFrames) {
                logger->debug("Starting input {} timestamp is {} ms",
                              frame.first,
                              static_cast<float>(frame.second->getTimestamp().time_since_epoch().count()) / 1000000.f);
            }
            tAfterMessageBeginning = steady_clock::now();
            int attempts = 0;
            while(true) {
                logger->trace("There have been {} attempts to sync", attempts);
                if(attempts > 50) {
                    logger->warn("Sync node has been trying to sync for {} messages, but the messages are still not in sync.", attempts);
                    for(const auto& frame : inputFrames) {
                        logger->warn("Output {} timestamp is {} ms",
                                     frame.first,
                                     static_cast<float>(frame.second->getTimestamp().time_since_epoch().count()) / 1000000.f);
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
                // Find a minimum timestamp
                auto minTs = inputFrames.begin()->second->getTimestamp();
                for(const auto& frame : inputFrames) {
                    if(frame.second->getTimestamp() < minTs) {
                        minTs = frame.second->getTimestamp();
                    }
                }

                // Find a max timestamp
                auto maxTs = inputFrames.begin()->second->getTimestamp();
                for(const auto& frame : inputFrames) {
                    if(frame.second->getTimestamp() > maxTs) {
                        maxTs = frame.second->getTimestamp();
                    }
                }
                logger->debug("Diff: {} ms", duration_cast<milliseconds>(maxTs - minTs).count());

                if(duration_cast<nanoseconds>(maxTs - minTs).count() < syncThresholdNs) {
                    break;
                }

                // Get the message with the minimum timestamp (oldest message)
                std::string minTsName;
                for(const auto& frame : inputFrames) {
                    if(frame.second->getTimestamp() == minTs) {
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
        dai::Buffer* newestFrame = inputFrames.begin()->second.get();
        for(const auto& name : inputNames) {
            logger->trace("Sending output: {}", name);
            logger->trace("Timestamp: {} ms",
                          static_cast<float>(duration_cast<microseconds>(inputFrames[name]->getTimestamp().time_since_epoch()).count()) / 1000.f);
            outputGroup->add(name, inputFrames[name]);
            if(inputFrames[name]->getTimestamp() > newestFrame->getTimestamp()) {
                newestFrame = inputFrames[name].get();
            }
        }
        outputGroup->setTimestamp(newestFrame->getTimestamp());
        outputGroup->setTimestampDevice(newestFrame->getTimestampDevice());
        outputGroup->setTimestampSystem(newestFrame->getTimestampSystem());
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
