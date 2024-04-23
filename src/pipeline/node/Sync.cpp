#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"

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

    while(isRunning()) {
        auto tAbsoluteBeginning = steady_clock::now();
        std::unordered_map<std::string, std::shared_ptr<dai::Buffer>> inputFrames;
        for(auto name : inputNames) {
            logger->trace("Receiving input: {}", name);
            inputFrames[name] = inputs[name].get<dai::Buffer>();
        }
        // Print out the timestamps
        for(const auto& frame : inputFrames) {
            logger->debug("Starting input {} timestamp is {} ms",
                          frame.first,
                          static_cast<float>(frame.second->getTimestampDevice().time_since_epoch().count()) / 1000000.f);
        }
        auto tAfterMessageBeginning = steady_clock::now();
        int attempts = 0;
        while(true) {
            logger->trace("There have been {} attempts to sync", attempts);
            if(attempts > 50) {
                logger->warn("Sync node has been trying to sync for {} messages, but the messages are still not in sync.", attempts);
                for(const auto& frame : inputFrames) {
                    logger->warn("Output {} timestamp is {} ms",
                                 frame.first,
                                 static_cast<float>(frame.second->getTimestampDevice().time_since_epoch().count()) / 1000000.f);
                }
            }
            if(attempts > properties.syncAttempts && properties.syncAttempts != -1) {
                logger->warn(
                    "Sync node has been trying to sync for {} messages, but the messages are still not in sync. "
                    "The node will send the messages anyway.",
                    attempts);
                break;
            }
            // Find a minimum timestamp
            auto minTs = inputFrames.begin()->second->getTimestampDevice();
            for(const auto& frame : inputFrames) {
                if(frame.second->getTimestampDevice() < minTs) {
                    minTs = frame.second->getTimestampDevice();
                }
            }

            // Find a max timestamp
            auto maxTs = inputFrames.begin()->second->getTimestampDevice();
            for(const auto& frame : inputFrames) {
                if(frame.second->getTimestampDevice() > maxTs) {
                    maxTs = frame.second->getTimestampDevice();
                }
            }
            logger->debug("Diff: {} ms", duration_cast<milliseconds>(maxTs - minTs).count());
            // TODO(Morato) - don't commit
            if(duration_cast<nanoseconds>(maxTs - minTs).count() < syncThresholdNs) {
                break;
            }

            // Get the message with the minimum timestamp (oldest message)
            std::string minTsName;
            for(const auto& frame : inputFrames) {
                if(frame.second->getTimestampDevice() == minTs) {
                    minTsName = frame.first;
                    break;
                }
            }
            logger->trace("Receiving input: {}", minTsName);
            inputFrames[minTsName] = inputs[minTsName].get<dai::Buffer>();
            attempts++;
        }
        auto tBeforeSend = steady_clock::now();
        auto outputGroup = std::make_shared<dai::MessageGroup>();
        for(const auto& name : inputNames) {
            logger->trace("Sending output: {}", name);
            logger->trace("Timestamp: {} ms",
                          static_cast<float>(duration_cast<microseconds>(inputFrames[name]->getTimestampDevice().time_since_epoch()).count()) / 1000.f);
            outputGroup->add(name, inputFrames[name]);
        }
        out.send(outputGroup);
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
