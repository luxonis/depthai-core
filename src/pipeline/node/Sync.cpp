#include "depthai/pipeline/node/Sync.hpp"

#include <chrono>
#include <thread>
#include <unordered_set>

#include "depthai/pipeline/Pipeline.hpp"
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

void Sync::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

ProcessorType Sync::getProcessor() const {
    return properties.processor;
}

void Sync::setTimestampSource(TimestampSource source) {
    properties.timestampSource = source;
}

Sync::TimestampSource Sync::getTimestampSource() const {
    return properties.timestampSource;
}

std::chrono::nanoseconds Sync::getSyncThreshold() const {
    return std::chrono::nanoseconds(properties.syncThresholdNs);
}

int Sync::getSyncAttempts() const {
    return properties.syncAttempts;
}

void Sync::buildStage1() {
    // Inputs spanning more than one device can only be synchronized with clocks that
    // are comparable across devices; per-device monotonic DEVICE timestamps are not
    std::unordered_set<const Device*> sources;
    for(const auto& input : inputs) {
        auto device = input.second.getSourceDevice();
        if(device != nullptr) {
            sources.insert(device.get());
        }
    }
    if(sources.size() > 1) {
        if(properties.timestampSource == TimestampSource::DEVICE) {
            throw std::runtime_error("Sync: TimestampSource::DEVICE cannot synchronize inputs from more than one device - use HOST or SYSTEM");
        }
        if(!runOnHost() && properties.timestampSource == TimestampSource::DEFAULT) {
            throw std::runtime_error(
                "Sync: inputs from more than one device with the default timestamp source require the node to run on the host (setRunOnHost(true)), or an "
                "explicit HOST/SYSTEM timestamp source");
        }
    }
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

Sync::TimestampSource Sync::getDefaultTimestampSource() const {
    return TimestampSource::HOST;
}

//////////////////////////////////////////////////////////////
//---------------------- HELPERS ---------------------------//
//////////////////////////////////////////////////////////////

class InvalidTimestampException : public std::runtime_error {
   public:
    explicit InvalidTimestampException(const std::string& message) : std::runtime_error(message) {}
};

double getTimestampMs(Sync::TimestampSource source, const dai::Buffer& buffer) {
    using namespace std::chrono;
    switch(source) {
        case Sync::TimestampSource::DEFAULT:
            throw std::runtime_error("Invalid timestamp source: DEFAULT. This should not happen.");
        case Sync::TimestampSource::DEVICE:
        case Sync::TimestampSource::HOST: {
            auto ts = source == Sync::TimestampSource::DEVICE ? buffer.getTimestampDevice() : buffer.getTimestamp();
            return static_cast<double>(duration_cast<nanoseconds>(ts.time_since_epoch()).count()) / 1e6;
        }
        case Sync::TimestampSource::SYSTEM: {
            auto tsOpt = buffer.getTimestampSystem();
            if(!tsOpt.has_value()) {
                throw InvalidTimestampException("Buffer does not have system timestamp, but Sync node is set to use system timestamps for synchronization.");
            }
            return static_cast<double>(duration_cast<nanoseconds>(tsOpt.value().time_since_epoch()).count()) / 1e6;
        }
    }
    return 0;
}

Buffer* getNewerBuffer(Sync::TimestampSource source, Buffer* a, Buffer* b) {
    if(a == nullptr) return b;
    if(b == nullptr) return a;

    switch(source) {
        case Sync::TimestampSource::DEFAULT:
            throw std::runtime_error("Invalid timestamp source: DEFAULT. This should not happen.");
        case Sync::TimestampSource::DEVICE:
        case Sync::TimestampSource::HOST: {
            auto tsA = source == Sync::TimestampSource::DEVICE ? a->getTimestampDevice() : a->getTimestamp();
            auto tsB = source == Sync::TimestampSource::DEVICE ? b->getTimestampDevice() : b->getTimestamp();
            return tsA > tsB ? a : b;
        }
        case Sync::TimestampSource::SYSTEM: {
            auto tsAOpt = a->getTimestampSystem();
            auto tsBOpt = b->getTimestampSystem();
            if(!tsAOpt.has_value() && !tsBOpt.has_value()) {
                return a;  // arbitrary
            }
            if(!tsAOpt.has_value()) return b;
            if(!tsBOpt.has_value()) return a;
            return tsAOpt.value() > tsBOpt.value() ? a : b;
        }
    }
    return a;
}

template <typename Clock>
class TimestampCompareGeneric {
   private:
    struct Pair {
        std::string name;
        std::chrono::time_point<Clock> timestamp;
    };
    Pair min = {"", std::chrono::time_point<Clock>::max()};
    Pair max = {"", std::chrono::time_point<Clock>::min()};

   public:
    void operator()(const std::string& name, const std::chrono::time_point<Clock>& timestamp) {
        if(timestamp < min.timestamp) {
            min = {name, timestamp};
        }
        if(timestamp > max.timestamp) {
            max = {name, timestamp};
        }
    }
    std::string getMinName() const {
        return min.name;
    }
    std::string getMaxName() const {
        return max.name;
    }
    std::chrono::time_point<Clock> getMinTimestamp() const {
        return min.timestamp;
    }
    std::chrono::time_point<Clock> getMaxTimestamp() const {
        return max.timestamp;
    }
    template <typename Duration>
    Duration getDifference() const {
        return std::chrono::duration_cast<Duration>(max.timestamp - min.timestamp);
    }
};

class TimestampCompare {
    Sync::TimestampSource source;

   public:
    std::variant<TimestampCompareGeneric<std::chrono::steady_clock>, TimestampCompareGeneric<std::chrono::system_clock>> impl;

    explicit TimestampCompare(Sync::TimestampSource source) : source(source) {
        switch(source) {
            case Sync::TimestampSource::DEFAULT:
                throw std::runtime_error("Invalid timestamp source: DEFAULT. This should not happen.");
            case Sync::TimestampSource::DEVICE:
            case Sync::TimestampSource::HOST:
                impl.emplace<TimestampCompareGeneric<std::chrono::steady_clock>>();
                break;
            case Sync::TimestampSource::SYSTEM:
                impl.emplace<TimestampCompareGeneric<std::chrono::system_clock>>();
                break;
        }
    }

    void operator()(const std::string& name, const dai::Buffer& buffer) {
        switch(source) {
            case Sync::TimestampSource::DEFAULT:
                throw std::runtime_error("Invalid timestamp source: DEFAULT. This should not happen.");
            case Sync::TimestampSource::DEVICE:
            case Sync::TimestampSource::HOST: {
                auto ts = source == Sync::TimestampSource::DEVICE ? buffer.getTimestampDevice() : buffer.getTimestamp();
                std::get<TimestampCompareGeneric<std::chrono::steady_clock>>(impl)(name, ts);
                break;
            }
            case Sync::TimestampSource::SYSTEM: {
                auto tsOpt = buffer.getTimestampSystem();
                if(!tsOpt.has_value()) {
                    throw InvalidTimestampException(
                        "Buffer does not have system timestamp, but Sync node is set to use system timestamps for synchronization.");
                }
                std::get<TimestampCompareGeneric<std::chrono::system_clock>>(impl)(name, tsOpt.value());
                break;
            }
        }
    }
    std::string getMinName() const {
        switch(source) {
            case Sync::TimestampSource::DEFAULT:
                throw std::runtime_error("Invalid timestamp source: DEFAULT. This should not happen.");
            case Sync::TimestampSource::DEVICE:
            case Sync::TimestampSource::HOST:
                return std::get<TimestampCompareGeneric<std::chrono::steady_clock>>(impl).getMinName();
            case Sync::TimestampSource::SYSTEM:
                return std::get<TimestampCompareGeneric<std::chrono::system_clock>>(impl).getMinName();
        }
        return "";
    }
    std::string getMaxName() const {
        switch(source) {
            case Sync::TimestampSource::DEFAULT:
                throw std::runtime_error("Invalid timestamp source: DEFAULT. This should not happen.");
            case Sync::TimestampSource::DEVICE:
            case Sync::TimestampSource::HOST:
                return std::get<TimestampCompareGeneric<std::chrono::steady_clock>>(impl).getMaxName();
            case Sync::TimestampSource::SYSTEM:
                return std::get<TimestampCompareGeneric<std::chrono::system_clock>>(impl).getMaxName();
        }
        return "";
    }
    template <typename Duration>
    Duration getDifference() const {
        switch(source) {
            case Sync::TimestampSource::DEFAULT:
                throw std::runtime_error("Invalid timestamp source: DEFAULT. This should not happen.");
            case Sync::TimestampSource::DEVICE:
            case Sync::TimestampSource::HOST:
                return std::get<TimestampCompareGeneric<std::chrono::steady_clock>>(impl).getDifference<Duration>();
            case Sync::TimestampSource::SYSTEM:
                return std::get<TimestampCompareGeneric<std::chrono::system_clock>>(impl).getDifference<Duration>();
        }
        return {};
    }
};

//////////////////////////////////////////////////////////////
//-------------------- HELPERS END -------------------------//
//////////////////////////////////////////////////////////////

void Sync::run() {
    using namespace std::chrono;
    auto& logger = pimpl->logger;
    const auto inputsName = inputs.name;

    auto timestampSource = properties.timestampSource;
    if(timestampSource == TimestampSource::DEFAULT) {
        timestampSource = getDefaultTimestampSource();
        logger->debug("Timestamp source set to DEFAULT, using {}", timestampSource == TimestampSource::DEVICE ? "DEVICE" : "HOST");
    }

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

    // Resolve which device produces each input (nullptr = host); while any of those
    // devices is not RUNNING the node drops instead of blocking on a dead stream.
    // The pipeline is resolved weakly per check - holding a strong reference for the
    // node's lifetime would let the pipeline destruct on this very thread.
    std::unordered_map<std::string, std::shared_ptr<Device>> sourceDevices;
    for(const auto& name : inputNames) {
        sourceDevices[name] = inputs[name].getSourceDevice();
    }
    auto anySourceNotRunning = [this, &sourceDevices]() {
        auto pipelineImpl = parent.lock();
        if(pipelineImpl == nullptr) {
            return false;
        }
        Pipeline pipeline(std::move(pipelineImpl));
        for(const auto& entry : sourceDevices) {
            if(entry.second != nullptr && pipeline.getDeviceState(entry.second) != DeviceState::RUNNING) {
                return true;
            }
        }
        return false;
    };
    // Receive one message without blocking indefinitely; nullptr means a source
    // device is degraded (or the node is stopping) and the current group is dropped
    auto receive = [this, &anySourceNotRunning](const std::string& name) -> std::shared_ptr<dai::Buffer> {
        auto& input = inputs[name];
        while(mainLoop()) {
            auto msg = input.tryGet<dai::Buffer>();
            if(msg != nullptr) return msg;
            if(anySourceNotRunning()) return nullptr;
            std::vector<std::reference_wrapper<MessageQueue>> queues{std::ref(static_cast<MessageQueue&>(input))};
            MessageQueue::waitAny(queues, std::chrono::milliseconds(100));
        }
        return nullptr;
    };

    time_point<steady_clock> tAfterMessageBeginning;

    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        std::unordered_map<std::string, std::shared_ptr<dai::Buffer>> inputFrames;
        bool dropped = false;
        {
            auto blockEvent = this->inputBlockEvent();

            // Drop (emit nothing) while any input's device is not running
            if(anySourceNotRunning()) {
                for(const auto& name : inputNames) {
                    inputs[name].tryGetAll();
                }
                std::this_thread::sleep_for(milliseconds(100));
                continue;
            }

            for(auto name : inputNames) {
                logger->trace("Receiving input: {}", name);
                inputFrames[name] = receive(name);
                if(inputFrames[name] == nullptr) {
                    dropped = true;
                    break;
                }
            }
            if(dropped) {
                continue;
            }
            // Print out the timestamps
            for(const auto& frame : inputFrames) {
                logger->debug("Starting input {} timestamp is {} ms", frame.first, getTimestampMs(timestampSource, *frame.second));
            }
            tAfterMessageBeginning = steady_clock::now();
            int attempts = 0;
            while(true) {
                logger->trace("There have been {} attempts to sync", attempts);
                if(attempts > 50) {
                    logger->warn("Sync node has been trying to sync for {} messages, but the messages are still not in sync.", attempts);
                    for(const auto& frame : inputFrames) {
                        logger->warn("Output {} timestamp is {} ms", frame.first, getTimestampMs(timestampSource, *frame.second));
                    }
                    logger->warn("Current timestamp {} ms",
                                 static_cast<double>(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count()) / 1e6);
                }

                if(attempts > properties.syncAttempts && properties.syncAttempts != -1) {
                    if(properties.syncAttempts != 0)
                        logger->warn(
                            "Sync node has been trying to sync for {} messages, but the messages are still not in sync. "
                            "The node will send the messages anyway.",
                            attempts);
                    break;
                }

                TimestampCompare tsCompare(timestampSource);

                for(const auto& frame : inputFrames) {
                    tsCompare(frame.first, *frame.second);
                }

                logger->debug("Diff: {} ms", tsCompare.getDifference<milliseconds>().count());

                if(tsCompare.getDifference<nanoseconds>().count() < syncThresholdNs) {
                    break;
                }

                // Get the message with the minimum timestamp (oldest message)
                std::string minTsName = tsCompare.getMinName();
                logger->trace("Receiving input: {}", minTsName);
                inputFrames[minTsName] = receive(minTsName);
                if(inputFrames[minTsName] == nullptr) {
                    dropped = true;
                    break;
                }
                attempts++;
            }
        }
        if(dropped) {
            continue;
        }
        auto tBeforeSend = steady_clock::now();
        auto outputGroup = std::make_shared<dai::MessageGroup>();
        dai::Buffer* newestFrame = inputFrames.begin()->second.get();
        for(const auto& name : inputNames) {
            logger->trace("Sending output: {}", name);
            logger->trace("Timestamp: {} ms", getTimestampMs(timestampSource, *inputFrames[name]));
            outputGroup->add(name, inputFrames[name]);
            newestFrame = getNewerBuffer(timestampSource, newestFrame, inputFrames[name].get());
        }

        outputGroup->setBufferMetadataFrom(newestFrame);
        outputGroup->setTimestampSource(timestampSource);

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
