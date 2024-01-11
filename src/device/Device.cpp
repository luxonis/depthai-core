#include "depthai/device/Device.hpp"

// std
#include <iostream>

// shared
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "DeviceLogger.hpp"
#include "depthai/device/DeviceBootloader.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "pipeline/Pipeline.hpp"
#include "pipeline/datatype/StreamMessageParser.hpp"
#include "pipeline/node/VideoEncoder.hpp"
#include "utility/Compression.hpp"
#include "utility/Environment.hpp"
#include "utility/Initialization.hpp"
#include "utility/Platform.hpp"
#include "utility/RecordConfig.hpp"
#include "utility/Resources.hpp"

namespace dai {

// Common explicit instantiation, to remove the need to define in header
constexpr std::size_t Device::EVENT_QUEUE_MAXIMUM_SIZE;

Device::Device(const Pipeline& pipeline) : DeviceBase(pipeline.getDeviceConfig()) {
    tryStartPipeline(pipeline);
}

template <typename T, std::enable_if_t<std::is_same<T, bool>::value, bool>>
Device::Device(const Pipeline& pipeline, T usb2Mode) : DeviceBase(pipeline.getDeviceConfig(), usb2Mode) {
    tryStartPipeline(pipeline);
}
template Device::Device(const Pipeline&, bool);

Device::Device(const Pipeline& pipeline, UsbSpeed maxUsbSpeed) : DeviceBase(pipeline.getDeviceConfig(), maxUsbSpeed) {
    tryStartPipeline(pipeline);
}

Device::Device(const Pipeline& pipeline, const dai::Path& pathToCmd) : DeviceBase(pipeline.getDeviceConfig(), pathToCmd) {
    tryStartPipeline(pipeline);
}

Device::Device(const Pipeline& pipeline, const DeviceInfo& devInfo) : DeviceBase(pipeline.getDeviceConfig(), devInfo) {
    tryStartPipeline(pipeline);
}

Device::Device(const Pipeline& pipeline, const DeviceInfo& devInfo, const dai::Path& pathToCmd) : DeviceBase(pipeline.getDeviceConfig(), devInfo, pathToCmd) {
    tryStartPipeline(pipeline);
}

template <typename T, std::enable_if_t<std::is_same<T, bool>::value, bool>>
Device::Device(const Pipeline& pipeline, const DeviceInfo& devInfo, T usb2Mode) : DeviceBase(pipeline.getDeviceConfig(), devInfo, usb2Mode) {
    tryStartPipeline(pipeline);
}
template Device::Device(const Pipeline&, const DeviceInfo&, bool);

Device::Device(const Pipeline& pipeline, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed) : DeviceBase(pipeline.getDeviceConfig(), devInfo, maxUsbSpeed) {
    tryStartPipeline(pipeline);
}

Device::Device() : DeviceBase() {}

Device::~Device() {
    DeviceBase::close();
}

void Device::closeImpl() {
    // Remove callbacks to this from queues
    for(const auto& kv : callbackIdMap) {
        outputQueueMap[kv.first]->removeCallback(kv.second);
    }
    // Clear map
    callbackIdMap.clear();

    // Stop recording
    for(auto& kv : recordStreams) {
        kv.second.running = false;
    }

    // Close the device before clearing the queues
    DeviceBase::closeImpl();

    // Close and clear queues
    for(auto& kv : outputQueueMap) kv.second->close();
    for(auto& kv : inputQueueMap) kv.second->close();
    outputQueueMap.clear();
    inputQueueMap.clear();

    // Stop recording
    for(auto& kv : recordStreams) {
        if(kv.second.thread != nullptr) {
            kv.second.thread->join();
        }
    }
}

std::shared_ptr<DataOutputQueue> Device::getOutputQueue(const std::string& name) {
    // Throw if queue not created
    // all queues for xlink streams are created upfront
    if(outputQueueMap.count(name) == 0) {
        throw std::runtime_error(fmt::format("Queue for stream name '{}' doesn't exist", name));
    }
    // Return pointer to this DataQueue
    return outputQueueMap.at(name);
}

std::shared_ptr<DataOutputQueue> Device::getOutputQueue(const std::string& name, unsigned int maxSize, bool blocking) {
    // Throw if queue not created
    // all queues for xlink streams are created upfront
    if(outputQueueMap.count(name) == 0) {
        throw std::runtime_error(fmt::format("Queue for stream name '{}' doesn't exist", name));
    }

    // Modify max size and blocking
    outputQueueMap.at(name)->setMaxSize(maxSize);
    outputQueueMap.at(name)->setBlocking(blocking);

    // Return pointer to this DataQueue
    return outputQueueMap.at(name);
}

std::vector<std::string> Device::getOutputQueueNames() const {
    std::vector<std::string> names;
    names.reserve(outputQueueMap.size());
    for(const auto& kv : outputQueueMap) {
        names.push_back(kv.first);
    }
    return names;
}

std::shared_ptr<DataInputQueue> Device::getInputQueue(const std::string& name) {
    // Throw if queue not created
    // all queues for xlink streams are created upfront
    if(inputQueueMap.count(name) == 0) {
        throw std::runtime_error(fmt::format("Queue for stream name '{}' doesn't exist", name));
    }
    // Return pointer to this DataQueue
    return inputQueueMap.at(name);
}

std::shared_ptr<DataInputQueue> Device::getInputQueue(const std::string& name, unsigned int maxSize, bool blocking) {
    // Throw if queue not created
    // all queues for xlink streams are created upfront
    if(inputQueueMap.count(name) == 0) {
        throw std::runtime_error(fmt::format("Queue for stream name '{}' doesn't exist", name));
    }

    // Modify max size and blocking
    inputQueueMap.at(name)->setMaxSize(maxSize);
    inputQueueMap.at(name)->setBlocking(blocking);

    // Return pointer to this DataQueue
    return inputQueueMap.at(name);
}

std::vector<std::string> Device::getInputQueueNames() const {
    std::vector<std::string> names;
    names.reserve(inputQueueMap.size());
    for(const auto& kv : inputQueueMap) {
        names.push_back(kv.first);
    }
    return names;
}

// void Device::setCallback(const std::string& name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb) {
//     // creates a CallbackHandler if not yet created
//     if(callbackMap.count(name) == 0) {
//         throw std::runtime_error(fmt::format("Queue for stream name '{}' doesn't exist", name));
//     } else {
//         // already exists, replace the callback
//         callbackMap.at(name).setCallback(cb);
//     }
// }

std::vector<std::string> Device::getQueueEvents(const std::vector<std::string>& queueNames, std::size_t maxNumEvents, std::chrono::microseconds timeout) {
    // First check if specified queues names are actually opened
    auto availableQueueNames = getOutputQueueNames();
    for(const auto& outputQueue : queueNames) {
        bool found = false;
        for(const auto& availableQueueName : availableQueueNames) {
            if(outputQueue == availableQueueName) {
                found = true;
                break;
            }
        }
        if(!found) throw std::runtime_error(fmt::format("Queue with name '{}' doesn't exist", outputQueue));
    }

    // Blocking part
    // lock eventMtx
    std::unique_lock<std::mutex> lock(eventMtx);

    // Create temporary string which predicate will fill when it finds the event
    std::vector<std::string> eventsFromQueue;
    // wait until predicate
    auto predicate = [this, &queueNames, &eventsFromQueue, &maxNumEvents]() {
        for(auto it = eventQueue.begin(); it != eventQueue.end();) {
            bool wasRemoved = false;
            for(const auto& name : queueNames) {
                if(name == *it) {
                    // found one of the events we have specified to wait for
                    eventsFromQueue.push_back(name);
                    // remove element from queue
                    it = eventQueue.erase(it);
                    wasRemoved = true;
                    // return and acknowledge the wait prematurelly, if reached maxnumevents
                    if(eventsFromQueue.size() >= maxNumEvents) {
                        return true;
                    }
                    // breaks as other queue names won't be same as this one
                    break;
                }
            }
            // If element wasn't removed, move iterator forward, else it was already moved by erase call
            if(!wasRemoved) ++it;
        }
        // After search, if no events were found, return false
        if(eventsFromQueue.empty()) return false;
        // Otherwise acknowledge the wait and exit
        return true;
    };

    if(timeout < std::chrono::microseconds(0)) {
        // if timeout < 0, infinite wait time (no timeout)
        eventCv.wait(lock, predicate);
    } else {
        // otherwise respect timeout
        eventCv.wait_for(lock, timeout, predicate);
    }

    // eventFromQueue should now contain the event name
    return eventsFromQueue;
}

std::vector<std::string> Device::getQueueEvents(const std::initializer_list<std::string>& queueNames,
                                                std::size_t maxNumEvents,
                                                std::chrono::microseconds timeout) {
    return getQueueEvents(std::vector<std::string>(queueNames), maxNumEvents, timeout);
}

std::vector<std::string> Device::getQueueEvents(std::string queueName, std::size_t maxNumEvents, std::chrono::microseconds timeout) {
    return getQueueEvents(std::vector<std::string>{queueName}, maxNumEvents, timeout);
}

std::vector<std::string> Device::getQueueEvents(std::size_t maxNumEvents, std::chrono::microseconds timeout) {
    return getQueueEvents(getOutputQueueNames(), maxNumEvents, timeout);
}

std::string Device::getQueueEvent(const std::vector<std::string>& queueNames, std::chrono::microseconds timeout) {
    auto events = getQueueEvents(queueNames, 1, timeout);
    if(events.empty()) return "";
    return events[0];
}
std::string Device::getQueueEvent(const std::initializer_list<std::string>& queueNames, std::chrono::microseconds timeout) {
    return getQueueEvent(std::vector<std::string>{queueNames}, timeout);
}

std::string Device::getQueueEvent(std::string queueName, std::chrono::microseconds timeout) {
    return getQueueEvent(std::vector<std::string>{queueName}, timeout);
}

std::string Device::getQueueEvent(std::chrono::microseconds timeout) {
    return getQueueEvent(getOutputQueueNames(), timeout);
}

bool checkRecordConfig(std::string& recordPath, utility::RecordConfig& config, spdlog::logger& logger) {
    if(!platform::checkPathExists(recordPath)) {
        logger.warn("DEPTHAI_RECORD path does not exist or is invalid. Record disabled.");
        return false;
    }
    if(platform::checkPathExists(recordPath, true)) {
        // Is a directory
        config.outputDir = recordPath;
    } else {
        // Is a file
        std::string ext = recordPath.substr(recordPath.find_last_of('.') + 1);
        if(ext != "json") {
            logger.warn("DEPTHAI_RECORD path is not a directory or a json file. Record disabled.");
            return false;
        }
        try {
            std::ifstream file(recordPath);
            json j = json::parse(file);
            config = j.get<utility::RecordConfig>();

            if(platform::checkPathExists(config.outputDir, true)) {
                // Is a directory
                recordPath = config.outputDir;
            } else {
                logger.warn("DEPTHAI_RECORD outputDir is not a directory. Record disabled.");
                return false;
            }
        } catch(const std::exception& e) {
            logger.warn("Error while processing DEPTHAI_RECORD json file: {}. Record disabled.", e.what());
            return false;
        }
    }
    return true;
}

void recordStream(rr::RecordStream* recordStream, spdlog::logger* logger) {
    while(recordStream->running) {
        std::shared_ptr<ADatatype> data = nullptr;
        try {
            data = recordStream->queue->get();
        } catch(std::exception& e) {
            // Queue closed / error
            logger->warn("Record stopped: {}", e.what());
        }
        if(data == nullptr) {
            // Queue closed
            logger->warn("Record stopped: queue closed");
            break;
        }
        try {
            auto serialized = StreamMessageParser::serializeMessage(data);
            auto compressed = recordStream->compressionLevel > 0 ? utility::deflate(serialized, recordStream->compressionLevel) : serialized;
            auto size = compressed.size();
            recordStream->file.write((char*)&size, sizeof(compressed.size()));
            recordStream->file.write((char*)compressed.data(), compressed.size());
            // TODO(asahtik): optimize written bytes - only write metadata when changed, compression, ...
        } catch(std::exception& e) {
            recordStream->running = false;
            logger->warn("Record stopped: {}", e.what());
            break;
        }
        std::this_thread::yield();
        // TODO(asahtik): calculate necessary frequency
    }
    recordStream->file.close();
}

bool Device::startPipelineImpl(const Pipeline& pipeline) {
    // Set up record / replay
    std::string recordPath = utility::getEnv("DEPTHAI_RECORD");
    std::string replayPath = utility::getEnv("DEPTHAI_REPLAY");

    Pipeline pipelineCopy = pipeline;

    if(!recordPath.empty() && !replayPath.empty()) {
        getLogger().warn("Both DEPTHAI_RECORD and DEPTHAI_REPLAY are set. Record and replay disabled.");
    } else if(!recordPath.empty()) {
        utility::RecordConfig config;
        if(checkRecordConfig(recordPath, config, getLogger())) {
            if(platform::checkWritePermissions(recordPath)) {
                recordReplayState = RecordReplayState::RECORD;
                auto sources = pipelineCopy.getSourceNodes();
                std::string mxId = getMxId();
                try {
                    for(auto& node : sources) {
                        NodeRecordParams nodeParams = node->getNodeRecordParams();
                        std::string nodeName = nodeParams.name;
                        recordStreams[nodeName].path = Path(platform::joinPaths(recordPath, mxId.append("_").append(nodeName).append(".dai.rec")));
                        recordStreams[nodeName].compressionLevel = config.byteEncoding.enabled ? config.byteEncoding.compressionLevel : 0;
                        auto xout = pipelineCopy.create<dai::node::XLinkOut>();
                        xout->setStreamName(nodeName);
                        if(nodeParams.isVideo) {
                            if(config.videoEncoding.enabled) {
                                auto videnc = pipelineCopy.create<dai::node::VideoEncoder>();
                                videnc->setProfile(config.videoEncoding.profile);
                                videnc->setLossless(config.videoEncoding.lossless);
                                videnc->setBitrate(config.videoEncoding.bitrate);
                                videnc->setQuality(config.videoEncoding.quality);
                                node->getRecordOutput().link(videnc->input);
                                videnc->out.link(xout->input);
                            } else {
                                node->getRecordOutput().link(xout->input);
                            }
                        } else {
                            node->getRecordOutput().link(xout->input);
                        }
                    }
                } catch(const std::runtime_error& e) {
                    recordReplayState = RecordReplayState::NONE;
                    getLogger().warn("Record disabled: {}", e.what());
                }
                // Write config to output dir
                try {
                    std::ofstream file(Path(platform::joinPaths(recordPath, "record_config.json")));
                    json j = config;
                    file << j.dump(4);
                } catch(const std::exception& e) {
                    getLogger().warn("Error while writing DEPTHAI_RECORD json file: {}", e.what());
                    recordReplayState = RecordReplayState::NONE;
                }
            } else {
                getLogger().warn("DEPTHAI_RECORD path does not have write permissions. Record disabled.");
            }
        }
    } else if(!replayPath.empty()) {
        if(platform::checkPathExists(replayPath)) {
            if(platform::checkWritePermissions(replayPath)) {
                recordReplayState = RecordReplayState::REPLAY;
                // TODO(asahtik): replay
            } else {
                getLogger().warn("DEPTHAI_REPLAY path does not have write permissions. Replay disabled.");
            }
        } else {
            getLogger().warn("DEPTHAI_REPLAY path does not exist or is invalid. Replay disabled.");
        }
    }

    // Open queues upfront, let queues know about data sizes (input queues)
    // Go through Pipeline and check for 'XLinkIn' and 'XLinkOut' nodes
    // and create corresponding default queues for them
    for(const auto& kv : pipelineCopy.getNodeMap()) {
        const auto& node = kv.second;
        const auto& xlinkIn = std::dynamic_pointer_cast<const node::XLinkIn>(node);
        if(xlinkIn == nullptr) {
            continue;
        }

        // Create DataInputQueue's
        auto streamName = xlinkIn->getStreamName();
        if(inputQueueMap.count(streamName) != 0) throw std::invalid_argument(fmt::format("Streams have duplicate name '{}'", streamName));
        // set max data size, for more verbosity
        inputQueueMap[std::move(streamName)] = std::make_shared<DataInputQueue>(connection, xlinkIn->getStreamName(), 16, true, xlinkIn->getMaxDataSize());
    }
    for(const auto& kv : pipelineCopy.getNodeMap()) {
        const auto& node = kv.second;
        const auto& xlinkOut = std::dynamic_pointer_cast<const node::XLinkOut>(node);
        if(xlinkOut == nullptr) {
            continue;
        }

        // Create DataOutputQueue's
        auto streamName = xlinkOut->getStreamName();
        if(outputQueueMap.count(streamName) != 0) throw std::invalid_argument(fmt::format("Streams have duplicate name '{}'", streamName));
        outputQueueMap[streamName] = std::make_shared<DataOutputQueue>(connection, streamName);

        // Add callback for events
        callbackIdMap[std::move(streamName)] =
            outputQueueMap[xlinkOut->getStreamName()]->addCallback([this](std::string queueName, std::shared_ptr<ADatatype>) {
                {
                    // Lock first
                    std::unique_lock<std::mutex> lock(eventMtx);

                    // Check if size is equal or greater than EVENT_QUEUE_MAXIMUM_SIZE
                    if(eventQueue.size() >= EVENT_QUEUE_MAXIMUM_SIZE) {
                        auto numToRemove = eventQueue.size() - EVENT_QUEUE_MAXIMUM_SIZE + 1;
                        eventQueue.erase(eventQueue.begin(), eventQueue.begin() + numToRemove);
                    }

                    // Add to the end of event queue
                    eventQueue.push_back(std::move(queueName));
                }

                // notify the rest
                eventCv.notify_all();
            });
    }
    bool status = DeviceBase::startPipelineImpl(pipelineCopy);
    if(!status) return status;

    if(recordReplayState == RecordReplayState::RECORD) {
        bool recordEnabled = true;
        try {
            for(auto& kv : recordStreams) {
                kv.second.file.open(kv.second.path, std::ios::out | std::ios::binary);
                if(!kv.second.file.is_open()) {
                    throw std::runtime_error(fmt::format("Failed to open file {}.", kv.second.path.string()));
                }
            }
        } catch(std::exception& e) {
            getLogger().warn("Record disabled: {}", e.what());
            recordEnabled = false;
        }
        if(recordEnabled) {
            for(auto& kv : recordStreams) {
                kv.second.queue = outputQueueMap[kv.first];
                kv.second.running = true;
                kv.second.thread = std::make_unique<std::thread>(recordStream, &kv.second, &getLogger());
            }
        }
    } else if(recordReplayState == RecordReplayState::REPLAY) {
        // TODO(asahtik): replay
    }

    return status;
}

}  // namespace dai
