#include "depthai/device/Device.hpp"

// std
#include <cstdio>
#include <iostream>
#include <memory>

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
#include "pipeline/datatype/Buffer.hpp"
#include "pipeline/datatype/StreamMessageParser.hpp"
#include "pipeline/node/ImageManip.hpp"
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

    // Stop recording / replaying
    for(auto& kv : recordStreams) {
        kv.second.running = false;
    }
    for(auto& kv : replayStreams) {
        kv.second.running = false;
    }

    // Close the device before clearing the queues
    DeviceBase::closeImpl();

    // Close and clear queues
    for(auto& kv : outputQueueMap) kv.second->close();
    for(auto& kv : inputQueueMap) kv.second->close();
    outputQueueMap.clear();
    inputQueueMap.clear();

    // Stop recording / replaying
    for(auto& kv : recordStreams) {
        if(kv.second.thread != nullptr) {
            kv.second.thread->join();
            kv.second.file.close();
            kv.second.fileMeta.close();
        }
    }
    for(auto& kv : replayStreams) {
        if(kv.second.thread != nullptr) {
            kv.second.thread->join();
            kv.second.file.close();
            kv.second.fileMeta.close();
        }
    }
    if(recordReplayState == RecordReplayState::RECORD) {
        std::vector<std::string> filenames = {platform::joinPaths(recordConfig.outputDir, mxId.append(("_record_config.json")))};
        std::vector<std::string> outFiles = {"record_config.json"};
        filenames.reserve(recordStreams.size() * 2 + 1);
        outFiles.reserve(recordStreams.size() * 2 + 1);
        for(auto& rstr : recordStreams) {
            filenames.push_back(rstr.second.path.string());
            filenames.push_back(rstr.second.path.string() + ".meta");
            outFiles.push_back(rstr.first);
            outFiles.push_back(rstr.first + ".meta");
        }
        getLogger().info("Record: Creating tar file with {} files", filenames.size());
        utility::tarFiles(platform::joinPaths(recordConfig.outputDir, "recording.tar.gz"), filenames, outFiles);
        getLogger().info("Record: Removing temporary files");
        for(auto& kv : recordStreams) {
            std::remove(kv.second.path.string().c_str());
            std::remove((kv.second.path.string() + ".meta").c_str());
        }
        std::remove(platform::joinPaths(recordConfig.outputDir, "record_config.json").c_str());
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

struct RecordMeta {
    size_t byteOffset;
    size_t size;
    int64_t seqNum;
    int64_t seconds;
    int64_t nanoseconds;
};

void recordStream(rr::RecordStream* recordStream, spdlog::logger* logger) {
    size_t byteOffset = 0;
    std::shared_ptr<ADatatype> msg = nullptr;
    try {
        while(recordStream->running) {
            msg = recordStream->queue->get();
            if(msg == nullptr) {
                // Queue closed
                logger->warn("Record stopped: queue closed");
                break;
            }
            auto msgBuf = std::dynamic_pointer_cast<Buffer>(msg);
            auto data = msgBuf->getRecordData();
            uint8_t* buf = data.first;
            size_t size = data.second;
            if(recordStream->compress) {
                auto compressed = utility::deflate(data.first, recordStream->compressionLevel);
                buf = compressed.data();
                size = compressed.size();
            }
            // TODO(asahtik): add header to unencoded ImgFrames (to be playable in VLC);
            recordStream->file.write((char*)buf, size);

            int64_t seqNum = msgBuf->getSequenceNum();
            int64_t seconds = std::chrono::duration_cast<std::chrono::seconds>(msgBuf->getTimestampDevice().time_since_epoch()).count();
            int64_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(msgBuf->getTimestampDevice().time_since_epoch()).count() % 1000000000;
            RecordMeta meta {byteOffset, size, seqNum, seconds, nanoseconds};
            // TODO(asahtik): Add meta such as width, height, sequence
            recordStream->fileMeta.write((char*)&meta, sizeof(RecordMeta));
            byteOffset += size;
            std::this_thread::yield();
            // TODO(asahtik): calculate necessary frequency
        }
    } catch(std::exception& e) {
        recordStream->running = false;
        logger->warn("Record stopped: {}", e.what());
    }
}

void replayStream(rr::ReplayStream* replayStream, spdlog::logger* logger) {
    std::shared_ptr<ADatatype> msg = nullptr;
    std::vector<uint8_t> buffer;
    try {
        std::chrono::steady_clock::time_point prevMsgTime = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point prevProcTime = std::chrono::steady_clock::now();
        bool first = true;
        while(replayStream->running) {
            RecordMeta meta {};
            replayStream->fileMeta.read((char*)&meta, sizeof(RecordMeta));

            if(replayStream->fileMeta.eof()) {
                // End of file
                logger->warn("Replay stopped: end of file");
                break;
            }
            if(replayStream->fileMeta.fail()) {
                // Error while reading
                logger->warn("Replay stopped: error while reading meta file");
                break;
            }
            buffer.resize(meta.size);
            replayStream->file.read((char*)buffer.data(), meta.size);

            auto decompressed = buffer;
            if(replayStream->decompress) {
                decompressed = utility::inflate(buffer.data(), meta.size);
            }
            RawBuffer rawBuffer;
            rawBuffer.data = decompressed;
            rawBuffer.tsDevice.sec = meta.seconds;
            rawBuffer.tsDevice.nsec = meta.nanoseconds;
            rawBuffer.sequenceNum = meta.seqNum;
            auto msg = replayStream->getMessageCallback(rawBuffer);

            auto msgTime = std::chrono::steady_clock::time_point(std::chrono::seconds(meta.seconds) + std::chrono::nanoseconds(meta.nanoseconds));
            if (first) {
                first = false;
                prevMsgTime = msgTime;
            } else {
                auto now = std::chrono::steady_clock::now();
                auto msgTimeDiff = msgTime - prevMsgTime;
                auto procTimeDiff = now - prevProcTime;
                auto diff = msgTimeDiff - procTimeDiff;
                if (diff > std::chrono::microseconds(0)) {
                    std::this_thread::sleep_for(diff);
                }
                prevMsgTime = msgTime;
            }
            prevProcTime = std::chrono::steady_clock::now();

            replayStream->queue->send(msg);

            std::this_thread::yield();
        }
    } catch(std::exception& e) {
        replayStream->running = false;
        logger->warn("Replay stopped: {}", e.what());
    }
}

bool allMatch(const std::vector<std::string>& v1, const std::vector<std::string>& v2) {
    for(const auto& el : v1) {
        if(std::find(v2.begin(), v2.end(), el) == v2.end()) return false;
    }
    return true;
}
std::string matchTo(const std::vector<std::string>& mxIds, const std::vector<std::string>& filenames, const std::vector<std::string>& nodenames) {
    std::string mxId = "";
    for(const auto& id : mxIds) {
        std::vector<std::string> matches;
        for(const auto& filename : filenames) {
            if(filename.size() >= 4 && filename.substr(filename.size() - 4, filename.size()) != "meta" && filename.find(id) != std::string::npos) {
                matches.push_back(filename.substr(id.size() + 1, filename.find_last_of('.') - id.size() - 1));
            }
        }
        if(matches.size() == nodenames.size()) {
            if(allMatch(matches, nodenames)) {
                if(mxId.empty()) {
                    mxId = id;
                } else {
                    throw std::runtime_error("Multiple recordings match the pipeline configuration - unsupported.");
                }
            }
        }
    }
    return mxId;
}

bool Device::startPipelineImpl(const Pipeline& pipeline) {
    // Set up record / replay
    std::string recordPath = utility::getEnv("DEPTHAI_RECORD");
    std::string replayPath = utility::getEnv("DEPTHAI_REPLAY");

    Pipeline pipelineCopy = pipeline;

    if(!recordPath.empty() && !replayPath.empty()) {
        getLogger().warn("Both DEPTHAI_RECORD and DEPTHAI_REPLAY are set. Record and replay disabled.");
    } else if(!recordPath.empty()) {
        if(checkRecordConfig(recordPath, recordConfig, getLogger())) {
            if(platform::checkWritePermissions(recordPath)) {
                recordReplayState = RecordReplayState::RECORD;
                auto sources = pipelineCopy.getSourceNodes();
                mxId = getMxId();
                try {
                    for(auto& node : sources) {
                        NodeRecordParams nodeParams = node->getNodeRecordParams();
                        std::string nodeName = nodeParams.name;
                        recordStreams[nodeName].path = Path(platform::joinPaths(recordPath, mxId.append("_").append(nodeName).append(".rec")));
                        recordStreams[nodeName].compress =
                            recordConfig.byteEncoding.enabled && recordConfig.byteEncoding.compressionLevel > 0 && !nodeParams.isVideo;
                        recordStreams[nodeName].compressionLevel = recordConfig.byteEncoding.enabled ? recordConfig.byteEncoding.compressionLevel : 0;
                        auto xout = pipelineCopy.create<dai::node::XLinkOut>();
                        xout->setStreamName(nodeName);
                        if(nodeParams.isVideo) {
                            if(recordConfig.videoEncoding.enabled) {
                                auto imageManip = pipelineCopy.create<dai::node::ImageManip>();
                                imageManip->initialConfig.setFrameType(ImgFrame::Type::NV12);
                                imageManip->setMaxOutputFrameSize(3110400);  // TODO(asahtik): set size depending on isp size
                                auto videnc = pipelineCopy.create<dai::node::VideoEncoder>();
                                videnc->setProfile(recordConfig.videoEncoding.profile);
                                videnc->setLossless(recordConfig.videoEncoding.lossless);
                                videnc->setBitrate(recordConfig.videoEncoding.bitrate);
                                videnc->setQuality(recordConfig.videoEncoding.quality);

                                node->getRecordOutput().link(imageManip->inputImage);
                                imageManip->out.link(videnc->input);
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
                    std::ofstream file(Path(platform::joinPaths(recordPath, mxId.append("_record_config.json"))));
                    json j = recordConfig;
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
                std::string rootPath = platform::getDirFromPath(replayPath);
                auto sources = pipelineCopy.getSourceNodes();
                mxId = getMxId();
                try {
                    auto tarFilenames = utility::filenamesInTar(replayPath);
                    std::remove_if(tarFilenames.begin(), tarFilenames.end(), [](const std::string& filename) { return filename.size() < 4 || filename.substr(filename.size() - 4, filename.size()) == "meta"; });
                    std::vector<std::string> nodeNames;
                    std::vector<std::string> pipelineFilenames;
                    pipelineFilenames.reserve(sources.size());
                    for(auto& node : sources) {
                        NodeRecordParams nodeParams = node->getNodeRecordParams();
                        std::string nodeName = mxId.append("_").append(nodeParams.name).append(".rec");
                        pipelineFilenames.push_back(nodeName);
                        nodeNames.push_back(nodeParams.name);
                    }
                    std::vector<std::string> inFiles;
                    std::vector<std::string> outFiles;
                    inFiles.reserve(sources.size() + 1);
                    outFiles.reserve(sources.size() + 1);
                    if(allMatch(tarFilenames, pipelineFilenames)) {
                        for(auto& nodename : nodeNames) {
                            auto filename = mxId.append("_").append(nodename).append(".rec");
                            inFiles.push_back(filename);
                            inFiles.push_back(filename + ".meta");
                            outFiles.push_back(platform::joinPaths(rootPath, filename));
                            outFiles.push_back(platform::joinPaths(rootPath, filename + ".meta"));
                        }
                        inFiles.emplace_back("record_config.json");
                        outFiles.push_back(platform::joinPaths(rootPath, mxId.append("_record_config.json")));
                        utility::untarFiles(replayPath, inFiles, outFiles);
                    } else {
                        std::vector<std::string> mxIds;
                        mxIds.reserve(tarFilenames.size());
                        for(auto& filename : tarFilenames) {
                            mxIds.push_back(filename.substr(0, filename.find_first_of('_')));
                        }
                        // Get unique mxIds
                        std::sort(mxIds.begin(), mxIds.end());
                        mxIds.erase(std::unique(mxIds.begin(), mxIds.end()), mxIds.end());
                        std::string mxIdRec = matchTo(mxIds, tarFilenames, nodeNames);
                        if (mxIdRec.empty()) {
                            throw std::runtime_error("No recordings match the pipeline configuration.");
                        }
                        for(auto& nodename : nodeNames) {
                            auto inFilename = mxIdRec.append("_").append(nodename).append(".rec");
                            auto outFilename = mxId.append("_").append(nodename).append(".rec");
                            inFiles.push_back(inFilename);
                            inFiles.push_back(inFilename + ".meta");
                            outFiles.push_back(platform::joinPaths(rootPath, outFilename));
                            outFiles.push_back(platform::joinPaths(rootPath, outFilename + ".meta"));
                        }
                        inFiles.emplace_back("record_config.json");
                        outFiles.push_back(platform::joinPaths(rootPath, mxId.append("_record_config.json")));
                        utility::untarFiles(replayPath, inFiles, outFiles);
                    }

                    // FIXME(asahtik): If this fails, extracted files do not get removed
                    std::ifstream file(recordPath);
                    json j = json::parse(file);
                    recordConfig = j.get<utility::RecordConfig>();

                    for (auto& node : sources) {
                        NodeRecordParams nodeParams = node->getNodeRecordParams();
                        std::string nodeName = nodeParams.name;
                        auto xin = pipelineCopy.create<dai::node::XLinkIn>();
                        xin->setStreamName(nodeName);
                        xin->out.link(node->getReplayInput());
                        replayStreams[nodeName].path = Path(platform::joinPaths(rootPath, mxId.append("_").append(nodeName).append(".rec")));
                        replayStreams[nodeName].decompress =
                            recordConfig.byteEncoding.enabled && recordConfig.byteEncoding.compressionLevel > 0 && !nodeParams.isVideo;
                        replayStreams[nodeName].getMessageCallback = node->getRecordedFrameCallback();
                    }
                } catch(const std::exception& e) {
                    recordReplayState = RecordReplayState::NONE;
                    getLogger().warn("Replay disabled: {}", e.what());
                }
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
                kv.second.fileMeta.open(kv.second.path.string() + ".meta", std::ios::out | std::ios::binary);
                if(!kv.second.file.is_open()) {
                    throw std::runtime_error(fmt::format("Failed to open file {}.", kv.second.path.string()));
                }
                if(!kv.second.fileMeta.is_open()) {
                    throw std::runtime_error(fmt::format("Failed to open file {}.", kv.second.path.string() + ".meta"));
                }
            }
        } catch(const std::exception& e) {
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
        bool replayEnabled = true;
        try {
            for(auto& kv : replayStreams) {
                kv.second.file.open(kv.second.path, std::ios::in | std::ios::binary);
                kv.second.fileMeta.open(kv.second.path.string() + ".meta", std::ios::in | std::ios::binary);
                if(!kv.second.file.is_open()) {
                    throw std::runtime_error(fmt::format("Failed to open file {}.", kv.second.path.string()));
                }
                if(!kv.second.fileMeta.is_open()) {
                    throw std::runtime_error(fmt::format("Failed to open file {}.", kv.second.path.string() + ".meta"));
                }
            }
        } catch(const std::exception& e) {
            getLogger().warn("Replay disabled: {}", e.what());
            replayEnabled = false;
        }
        if(replayEnabled) {
            for(auto& kv : replayStreams) {
                kv.second.queue = inputQueueMap[kv.first];
                kv.second.running = true;
                kv.second.thread = std::make_unique<std::thread>(replayStream, &kv.second, &getLogger());
            }
        }
    }

    return status;
}

}  // namespace dai
