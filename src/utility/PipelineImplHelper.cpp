#include "PipelineImplHelper.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include "pipeline/node/internal/PipelineEventAggregation.hpp"
#include "pipeline/node/internal/PipelineStateMerge.hpp"
#include "pipeline/node/internal/XLinkIn.hpp"
#include "pipeline/node/internal/XLinkInHost.hpp"
#include "pipeline/node/internal/XLinkOut.hpp"
#include "pipeline/node/internal/XLinkOutHost.hpp"
#include "utility/Compression.hpp"
#include "utility/Environment.hpp"
#include "utility/HolisticRecordReplay.hpp"
#include "utility/Platform.hpp"
#include "utility/RecordReplayImpl.hpp"

namespace dai {
namespace utility {

void PipelineImplHelper::setupHolisticRecordAndReplay(const std::weak_ptr<PipelineImpl>& pipelineWeak) {
    auto pipeline = pipelineWeak.lock();
    if(!pipeline) throw std::runtime_error("PipelineImplHelper: Pipeline is no longer available.");

    // TODO: Refactor this function to reduce complexity
    if(pipeline->buildingOnHost) {
        if(pipeline->defaultDevice) {
            auto recordPath = std::filesystem::path(utility::getEnvAs<std::string>("DEPTHAI_RECORD", ""));
            auto replayPath = std::filesystem::path(utility::getEnvAs<std::string>("DEPTHAI_REPLAY", ""));

            // The whole record/replay setup below is keyed to the default device only.
            // Note: this check must stay outside the try block below, which downgrades
            // runtime errors to warnings.
            if((pipeline->enableHolisticRecordReplay || !recordPath.empty() || !replayPath.empty()) && pipeline->getAllAssignedDevices().size() > 1) {
                throw std::runtime_error("Holistic record/replay is not supported with multiple devices yet");
            }

            if(pipeline->defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_2
               || pipeline->defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_X
               || pipeline->defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_RVC4) {
                try {
#ifdef DEPTHAI_MERGED_TARGET
                    if(pipeline->enableHolisticRecordReplay) {
                        switch(pipeline->recordConfig.state) {
                            case RecordConfig::RecordReplayState::RECORD:
                                recordPath = pipeline->recordConfig.outputDir;
                                replayPath = "";
                                break;
                            case RecordConfig::RecordReplayState::REPLAY:
                                recordPath = "";
                                replayPath = pipeline->recordConfig.outputDir;
                                break;
                            case RecordConfig::RecordReplayState::NONE:
                                pipeline->enableHolisticRecordReplay = false;
                                break;
                        }
                    }

                    // The pipeline only needs the stable host-side device identifier here.
                    // Avoid a live RPC call during build(), which can fail if the device RPC
                    // client is not available for a reused/external Device instance.
                    pipeline->defaultDeviceId = pipeline->defaultDevice->getDeviceInfo().getDeviceId();

                    if(!recordPath.empty() && !replayPath.empty()) {
                        Logging::getInstance().logger.warn("Both DEPTHAI_RECORD and DEPTHAI_REPLAY are set. Record and replay disabled.");
                    } else if(!recordPath.empty()) {
                        if(pipeline->enableHolisticRecordReplay || checkRecordConfig(recordPath, pipeline->recordConfig)) {
                            if(platform::checkWritePermissions(recordPath)) {
                                if(utility::setupHolisticRecord(dai::Pipeline(pipeline),
                                                                pipeline->defaultDeviceId,
                                                                pipeline->recordConfig,
                                                                pipeline->recordReplayFilenames,
                                                                pipeline->defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_2
                                                                    || pipeline->defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_X)) {
                                    pipeline->recordConfig.state = RecordConfig::RecordReplayState::RECORD;
                                    Logging::getInstance().logger.info("Record enabled.");
                                } else {
                                    Logging::getInstance().logger.warn("Could not set up holistic record. Record and replay disabled.");
                                }
                            } else {
                                Logging::getInstance().logger.warn("DEPTHAI_RECORD path does not have write permissions. Record disabled.");
                            }
                        } else {
                            Logging::getInstance().logger.warn("Could not successfully parse DEPTHAI_RECORD. Record disabled.");
                        }
                    } else if(!replayPath.empty()) {
                        if(platform::checkPathExists(replayPath)) {
                            if(utility::setupHolisticReplay(dai::Pipeline(pipeline),
                                                            replayPath,
                                                            pipeline->defaultDeviceId,
                                                            pipeline->recordConfig,
                                                            pipeline->recordReplayFilenames,
                                                            pipeline->defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_2
                                                                || pipeline->defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_X)) {
                                pipeline->recordConfig.state = RecordConfig::RecordReplayState::REPLAY;
                                if(platform::checkPathExists(replayPath, true)) {
                                    pipeline->removeRecordReplayFiles = false;
                                }
                                Logging::getInstance().logger.info("Replay enabled.");
                            } else {
                                Logging::getInstance().logger.warn("Could not set up holistic replay. Record and replay disabled.");
                            }
                        } else {
                            Logging::getInstance().logger.warn("DEPTHAI_REPLAY path does not exist or is invalid. Replay disabled.");
                        }
                    }
#else
                    pipeline->recordConfig.state = RecordConfig::RecordReplayState::NONE;
                    if(!recordPath.empty() || !replayPath.empty()) {
                        Logging::getInstance().logger.warn("Merged target is required to use holistic record/replay.");
                    }
#endif
                } catch(std::runtime_error& e) {
                    Logging::getInstance().logger.warn("Could not set up record / replay: {}", e.what());
                }
            } else if(pipeline->enableHolisticRecordReplay || !recordPath.empty() || !replayPath.empty()) {
                throw std::runtime_error("Holistic record/replay is only supported on RVC2 and RVC4 devices.");
            }
        }
    }
}
bool PipelineImplHelper::packHolisticRecording(PipelineImpl* pipeline) {
    if(pipeline == nullptr || !pipeline->buildingOnHost) return false;
    if(pipeline->holisticRecordPacked) return true;
    if(pipeline->recordConfig.state != RecordConfig::RecordReplayState::RECORD) return false;
    if(pipeline->recordTransferFailed) {
        std::string recordingName = "recording";
        const auto found = pipeline->recordReplayFilenames.find("holistic_recording");
        if(found != pipeline->recordReplayFilenames.end()) recordingName = found->second.filename().string();
        Logging::getInstance().logger.error("Record download of {} failed: archive was not created", recordingName);
        return false;
    }

    std::vector<std::filesystem::path> filenames;
    std::vector<std::string> outFiles;
    const auto addIfExists = [&](const std::filesystem::path& path, const std::string& nameInArchive) {
        std::error_code error;
        if(!std::filesystem::is_regular_file(path, error)) return;
        filenames.push_back(path);
        outFiles.push_back(nameInArchive);
    };
    const auto addNamed = [&](const std::string& key, const std::string& nameInArchive) {
        const auto found = pipeline->recordReplayFilenames.find(key);
        if(found != pipeline->recordReplayFilenames.end()) addIfExists(found->second, nameInArchive);
    };

    const auto holistic = pipeline->recordReplayFilenames.find("holistic_recording");
    if(holistic != pipeline->recordReplayFilenames.end() && std::filesystem::is_directory(holistic->second)) {
        for(const auto& entry : std::filesystem::recursive_directory_iterator(holistic->second)) {
            if(!entry.is_regular_file()) continue;
            const auto filename = entry.path().filename().string();
            if(!filename.empty() && filename.front() == '.') continue;
            const auto relative = std::filesystem::relative(entry.path(), holistic->second).generic_string();
            addIfExists(entry.path(), relative);
        }
        addNamed("record_config", "record_config.json");
        addNamed("camera_info", "camera_info.json");
        for(const auto& entry : pipeline->recordReplayFilenames) {
            if(entry.first == "record_config" || entry.first == "camera_info" || entry.first == "holistic_recording") continue;
            if(entry.first.empty() || entry.first[0] != 'b') continue;
            const auto nodeName = entry.first.size() > 2 ? entry.first.substr(2) : entry.first;
            addIfExists(std::filesystem::path(entry.second).concat(".mcap"), nodeName + ".mcap");
        }
    } else {
        addNamed("record_config", "record_config.json");
        addNamed("camera_info", "camera_info.json");
        for(const auto& entry : pipeline->recordReplayFilenames) {
            if(entry.first == "record_config" || entry.first == "camera_info" || entry.first == "holistic_recording") continue;
            if(entry.first.size() < 2) continue;
            const auto nodeName = entry.first.substr(2);
            addIfExists(std::filesystem::path(entry.second).concat(".mcap"), nodeName + ".mcap");
            if(entry.first[0] == 'v') {
                const auto extension = pipeline->recordConfig.videoEncoding.enabled ? ".mp4" : ".avi";
                addIfExists(std::filesystem::path(entry.second).concat(extension), nodeName + extension);
            }
        }
    }

    Logging::getInstance().logger.info("Record: Creating tar.gz file with {} files", filenames.size());
    try {
        const auto now = std::chrono::system_clock::now();
        const std::time_t time = std::chrono::system_clock::to_time_t(now);
        std::tm local{};
#if defined(_WIN32)
        localtime_s(&local, &time);
#else
        localtime_r(&time, &local);
#endif
        std::ostringstream stamp;
        stamp << std::put_time(&local, "%Y-%m-%d_%H-%M-%S");
        utility::archiveFilesCompressed(platform::joinPaths(pipeline->recordConfig.outputDir, "recording_" + stamp.str() + ".tar.gz"), filenames, outFiles);
    } catch(const std::exception& ex) {
        Logging::getInstance().logger.error("Record: Failed to create tar file: {}", ex.what());
        return false;
    }

    pipeline->holisticRecordPacked = true;
    if(pipeline->removeRecordReplayFiles) {
        Logging::getInstance().logger.info("Record and Replay: Removing temporary files");
        for(const auto& entry : pipeline->recordReplayFilenames) {
            std::error_code error;
            if(entry.first == "holistic_recording") {
                std::filesystem::remove_all(entry.second, error);
            } else if(entry.first == "record_config" || entry.first == "camera_info") {
                std::filesystem::remove(entry.second, error);
            } else {
                std::filesystem::remove(std::filesystem::path(entry.second).concat(".mcap"), error);
                std::filesystem::remove(std::filesystem::path(entry.second).concat(pipeline->recordConfig.videoEncoding.enabled ? ".mp4" : ".avi"), error);
            }
        }
    }
    return true;
}

void PipelineImplHelper::finishHolisticRecordAndReplay(PipelineImpl* pipeline) {
    if(pipeline->buildingOnHost) {
        if(pipeline->recordConfig.state == RecordConfig::RecordReplayState::RECORD) {
            packHolisticRecording(pipeline);
        } else if(pipeline->removeRecordReplayFiles && pipeline->recordConfig.state != RecordConfig::RecordReplayState::NONE) {
            Logging::getInstance().logger.info("Record and Replay: Removing temporary files");
            for(const auto& entry : pipeline->recordReplayFilenames) {
                std::error_code error;
                if(entry.first == "record_config" || entry.first == "camera_info" || entry.first == "holistic_recording") {
                    if(entry.first == "holistic_recording") {
                        std::filesystem::remove_all(entry.second, error);
                    } else {
                        std::filesystem::remove(entry.second, error);
                    }
                } else {
                    std::filesystem::remove(std::filesystem::path(entry.second).concat(".mcap"), error);
                    std::filesystem::remove(std::filesystem::path(entry.second).concat(pipeline->recordConfig.videoEncoding.enabled ? ".mp4" : ".avi"), error);
                }
            }
        }
        pipeline->recordConfig.state = RecordConfig::RecordReplayState::NONE;
    }
}
void PipelineImplHelper::setupPipelineDebuggingPre(const std::weak_ptr<PipelineImpl>& pipelineWeak) {
    auto pipeline = pipelineWeak.lock();
    if(!pipeline) throw std::runtime_error("PipelineImplHelper: Pipeline is no longer available.");

    // Create pipeline event aggregator node and link
    bool envPipelineDebugging = utility::getEnvAs<bool>("DEPTHAI_PIPELINE_DEBUGGING", false);
    pipeline->enablePipelineDebugging = pipeline->enablePipelineDebugging || envPipelineDebugging;
    if(pipeline->buildingOnHost && pipeline->enablePipelineDebugging) {
        // Check if any nodes are on host or device
        bool hasDeviceNodes = false;
        for(const auto& node : pipeline->getAllNodes()) {
            if(std::string(node->getName()) == std::string("NodeGroup") || std::string(node->getName()) == std::string("DeviceNodeGroup")) continue;
            if(!node->runOnHost()) {
                hasDeviceNodes = true;
            }
        }
        std::shared_ptr<node::internal::PipelineEventAggregation> hostEventAgg = nullptr;
        std::shared_ptr<node::internal::PipelineEventAggregation> deviceEventAgg = nullptr;
        hostEventAgg = pipeline->create<node::internal::PipelineEventAggregation>(pipeline);
        hostEventAgg->setRunOnHost(true);
        hostEventAgg->setTraceOutput(envPipelineDebugging);
        if(hasDeviceNodes) {
            deviceEventAgg = pipeline->create<node::internal::PipelineEventAggregation>(pipeline);
            deviceEventAgg->setRunOnHost(false);
            deviceEventAgg->setTraceOutput(envPipelineDebugging);
        }
        for(auto& node : pipeline->getAllNodes()) {
            if(std::string(node->getName()) == std::string("NodeGroup") || std::string(node->getName()) == std::string("DeviceNodeGroup")) continue;

            auto threadedNode = std::dynamic_pointer_cast<ThreadedNode>(node);
            if(threadedNode) {
                if(node->runOnHost() && hostEventAgg && node->id != hostEventAgg->id) {
                    threadedNode->pipelineEventOutput.link(hostEventAgg->inputs[fmt::format("{} - {}", node->getName(), node->id)]);
                } else if(!node->runOnHost() && deviceEventAgg && node->id != deviceEventAgg->id) {
                    threadedNode->pipelineEventOutput.link(deviceEventAgg->inputs[fmt::format("{} - {}", node->getName(), node->id)]);
                }
            }
        }
        auto stateMerge = pipeline->create<node::PipelineStateMerge>(pipeline)->build(hasDeviceNodes, true);
        std::shared_ptr<node::PipelineStateMerge> traceStateMerge;
        if(envPipelineDebugging) {
            traceStateMerge = pipeline->create<node::PipelineStateMerge>(pipeline)->build(hasDeviceNodes, true);
            traceStateMerge->setAllowConfiguration(false);
        }
        if(deviceEventAgg) {
            deviceEventAgg->out.link(stateMerge->inputDevice);
            stateMerge->outRequest.link(deviceEventAgg->request);
            if(envPipelineDebugging) {
                deviceEventAgg->outTrace.link(traceStateMerge->inputDevice);
                traceStateMerge->outRequest.link(deviceEventAgg->request);
            }
        }
        if(hostEventAgg) {
            hostEventAgg->out.link(stateMerge->inputHost);
            stateMerge->outRequest.link(hostEventAgg->request);
            if(envPipelineDebugging) {
                hostEventAgg->outTrace.link(traceStateMerge->inputHost);
                traceStateMerge->outRequest.link(hostEventAgg->request);
            }
        }
        pipeline->pipelineStateOut = stateMerge->out.createOutputQueue(1, false);
        pipeline->pipelineStateRequest = stateMerge->request.createInputQueue();
        if(envPipelineDebugging) {
            pipeline->pipelineStateTraceOut = traceStateMerge->out.createOutputQueue(1, false);
            pipeline->pipelineStateTraceRequest = traceStateMerge->request.createInputQueue();
        }
    }
}
void PipelineImplHelper::setupPipelineDebuggingPost(const std::weak_ptr<PipelineImpl>& pipelineWeak,
                                                    std::unordered_map<dai::Node::Output*, node::internal::XLinkOutBridge>& bridgesOut,
                                                    std::unordered_map<dai::Node::Input*, node::internal::XLinkInBridge>& bridgesIn) {
    auto pipeline = pipelineWeak.lock();
    if(!pipeline) throw std::runtime_error("PipelineImplHelper: Pipeline is no longer available.");

    // Finish setting up pipeline debugging
    if(pipeline->buildingOnHost && pipeline->enablePipelineDebugging) {
        // Enable events on xlink bridges
        std::shared_ptr<node::internal::PipelineEventAggregation> pipelineEventAggHost = nullptr;
        std::shared_ptr<node::internal::PipelineEventAggregation> pipelineEventAggDevice = nullptr;
        for(const auto& node : pipeline->getAllNodes()) {
            if(strcmp(node->getName(), "PipelineEventAggregation") == 0) {
                if(node->runOnHost() && !pipelineEventAggHost) {
                    pipelineEventAggHost = std::dynamic_pointer_cast<node::internal::PipelineEventAggregation>(node);
                } else if(!node->runOnHost() && !pipelineEventAggDevice) {
                    pipelineEventAggDevice = std::dynamic_pointer_cast<node::internal::PipelineEventAggregation>(node);
                }
            }
            if(pipelineEventAggHost && pipelineEventAggDevice) {
                break;
            }
        }
        if(!(bridgesOut.empty() && bridgesIn.empty()) && (!pipelineEventAggHost || !pipelineEventAggDevice)) {
            throw std::runtime_error("PipelineEventAggregation nodes not found for pipeline debugging setup");
        }
        for(auto& bridge : bridgesOut) {
            auto& nodes = bridge.second;
            nodes.xLinkInHost->pipelineEventOutput.link(
                pipelineEventAggHost->inputs[fmt::format("{} - {}", nodes.xLinkInHost->getName(), nodes.xLinkInHost->id)]);
            nodes.xLinkOut->pipelineEventOutput.link(pipelineEventAggDevice->inputs[fmt::format("{} - {}", nodes.xLinkOut->getName(), nodes.xLinkOut->id)]);
        }
        for(auto& bridge : bridgesIn) {
            auto& nodes = bridge.second;
            nodes.xLinkIn->pipelineEventOutput.link(pipelineEventAggDevice->inputs[fmt::format("{} - {}", nodes.xLinkIn->getName(), nodes.xLinkIn->id)]);
            nodes.xLinkOutHost->pipelineEventOutput.link(
                pipelineEventAggHost->inputs[fmt::format("{} - {}", nodes.xLinkOutHost->getName(), nodes.xLinkOutHost->id)]);
        }
    }

    // Disable event sending if no PipelineEventAggregation node is present
    {
        auto allNodes = pipeline->getAllNodes();
        if(std::find_if(allNodes.begin(), allNodes.end(), [](const std::shared_ptr<Node>& n) { return strcmp(n->getName(), "PipelineEventAggregation") == 0; })
           == allNodes.end()) {
            for(auto& node : allNodes) node->pipelineEventDispatcher->sendEvents = false;
        }
    }
}

}  // namespace utility
}  // namespace dai
