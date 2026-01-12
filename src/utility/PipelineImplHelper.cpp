#include "PipelineImplHelper.hpp"

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

void PipelineImplHelper::setupHolisticRecordAndReplay() {
    // TODO: Refactor this function to reduce complexity
    if(pipeline->buildingOnHost) {
        if(pipeline->defaultDevice) {
            auto recordPath = std::filesystem::path(utility::getEnvAs<std::string>("DEPTHAI_RECORD", ""));
            auto replayPath = std::filesystem::path(utility::getEnvAs<std::string>("DEPTHAI_REPLAY", ""));

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

                    pipeline->defaultDeviceId = pipeline->defaultDevice->getDeviceId();

                    if(!recordPath.empty() && !replayPath.empty()) {
                        Logging::getInstance().logger.warn("Both DEPTHAI_RECORD and DEPTHAI_REPLAY are set. Record and replay disabled.");
                    } else if(!recordPath.empty()) {
                        if(pipeline->enableHolisticRecordReplay || checkRecordConfig(recordPath, pipeline->recordConfig)) {
                            if(platform::checkWritePermissions(recordPath)) {
                                if(utility::setupHolisticRecord(pipeline->parent,
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
                            if(platform::checkWritePermissions(replayPath)) {
                                if(utility::setupHolisticReplay(pipeline->parent,
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
                                Logging::getInstance().logger.warn("DEPTHAI_REPLAY path does not have write permissions. Replay disabled.");
                            }
                        } else {
                            Logging::getInstance().logger.warn("DEPTHAI_REPLAY path does not exist or is invalid. Replay disabled.");
                        }
                    }
#else
                    recordConfig.state = RecordConfig::RecordReplayState::NONE;
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
void PipelineImplHelper::finishHolisticRecordAndReplay() {
    if(pipeline->buildingOnHost) {
        if(pipeline->recordConfig.state == RecordConfig::RecordReplayState::RECORD) {
            std::vector<std::filesystem::path> filenames = {pipeline->recordReplayFilenames["record_config"], pipeline->recordReplayFilenames["calibration"]};
            std::vector<std::string> outFiles = {"record_config.json", "calibration.json"};
            filenames.reserve(pipeline->recordReplayFilenames.size() * 2 + 1);
            outFiles.reserve(pipeline->recordReplayFilenames.size() * 2 + 1);
            for(auto& rstr : pipeline->recordReplayFilenames) {
                if(rstr.first != "record_config" && rstr.first != "calibration") {
                    std::string nodeName = rstr.first.substr(2);
                    std::filesystem::path filePath = rstr.second;
                    filenames.push_back(std::filesystem::path(filePath).concat(".mcap"));
                    outFiles.push_back(nodeName + ".mcap");
                    if(rstr.first[0] == 'v') {
                        filenames.push_back(std::filesystem::path(filePath).concat(pipeline->recordConfig.videoEncoding.enabled ? ".mp4" : ".avi"));
                        outFiles.push_back(nodeName + (pipeline->recordConfig.videoEncoding.enabled ? ".mp4" : ".avi"));
                    }
                }
            }
            Logging::getInstance().logger.info("Record: Creating tar file with {} files", filenames.size());
            try {
                auto now = std::chrono::system_clock::now();
                std::time_t t = std::chrono::system_clock::to_time_t(now);
                std::tm tm{};
#if defined(_WIN32)
                localtime_s(&tm, &t);
#else
                localtime_r(&t, &tm);
#endif
                std::ostringstream oss;
                oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

                utility::tarFiles(platform::joinPaths(pipeline->recordConfig.outputDir, "recording_" + oss.str() + ".tar"), filenames, outFiles);
            } catch(const std::exception& e) {
                Logging::getInstance().logger.error("Record: Failed to create tar file: {}", e.what());
            }
        }

        if(pipeline->removeRecordReplayFiles && pipeline->recordConfig.state != RecordConfig::RecordReplayState::NONE) {
            Logging::getInstance().logger.info("Record and Replay: Removing temporary files");
            for(auto& kv : pipeline->recordReplayFilenames) {
                if(kv.first != "record_config" && kv.first != "calibration") {
                    std::filesystem::remove(std::filesystem::path(kv.second).concat(".mcap"));
                    std::filesystem::remove(std::filesystem::path(kv.second).concat(pipeline->recordConfig.videoEncoding.enabled ? ".mp4" : ".avi"));
                } else {
                    std::filesystem::remove(kv.second);
                }
            }
        }
    }
}
void PipelineImplHelper::setupPipelineDebuggingPre() {
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
        hostEventAgg = pipeline->parent.create<node::internal::PipelineEventAggregation>();
        hostEventAgg->setRunOnHost(true);
        hostEventAgg->setTraceOutput(envPipelineDebugging);
        if(hasDeviceNodes) {
            deviceEventAgg = pipeline->parent.create<node::internal::PipelineEventAggregation>();
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
        auto stateMerge = pipeline->parent.create<node::PipelineStateMerge>()->build(hasDeviceNodes, true);
        std::shared_ptr<node::PipelineStateMerge> traceStateMerge;
        if(envPipelineDebugging) {
            traceStateMerge = pipeline->parent.create<node::PipelineStateMerge>()->build(hasDeviceNodes, true);
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
void PipelineImplHelper::setupPipelineDebuggingPost(std::unordered_map<dai::Node::Output*, node::internal::XLinkOutBridge>& bridgesOut,
                                                    std::unordered_map<dai::Node::Input*, node::internal::XLinkInBridge>& bridgesIn) {
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
