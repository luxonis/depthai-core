#include "PipelineBuilder.hpp"

// libraries
#include <spdlog/fmt/fmt.h>
#include <stdexcept>
#include <string>

// project
#include "MultiDeviceCalibrationStore.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/utility/Memory.hpp"
#include "utilities/FsyncController.hpp"
#include "utilities/OSUtils.hpp"

// nodes
#include "IMU.hpp"
#include "node/AprilTag/AprilTag.hpp"
#include "node/Benchmark.hpp"
#ifdef DEPTHAI_HAVE_BETA
    #include "node/ClassificationParser.hpp"
    #include "node/ClassificationSequenceParser.hpp"
    #include "node/EmbeddingsParser.hpp"
    #include "node/FastSAMParser.hpp"
    #include "node/HRNetParser.hpp"
    #include "node/ImageOutputParser.hpp"
    #include "node/KeypointParser.hpp"
    #include "node/LaneDetectionParser.hpp"
    #include "node/MLSDParser.hpp"
    #include "node/MPPalmDetectionParser.hpp"
    #include "node/MapOutputParser.hpp"
    #include "node/PPTextDetectionParser.hpp"
    #include "node/RFDETRParser.hpp"
    #include "node/RegressionParser.hpp"
    #include "node/SCRFDParser.hpp"
    #include "node/SuperAnimalParser.hpp"
    #include "node/XFeatMonoParser.hpp"
    #include "node/XFeatStereoParser.hpp"
    #include "node/YuNetParser.hpp"
    #include "node/ToFStereoFusion/ToFStereoFusion.hpp"
#endif
#include "node/DetectionParserNode.hpp"
#include "node/FeatureTracker/FeatureTracker.hpp"
#include "node/Gate/Gate.hpp"
#include "node/ImageManip/ImageManip.hpp"
#include "node/ImgDetectionsFilter.hpp"
#include "node/MessageDemux/MessageDemux.hpp"
#include "node/NeuralNetwork/NeuralNetwork.hpp"
#include "node/ObjectTracker/ObjectTracker.hpp"
#include "node/PipelineEventAggregation.hpp"
#include "node/PointCloud/PointCloud.hpp"
#include "node/Rectification.hpp"
#include "node/SegmentationParser.hpp"
#include "node/Vpp/Vpp.hpp"

#include "node/GPUStereo/GPUStereo.hpp"
#include "node/NeuralDepth/NeuralDepth.hpp"
#include "node/Script/Script.hpp"
#include "node/SpatialLocationCalculator.hpp"
#include "node/StereoDepth/StereoDepth.hpp"
#include "node/Stitching/Stitching.hpp"
#include "node/Sync/Sync.hpp"
#include "node/SystemLogger.hpp"
#include "node/ToF/ToF.hpp"
#include "node/Align/Align.hpp"
#include "node/Camera/Camera.hpp"
#include "node/ImageAlign/ImageAlign.hpp"
#include "node/Warp/Warp.hpp"
#include "node/VideoEncoder.hpp"
#include "node/XLinkIn.hpp"
#include "node/XLinkOut.hpp"

#include "pipeline/ThreadedNodeImpl.hpp"
#include "utilities/ErrorMacros.hpp"

namespace dai {
namespace gate {

namespace {

void installMultiDeviceCalibration(const Pipeline& pipeline) {
    std::optional<CalibrationHandler> localCalibration;
    if(pipeline.isCalibrationDataAvailable()) {
        localCalibration = pipeline.getCalibrationData();
    }
    // Validates the graph and this device's local origin; a mismatch is a hard build error.
    MultiDeviceCalibrationStore::getInstance().set(pipeline.getMultiDeviceCalibration(), localCalibration, getSerialNumber());
}

void validateFsync1() {
    auto isExternalStrobeEnabled = FsyncController::getInstance().getExternalStrobeEnable();
    auto fsyncRole = FsyncController::getInstance().getExternalFrameSyncRole();
    auto fsyncOk = FsyncController::getInstance().isOk();

    if ((fsyncRole != ExternalFrameSyncRole::MASTER || !fsyncOk) &&
        isExternalStrobeEnabled)
    {
        throw std::runtime_error("External strobe requires the device to be set in MASTER mode.");
    }
}

void validateFsync2() {
    auto numFsyncConsumers = FsyncController::getInstance().getNumFsyncConsumers();
    auto isExternalFsyncExplicit = FsyncController::getInstance().isExternalFrameSyncRoleSetExplicitly();
    auto isExternalStrobeEnabled = FsyncController::getInstance().getExternalStrobeEnable();
    auto fsyncOk = FsyncController::getInstance().isOk();

    if ((numFsyncConsumers == 0 || !fsyncOk) && isExternalFsyncExplicit) {
        throw std::runtime_error("At least one camera with fsync enabled is needed when External Frame Sync role is forced.");
    }

    if ((numFsyncConsumers == 0 || !fsyncOk) && isExternalStrobeEnabled) {
        throw std::runtime_error("At least one camera with fsync enabled is needed when External strobe is enabled.");
    }
}
}

decltype(XLinkOut::NAME) constexpr XLinkOut::NAME;
decltype(NeuralNetwork::NAME) constexpr NeuralNetwork::NAME;
decltype(Vpp::NAME) constexpr Vpp::NAME;
decltype(PointCloud::NAME) constexpr PointCloud::NAME;
decltype(ImgDetectionsFilter::NAME) constexpr ImgDetectionsFilter::NAME;
#ifdef DEPTHAI_HAVE_BETA
decltype(ClassificationParser::NAME) constexpr ClassificationParser::NAME;
decltype(ClassificationSequenceParser::NAME) constexpr ClassificationSequenceParser::NAME;
decltype(EmbeddingsParser::NAME) constexpr EmbeddingsParser::NAME;
decltype(FastSAMParser::NAME) constexpr FastSAMParser::NAME;
decltype(HRNetParser::NAME) constexpr HRNetParser::NAME;
decltype(ImageOutputParser::NAME) constexpr ImageOutputParser::NAME;
decltype(KeypointParser::NAME) constexpr KeypointParser::NAME;
decltype(LaneDetectionParser::NAME) constexpr LaneDetectionParser::NAME;
decltype(MLSDParser::NAME) constexpr MLSDParser::NAME;
decltype(MPPalmDetectionParser::NAME) constexpr MPPalmDetectionParser::NAME;
decltype(MapOutputParser::NAME) constexpr MapOutputParser::NAME;
decltype(PPTextDetectionParser::NAME) constexpr PPTextDetectionParser::NAME;
decltype(RFDETRParser::NAME) constexpr RFDETRParser::NAME;
decltype(RegressionParser::NAME) constexpr RegressionParser::NAME;
decltype(SCRFDParser::NAME) constexpr SCRFDParser::NAME;
decltype(SuperAnimalParser::NAME) constexpr SuperAnimalParser::NAME;
decltype(XFeatMonoParser::NAME) constexpr XFeatMonoParser::NAME;
decltype(XFeatStereoParser::NAME) constexpr XFeatStereoParser::NAME;
decltype(YuNetParser::NAME) constexpr YuNetParser::NAME;
decltype(ToFStereoFusion::NAME) constexpr ToFStereoFusion::NAME;
#endif
decltype(XLinkIn::NAME) constexpr XLinkIn::NAME;
decltype(VideoEncoder::NAME) constexpr VideoEncoder::NAME;
decltype(StereoDepth::NAME) constexpr StereoDepth::NAME;
decltype(SystemLogger::NAME) constexpr SystemLogger::NAME;
decltype(Sync::NAME) constexpr Sync::NAME;
decltype(SpatialLocationCalculator::NAME) constexpr SpatialLocationCalculator::NAME;
decltype(ImageManip::NAME) constexpr ImageManip::NAME;
decltype(IMU::NAME) constexpr IMU::NAME;
decltype(BenchmarkOut::NAME) constexpr BenchmarkOut::NAME;
decltype(BenchmarkIn::NAME) constexpr BenchmarkIn::NAME;
decltype(Script::NAME) constexpr Script::NAME;
decltype(FeatureTracker::NAME) constexpr FeatureTracker::NAME;
decltype(ObjectTracker::NAME) constexpr ObjectTracker::NAME;
decltype(PipelineEventAggregation::NAME) constexpr PipelineEventAggregation::NAME;
decltype(Rectification::NAME) constexpr Rectification::NAME;
decltype(NeuralDepth::NAME) constexpr NeuralDepth::NAME;
decltype(GPUStereo::NAME) constexpr GPUStereo::NAME;
decltype(Gate::NAME) constexpr Gate::NAME;
decltype(Stitching::NAME) constexpr Stitching::NAME;

decltype(ToFBase::NAME) constexpr ToFBase::NAME;

decltype(MessageDemux::NAME) constexpr MessageDemux::NAME;

template <typename N>
inline std::shared_ptr<ThreadedNode> nodeHelper(Pipeline& pipeline, const dai::NodeObjInfo& nodeInfo) {
    if(nodeInfo.name == N::NAME) {
        // deserialize props
        auto props = std::make_unique<typename N::Properties>();
        if(props == nullptr) throw std::runtime_error(fmt::format("Couldn't create properties for node with name: '{}'", N::NAME));

        dai::utility::deserialize(nodeInfo.properties, static_cast<typename N::Properties&>(*props));
        auto node = std::make_shared<N>(std::move(props));
        node->id = nodeInfo.id;

        // Deserialize into properties afterwards
        // auto node = std::make_shared<N>(pipeline.impl()->shared_from_this(), nodeInfo.id, std::move(props));
        // dai::utility::deserialize(nodeInfo.properties, node->properties);

        // Create ios
        for(auto& ioInfoPair : nodeInfo.ioInfo) {
            auto ioInfo = ioInfoPair.second;
            std::string group = std::get<0>(ioInfoPair.first);
            std::string name = std::get<1>(ioInfoPair.first);
            if(group.empty() && name.empty()) {
                throw std::runtime_error(fmt::format("Empty group and name for node: '{}'", N::NAME));
            }
            if(group.empty()) {  // First create normal inputs and outputs (not part of a group)
                if(ioInfo.type == NodeIoInfo::Type::MReceiver || ioInfo.type == NodeIoInfo::Type::SReceiver) {
                    Node::Input* inputRef = node->getInputRef(name);
                    if(inputRef != nullptr) {
                        inputRef->setBlocking(ioInfo.blocking);
                        inputRef->setMaxSize(ioInfo.queueSize);
                        inputRef->setWaitForMessage(ioInfo.waitForMessage);
                    } else {
                        throw std::runtime_error(fmt::format("No input with name: '{}', group: '{}' found on node '{}'.", name, group, nodeInfo.name));
                    }
                } else {
                    Node::Output* outputRef = node->getOutputRef(name);
                    if(outputRef != nullptr) {
                        ;  // Nothing to do with the outputs
                    } else {
                        throw std::runtime_error(fmt::format("No output with name: '{}', group: '{}' found on node '{}'.", name, group, nodeInfo.name));
                    }
                }
            } else {  // Group inputs have to be created first (they don't exist at this point)
                if(ioInfo.type == NodeIoInfo::Type::SSender || ioInfo.type == NodeIoInfo::Type::MSender) {
                    Node::OutputMap* outGroup = node->getOutputMapRef(group);
                    if(outGroup != nullptr) {
                        outGroup->operator[](name);  // Only create the output node
                    } else {
                        throw std::runtime_error(fmt::format("No output group with name: '{}' found on node '{}'.", group, nodeInfo.name));
                    }
                } else {  // Input
                    Node::InputMap* inGroup = node->getInputMapRef(group);
                    if(inGroup != nullptr) {
                        inGroup->operator[](name).setBlocking(ioInfo.blocking);
                        inGroup->operator[](name).setMaxSize(ioInfo.queueSize);
                        inGroup->operator[](name).setWaitForMessage(ioInfo.waitForMessage);
                    } else {
                        throw std::runtime_error(fmt::format("No input group with name: '{}' found on node '{}'.", group, nodeInfo.name));
                    }
                }
            }
        }

        if(node == nullptr) throw std::runtime_error(fmt::format("Couldn't create node with name: '{}'", N::NAME));
        node->getAssetManager().setRootPath(fmt::format("/node/{}/", node->id));
        return node;
    }
    return nullptr;
}

PipelineBuilder::NodeHandler PipelineBuilder::getDefaultNodeHandler() {
    NodeHandler handler;
    handler[XLinkOut::NAME] = nodeHelper<XLinkOut>;
    handler[NeuralNetwork::NAME] = nodeHelper<NeuralNetwork>;
    handler[DetectionParser::NAME] = nodeHelper<DetectionParser>;
    handler[SegmentationParser::NAME] = nodeHelper<SegmentationParser>;
    handler[ImgDetectionsFilter::NAME] = nodeHelper<ImgDetectionsFilter>;
#ifdef DEPTHAI_HAVE_BETA
    handler[ClassificationParser::NAME] = nodeHelper<ClassificationParser>;
    handler[ClassificationSequenceParser::NAME] = nodeHelper<ClassificationSequenceParser>;
    handler[EmbeddingsParser::NAME] = nodeHelper<EmbeddingsParser>;
    handler[FastSAMParser::NAME] = nodeHelper<FastSAMParser>;
    handler[HRNetParser::NAME] = nodeHelper<HRNetParser>;
    handler[ImageOutputParser::NAME] = nodeHelper<ImageOutputParser>;
    handler[KeypointParser::NAME] = nodeHelper<KeypointParser>;
    handler[LaneDetectionParser::NAME] = nodeHelper<LaneDetectionParser>;
    handler[MLSDParser::NAME] = nodeHelper<MLSDParser>;
    handler[MPPalmDetectionParser::NAME] = nodeHelper<MPPalmDetectionParser>;
    handler[MapOutputParser::NAME] = nodeHelper<MapOutputParser>;
    handler[PPTextDetectionParser::NAME] = nodeHelper<PPTextDetectionParser>;
    handler[RFDETRParser::NAME] = nodeHelper<RFDETRParser>;
    handler[RegressionParser::NAME] = nodeHelper<RegressionParser>;
    handler[SCRFDParser::NAME] = nodeHelper<SCRFDParser>;
    handler[SuperAnimalParser::NAME] = nodeHelper<SuperAnimalParser>;
    handler[XFeatMonoParser::NAME] = nodeHelper<XFeatMonoParser>;
    handler[XFeatStereoParser::NAME] = nodeHelper<XFeatStereoParser>;
    handler[YuNetParser::NAME] = nodeHelper<YuNetParser>;
    handler[ToFStereoFusion::NAME] = nodeHelper<ToFStereoFusion>;
#endif
    handler[XLinkIn::NAME] = nodeHelper<XLinkIn>;
    handler[VideoEncoder::NAME] = nodeHelper<VideoEncoder>;
    handler[SpatialLocationCalculator::NAME] = nodeHelper<SpatialLocationCalculator>;
    handler[StereoDepth::NAME] = nodeHelper<StereoDepth>;
    handler[Vpp::NAME] = nodeHelper<Vpp>;
    handler[Gate::NAME] = nodeHelper<Gate>;
    handler[PointCloud::NAME] = nodeHelper<PointCloud>;
    handler[SystemLogger::NAME] = nodeHelper<SystemLogger>;
    handler[Camera::NAME] = nodeHelper<Camera>;
    handler[Stitching::NAME] = nodeHelper<Stitching>;
    handler[Sync::NAME] = nodeHelper<Sync>;
    handler[ImageManip::NAME] = nodeHelper<ImageManip>;
    handler[IMU::NAME] = nodeHelper<IMU>;
    handler[BenchmarkOut::NAME] = nodeHelper<BenchmarkOut>;
    handler[BenchmarkIn::NAME] = nodeHelper<BenchmarkIn>;
    handler[Script::NAME] = nodeHelper<Script>;
    handler[FeatureTracker::NAME] = nodeHelper<FeatureTracker>;
    handler[ObjectTracker::NAME] = nodeHelper<ObjectTracker>;
    handler[Rectification::NAME] = nodeHelper<Rectification>;
    handler[NeuralDepth::NAME] = nodeHelper<NeuralDepth>;
    handler[GPUStereo::NAME] = nodeHelper<GPUStereo>;

    handler[ToFBase::NAME] = nodeHelper<ToFBase>;
    handler[Warp::NAME] = nodeHelper<Warp>;
    handler[ImageAlign::NAME] = nodeHelper<ImageAlign>;
    handler[Align::NAME] = nodeHelper<Align>;
    handler[AprilTag::NAME] = nodeHelper<AprilTag>;
    handler[MessageDemux::NAME] = nodeHelper<MessageDemux>;
    handler[PipelineEventAggregation::NAME] = nodeHelper<PipelineEventAggregation>;

    return handler;
}

void PipelineBuilder::setNodeHandler(NodeHandler handler) {
    nodeHandler = std::move(handler);
}

void PipelineBuilder::build(const PipelineSchema& schema, Pipeline& pipeline) {
    // Set global properties
    pipeline.setGlobalProperties(schema.globalProperties);
    installMultiDeviceCalibration(pipeline);

    // Create all nodes
    for(const auto& kv : schema.nodes) {
        const auto& nodeInfo = kv.second;
        // Create the node
        if(nodeHandler.count(nodeInfo.name) > 0) {
            auto node = nodeHandler.at(nodeInfo.name)(pipeline, nodeInfo);
            // Create the logger
            std::string loggerName = fmt::format("{}({})", node->getName(), nodeInfo.id);
            node->pimpl->logger =
                std::make_shared<spdlog::async_logger>(loggerName, sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
            nodeLoggers[node->id] = node->pimpl->logger;
            if(nodeIdToLogLevel.count(node->id) > 0) {
                node->pimpl->logger->set_level(nodeIdToLogLevel[node->id]);
            } else {
                node->pimpl->logger->set_level(currentLogLevelLoggers);
            }
            pipeline.add(node);
        } else {
            std::string error = "Pipeline node with name: '" + nodeInfo.name + "' doesn't exist";
            throw std::runtime_error(error);
        }
    }

    // Connect all nodes
    for(const auto& conn : schema.connections) {
        auto outNode = pipeline.getNode(conn.node1Id);
        auto inNode = pipeline.getNode(conn.node2Id);

        auto output = outNode->getOutputRef(conn.node1OutputGroup, conn.node1Output);
        if(output == nullptr) {
            // TODO(themarpe) - error message
            // fmt::format("Node '{}' didn't initialize input '{}'"}};
            throw std::runtime_error(
                fmt::format("No output with name: '{}', group: '{}' found on node '{}'.", conn.node1Output, conn.node1OutputGroup, outNode->getName()));
        }

        auto input = inNode->getInputRef(conn.node2InputGroup, conn.node2Input);
        if(input == nullptr) {
            // TODO(themarpe) - error message
            // fmt::format("Node '{}' didn't initialize input '{}'"}};
            throw std::runtime_error(
                fmt::format("No input with name: '{}', group: '{}' found on node '{}'.", conn.node2Input, conn.node2InputGroup, inNode->getName()));
        }

        // Link
        output->link(*input);
    }
}

void PipelineBuilder::start(Pipeline& pipeline) {
    dai::CameraBoardSocket lastColorCamSocket = dai::CameraBoardSocket::RGB;
    int lastColorCamId = 0;
    // PipelineRequirements req;

    validateFsync1();
    // Build the pipeline
    pipeline.buildDevice();
    validateFsync2();

    // Afterwards, start Pipeline
    pipeline.start();
}

void PipelineBuilder::init() {
    spdlog::init_thread_pool(8192, 1);
    stdoutSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    callbackSink = std::make_shared<dai::gate::CallbackSink_mt>([this](dai::LogMessage msg) {
        if(logEventCallback) {
            logEventCallback(std::move(msg));
        }
    });

    stdoutSink->set_pattern(dai::LOG_DEFAULT_PATTERN);

    callbackSink->set_pattern(dai::LOG_DEFAULT_PATTERN);

    sinks = {stdoutSink, callbackSink};
    systemLogger = std::make_shared<spdlog::async_logger>("system", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
    systemLogger->set_level(currentLogLevelLoggers);
}

void PipelineBuilder::setLogEventCallback(std::function<void(dai::LogMessage)> cb) {
    logEventCallback = cb;
}

void PipelineBuilder::setLogLevelCallback(dai::LogLevel level) {
    currentLogLevelLoggers = daiLogLevelToSpdlogLevel(level);

    // Set the level for all node loggers
    for(auto& element : nodeLoggers) {
        auto nodeLogger = element.second;
        nodeLogger->set_level(currentLogLevelLoggers);
    }

    // Everything has the same level, remove node-specific levels
    nodeIdToLogLevel.clear();

    // Set the level for the system logger
    assert(systemLogger && "Logger has to exist, before setting its level.");
    systemLogger->set_level(currentLogLevelLoggers);
}

void PipelineBuilder::setNodeLogLevelCallback(int64_t nodeId, dai::LogLevel level) {
    nodeIdToLogLevel[nodeId] = daiLogLevelToSpdlogLevel(level);

    // Set the level for the node logger
    if(nodeLoggers.count(nodeId) > 0 && nodeLoggers[nodeId] != nullptr) {
        nodeLoggers[nodeId]->set_level(nodeIdToLogLevel[nodeId]);
    }
}

void PipelineBuilder::setLogLevel(dai::LogLevel level) {
    currentLogLevel = daiLogLevelToSpdlogLevel(level);
    assert(stdoutSink && "Sink has to exist, before setting its level.");
    stdoutSink->set_level(currentLogLevel);
}

dai::LogLevel PipelineBuilder::getLogLevel() {
    return spdlogLevelToDaiLogLevel(currentLogLevel);
}

dai::LogLevel PipelineBuilder::getLogLevelCallback() {
    return spdlogLevelToDaiLogLevel(currentLogLevelLoggers);
}

dai::LogLevel PipelineBuilder::getNodeLogLevelCallback(int64_t nodeId) {
    // Return global level if nodeId is not set
    if(nodeIdToLogLevel.count(nodeId) == 0) {
        return getLogLevelCallback();
    }

    // Return node level if set
    return spdlogLevelToDaiLogLevel(nodeIdToLogLevel[nodeId]);
}

std::shared_ptr<spdlog::logger> PipelineBuilder::getSystemLogger() {
    return systemLogger;
}

PipelineBuilder& PipelineBuilder::getInstance() {
    static PipelineBuilder instance;
    return instance;
}

PipelineBuilder::PipelineBuilder() {}

PipelineBuilder::~PipelineBuilder(){};

// // Extracts camera settings and assigns IDs

//     for(const auto& kv : pipeline.getNodeMap()) {

//         if(node->getName() == ColorCamera::NAME) {
//             auto color = std::static_pointer_cast<ColorCamera>(node);

//             int camId = static_cast<int>(color->getBoardSocket());
//             if(camId == static_cast<int>(dai::CameraBoardSocket::AUTO)) {
//                 camId = lastColorCamId++;
//             }
//             PipelineRequirements colorReq;
//             colorReq.colorCamera = true;
//             colorReq.colorConfig.fps = color->getFps();
//             colorReq.colorConfig.width = color->getResolutionWidth();
//             colorReq.colorConfig.height = color->getResolutionHeight();
//             colorReq.colorConfig.height = color->getResolutionHeight();

//             req.add(PipelineRequirements::)

//         } else if(node->getName() == MonoCamera::NAME) {

//         }
//     }
// }

}  // namespace gate
}  // namespace dai
