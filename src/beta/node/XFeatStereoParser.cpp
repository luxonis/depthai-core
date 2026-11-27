#include "depthai/beta/node/XFeatStereoParser.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "beta/utilities/XFeat/XFeatUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

namespace node {

XFeatStereoParser::XFeatStereoParser(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<BetaNode, XFeatStereoParser, XFeatStereoParserProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

XFeatStereoParser::Properties& XFeatStereoParser::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

NNArchive XFeatStereoParser::createNNArchive(NNModelDescription& modelDesc) {
    if(modelDesc.platform.empty()) {
        auto device = getParentPipeline().getDefaultDevice();
        DAI_CHECK(device != nullptr, "Device is not set. Specify the platform in the NNModelDescription when building without a device.");
        modelDesc.platform = device->getPlatformAsString();
    }
    auto path = getModelFromZoo(modelDesc);
    auto modelType = model::readModelType(path);
    DAI_CHECK(modelType == model::ModelType::NNARCHIVE, "Model from zoo must be an NNArchive to use the build function");
    auto nnArchive = NNArchive(path);
    return nnArchive;
}

NNArchive XFeatStereoParser::decodeModel(const Model& model) {
    std::optional<NNArchive> nnArchive;

    if(const auto* s = std::get_if<std::string>(&model)) {
        NNModelDescription description;
        description.model = *s;
        nnArchive = createNNArchive(description);
    } else if(const auto* desc = std::get_if<NNModelDescription>(&model)) {
        NNModelDescription tmpDesc = *desc;
        nnArchive = createNNArchive(tmpDesc);
    } else if(const auto* archive = std::get_if<NNArchive>(&model)) {
        nnArchive = *archive;
    }

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to XFeatStereoParser::build");
    return *nnArchive;
}

std::shared_ptr<XFeatStereoParser> XFeatStereoParser::build(Node::Output& reference, Node::Output& target, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    reference.link(referenceInput);
    target.link(targetInput);
    return std::static_pointer_cast<XFeatStereoParser>(shared_from_this());
}

std::shared_ptr<XFeatStereoParser> XFeatStereoParser::build(Node::Output& reference, Node::Output& target, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    reference.link(referenceInput);
    target.link(targetInput);
    return std::static_pointer_cast<XFeatStereoParser>(shared_from_this());
}

void XFeatStereoParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void XFeatStereoParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void XFeatStereoParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int xfeatHeads = 0;
    auto xfeatHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            xfeatHeads++;
            xfeatHead = head;
        }
    }

    DAI_CHECK_V(xfeatHeads > 0, "NNArchive does not contain an XFeatStereoParser head.");
    DAI_CHECK_V(xfeatHeads == 1, "NNArchive contains {} XFeatStereoParser heads. Please build with a specific head.", xfeatHeads);

    setConfig(xfeatHead);

    if(!utilities::XFeatUtils::readSizeParam(xfeatHead.metadata.extraParams, "input_size").has_value()) {
        const auto modelInputSize = utilities::XFeatUtils::getModelInputSize(configV1.model.inputs, NAME);
        setInputSize(modelInputSize.first, modelInputSize.second);
    }
}

void XFeatStereoParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an XFeatStereoParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 3, "Only three output layers supported for XFeat, got {} layers.", numOutputs);
    for(const auto& layer : *head.outputs) {
        if(layer.find("feats") != std::string::npos) {
            setOutputLayerFeats(layer);
        } else if(layer.find("keypoints") != std::string::npos) {
            setOutputLayerKeypoints(layer);
        } else if(layer.find("heatmaps") != std::string::npos) {
            setOutputLayerHeatmaps(layer);
        }
    }

    const auto& extraParams = head.metadata.extraParams;

    if(const auto headOriginalSize = utilities::XFeatUtils::readSizeParam(extraParams, "original_size")) {
        setOriginalSize(headOriginalSize->first, headOriginalSize->second);
    }
    if(const auto headInputSize = utilities::XFeatUtils::readSizeParam(extraParams, "input_size")) {
        setInputSize(headInputSize->first, headInputSize->second);
    }
    if(const auto headMaxKeypoints = utilities::XFeatUtils::readMaxKeypointsParam(extraParams)) {
        setMaxKeypoints(*headMaxKeypoints);
    }
}

void XFeatStereoParser::setOutputLayerFeats(const std::string& outputLayerFeats) {
    properties.outputLayerFeats = outputLayerFeats;
}

std::string XFeatStereoParser::getOutputLayerFeats() const {
    return properties.outputLayerFeats;
}

void XFeatStereoParser::setOutputLayerKeypoints(const std::string& outputLayerKeypoints) {
    properties.outputLayerKeypoints = outputLayerKeypoints;
}

std::string XFeatStereoParser::getOutputLayerKeypoints() const {
    return properties.outputLayerKeypoints;
}

void XFeatStereoParser::setOutputLayerHeatmaps(const std::string& outputLayerHeatmaps) {
    properties.outputLayerHeatmaps = outputLayerHeatmaps;
}

std::string XFeatStereoParser::getOutputLayerHeatmaps() const {
    return properties.outputLayerHeatmaps;
}

void XFeatStereoParser::setOriginalSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Original image size must be greater than 0.");
    properties.originalSize = std::make_pair(width, height);
}

std::optional<std::pair<std::uint32_t, std::uint32_t>> XFeatStereoParser::getOriginalSize() const {
    return properties.originalSize;
}

void XFeatStereoParser::setInputSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Input image size must be greater than 0.");
    properties.inputSize = std::make_pair(width, height);
}

std::pair<std::uint32_t, std::uint32_t> XFeatStereoParser::getInputSize() const {
    return properties.inputSize;
}

void XFeatStereoParser::setMaxKeypoints(int maxKeypoints) {
    initialConfig->setMaxKeypoints(maxKeypoints);
}

int XFeatStereoParser::getMaxKeypoints() const {
    return initialConfig->getMaxKeypoints();
}

void XFeatStereoParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool XFeatStereoParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void XFeatStereoParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHostVar ? "host" : "device");
    auto config = getProperties().initialConfig;
    DAI_CHECK(config.validate(), "XFeatStereoParser initial configuration is invalid.");
    const bool inputConfigSync = inputConfig.getWaitForMessage();

    DAI_CHECK(properties.originalSize.has_value(), "Original image size must be specified!");
    DAI_CHECK(!properties.outputLayerFeats.empty(), "Output layer containing features must be specified!");
    DAI_CHECK(!properties.outputLayerKeypoints.empty(), "Output layer containing keypoints must be specified!");
    DAI_CHECK(!properties.outputLayerHeatmaps.empty(), "Output layer containing heatmaps must be specified!");

    const double resizeRateW = static_cast<double>(properties.originalSize->first) / static_cast<double>(properties.inputSize.first);
    const double resizeRateH = static_cast<double>(properties.originalSize->second) / static_cast<double>(properties.inputSize.second);

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        std::shared_ptr<dai::NNData> referenceOutput;  // Two sequential blocking reads without synchronization, like the source run()
        std::shared_ptr<dai::NNData> targetOutput;
        {
            auto blockEvent = this->inputBlockEvent();
            std::shared_ptr<XFeatStereoParserConfig> candidate;
            if(inputConfigSync) {
                candidate = inputConfig.get<XFeatStereoParserConfig>();
            } else {
                auto candidates = inputConfig.tryGetAll<XFeatStereoParserConfig>();
                if(!candidates.empty()) {
                    candidate = candidates.back();
                }
            }
            if(candidate) {
                if(candidate->validate()) {
                    config = *candidate;
                } else {
                    logger->warn("XFeatStereoParser received an invalid configuration; retaining the previous configuration.");
                }
            }

            referenceOutput = referenceInput.get<dai::NNData>();
            targetOutput = targetInput.get<dai::NNData>();
            if(!referenceOutput || !targetOutput) {
                continue;
            }
        }
        auto tGotInput = std::chrono::steady_clock::now();
        const XFeatStereoParserConfig configSnapshot = config;

        auto referenceTensors = utilities::XFeatUtils::extractXFeatTensors(
            *referenceOutput, properties.outputLayerFeats, properties.outputLayerKeypoints, properties.outputLayerHeatmaps);
        auto targetTensors = utilities::XFeatUtils::extractXFeatTensors(
            *targetOutput, properties.outputLayerFeats, properties.outputLayerKeypoints, properties.outputLayerHeatmaps);

        const auto referenceResult = utilities::XFeatUtils::detectAndCompute(referenceTensors.feats,
                                                                             referenceTensors.keypoints,
                                                                             referenceTensors.heatmaps,
                                                                             resizeRateW,
                                                                             resizeRateH,
                                                                             properties.inputSize.first,
                                                                             properties.inputSize.second,
                                                                             configSnapshot.maxKeypoints);
        const auto targetResult = utilities::XFeatUtils::detectAndCompute(targetTensors.feats,
                                                                          targetTensors.keypoints,
                                                                          targetTensors.heatmaps,
                                                                          resizeRateW,
                                                                          resizeRateH,
                                                                          properties.inputSize.first,
                                                                          properties.inputSize.second,
                                                                          configSnapshot.maxKeypoints);

        if(!referenceResult.has_value()) {
            auto message = std::make_shared<dai::TrackedFeatures>();
            message->setBufferMetadataFrom(referenceOutput);
            logger->debug("XFeatStereoParser: no reference keypoints found, sending empty TrackedFeatures message");
            auto tProcessed = std::chrono::steady_clock::now();
            {
                auto blockEvent = this->outputBlockEvent();
                out.send(message);
            }
            auto tAbsoluteEnd = std::chrono::steady_clock::now();
            this->logTiming(logger, tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
            continue;
        }
        if(!targetResult.has_value()) {
            auto message = std::make_shared<dai::TrackedFeatures>();
            message->setBufferMetadataFrom(targetOutput);
            message->setSequenceNum(referenceOutput->getSequenceNum());
            logger->debug("XFeatStereoParser: no target keypoints found, sending empty TrackedFeatures message");
            auto tProcessed = std::chrono::steady_clock::now();
            {
                auto blockEvent = this->outputBlockEvent();
                out.send(message);
            }
            auto tAbsoluteEnd = std::chrono::steady_clock::now();
            this->logTiming(logger, tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
            continue;
        }

        const auto matchedPoints = utilities::XFeatUtils::matchResults(*referenceResult, *targetResult);
        auto message = utilities::XFeatUtils::createTrackedFeaturesMessage(matchedPoints);
        message->setBufferMetadataFrom(targetOutput);
        message->setSequenceNum(referenceOutput->getSequenceNum());
        logger->debug("XFeatStereoParser created message with {} tracked features", message->trackedFeatures.size());
        auto tProcessed = std::chrono::steady_clock::now();
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }
        auto tAbsoluteEnd = std::chrono::steady_clock::now();
        this->logTiming(logger, tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
