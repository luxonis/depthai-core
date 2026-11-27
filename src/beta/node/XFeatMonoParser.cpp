#include "depthai/beta/node/XFeatMonoParser.hpp"

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

XFeatMonoParser::XFeatMonoParser(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<BetaNode, XFeatMonoParser, XFeatMonoParserProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

XFeatMonoParser::Properties& XFeatMonoParser::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

NNArchive XFeatMonoParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive XFeatMonoParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to XFeatMonoParser::build");
    return *nnArchive;
}

std::shared_ptr<XFeatMonoParser> XFeatMonoParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<XFeatMonoParser>(shared_from_this());
}

std::shared_ptr<XFeatMonoParser> XFeatMonoParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<XFeatMonoParser>(shared_from_this());
}

void XFeatMonoParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void XFeatMonoParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void XFeatMonoParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
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

    DAI_CHECK_V(xfeatHeads > 0, "NNArchive does not contain an XFeatMonoParser head.");
    DAI_CHECK_V(xfeatHeads == 1, "NNArchive contains {} XFeatMonoParser heads. Please build with a specific head.", xfeatHeads);

    setConfig(xfeatHead);

    if(!utilities::XFeatUtils::readSizeParam(xfeatHead.metadata.extraParams, "input_size").has_value()) {
        const auto modelInputSize = utilities::XFeatUtils::getModelInputSize(configV1.model.inputs, NAME);
        setInputSize(modelInputSize.first, modelInputSize.second);
    }
}

void XFeatMonoParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an XFeatMonoParser head, got '{}'.", head.parser);

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

void XFeatMonoParser::setOutputLayerFeats(const std::string& outputLayerFeats) {
    properties.outputLayerFeats = outputLayerFeats;
}

std::string XFeatMonoParser::getOutputLayerFeats() const {
    return properties.outputLayerFeats;
}

void XFeatMonoParser::setOutputLayerKeypoints(const std::string& outputLayerKeypoints) {
    properties.outputLayerKeypoints = outputLayerKeypoints;
}

std::string XFeatMonoParser::getOutputLayerKeypoints() const {
    return properties.outputLayerKeypoints;
}

void XFeatMonoParser::setOutputLayerHeatmaps(const std::string& outputLayerHeatmaps) {
    properties.outputLayerHeatmaps = outputLayerHeatmaps;
}

std::string XFeatMonoParser::getOutputLayerHeatmaps() const {
    return properties.outputLayerHeatmaps;
}

void XFeatMonoParser::setOriginalSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Original image size must be greater than 0.");
    properties.originalSize = std::make_pair(width, height);
}

std::optional<std::pair<std::uint32_t, std::uint32_t>> XFeatMonoParser::getOriginalSize() const {
    return properties.originalSize;
}

void XFeatMonoParser::setInputSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Input image size must be greater than 0.");
    properties.inputSize = std::make_pair(width, height);
}

std::pair<std::uint32_t, std::uint32_t> XFeatMonoParser::getInputSize() const {
    return properties.inputSize;
}

void XFeatMonoParser::setMaxKeypoints(int maxKeypoints) {
    initialConfig->setMaxKeypoints(maxKeypoints);
}

int XFeatMonoParser::getMaxKeypoints() const {
    return initialConfig->getMaxKeypoints();
}

void XFeatMonoParser::setTrigger() {
    trigger = true;
}

void XFeatMonoParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool XFeatMonoParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void XFeatMonoParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHostVar ? "host" : "device");
    auto config = getProperties().initialConfig;
    DAI_CHECK(config.validate(), "XFeatMonoParser initial configuration is invalid.");
    const bool inputConfigSync = inputConfig.getWaitForMessage();

    DAI_CHECK(properties.originalSize.has_value(), "Original image size must be specified!");
    DAI_CHECK(!properties.outputLayerFeats.empty(), "Output layer containing features must be specified!");
    DAI_CHECK(!properties.outputLayerKeypoints.empty(), "Output layer containing keypoints must be specified!");
    DAI_CHECK(!properties.outputLayerHeatmaps.empty(), "Output layer containing heatmaps must be specified!");

    const double resizeRateW = static_cast<double>(properties.originalSize->first) / static_cast<double>(properties.inputSize.first);
    const double resizeRateH = static_cast<double>(properties.originalSize->second) / static_cast<double>(properties.inputSize.second);

    std::optional<utilities::XFeatUtils::XFeatResult> previousResult;

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            std::shared_ptr<XFeatMonoParserConfig> candidate;
            if(inputConfigSync) {
                candidate = inputConfig.get<XFeatMonoParserConfig>();
            } else {
                auto candidates = inputConfig.tryGetAll<XFeatMonoParserConfig>();
                if(!candidates.empty()) {
                    candidate = candidates.back();
                }
            }
            if(candidate) {
                if(candidate->validate()) {
                    config = *candidate;
                } else {
                    logger->warn("XFeatMonoParser received an invalid configuration; retaining the previous configuration.");
                }
            }

            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }
        auto tGotInput = std::chrono::steady_clock::now();
        const XFeatMonoParserConfig configSnapshot = config;

        auto tensors =
            utilities::XFeatUtils::extractXFeatTensors(*nnData, properties.outputLayerFeats, properties.outputLayerKeypoints, properties.outputLayerHeatmaps);

        const auto result = utilities::XFeatUtils::detectAndCompute(tensors.feats,
                                                                    tensors.keypoints,
                                                                    tensors.heatmaps,
                                                                    resizeRateW,
                                                                    resizeRateH,
                                                                    properties.inputSize.first,
                                                                    properties.inputSize.second,
                                                                    configSnapshot.maxKeypoints);

        if(!result.has_value()) {
            auto message = std::make_shared<dai::TrackedFeatures>();
            message->setBufferMetadataFrom(nnData);
            logger->debug("XFeatMonoParser: no keypoints found, sending empty TrackedFeatures message");
            auto tProcessed = std::chrono::steady_clock::now();
            {
                auto blockEvent = this->outputBlockEvent();
                out.send(message);
            }
            auto tAbsoluteEnd = std::chrono::steady_clock::now();
            this->logTiming(logger, tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
            continue;
        }

        std::shared_ptr<dai::TrackedFeatures> message;
        if(previousResult.has_value()) {
            const auto matchedPoints = utilities::XFeatUtils::matchResults(*previousResult, *result);
            message = utilities::XFeatUtils::createTrackedFeaturesMessage(matchedPoints);
        } else {
            message = std::make_shared<dai::TrackedFeatures>();
        }
        message->setBufferMetadataFrom(nnData);
        logger->debug("XFeatMonoParser created message with {} tracked features", message->trackedFeatures.size());
        auto tProcessed = std::chrono::steady_clock::now();
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }

        if(trigger.exchange(false)) {
            previousResult = *result;
        }
        auto tAbsoluteEnd = std::chrono::steady_clock::now();
        this->logTiming(logger, tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
