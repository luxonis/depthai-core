#include "depthai/beta/node/HRNetParser.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"
#include "beta/utilities/Keypoints/KeypointsUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

namespace node {

HRNetParser::HRNetParser(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<BetaNode, HRNetParser, HRNetParserProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

HRNetParser::Properties& HRNetParser::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

NNArchive HRNetParser::createNNArchive(NNModelDescription& modelDesc) {
    // Download model from zoo
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

NNArchive HRNetParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to HRNetParser::build");
    return *nnArchive;
}

std::shared_ptr<HRNetParser> HRNetParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<HRNetParser>(shared_from_this());
}

std::shared_ptr<HRNetParser> HRNetParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<HRNetParser>(shared_from_this());
}

void HRNetParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void HRNetParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void HRNetParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int hrnetHeads = 0;
    auto hrnetHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            hrnetHeads++;
            hrnetHead = head;
        }
    }

    DAI_CHECK_V(hrnetHeads > 0, "NNArchive does not contain an HRNetParser head.");
    DAI_CHECK_V(hrnetHeads == 1, "NNArchive contains {} HRNetParser heads. Please build with a specific head.", hrnetHeads);

    setConfig(hrnetHead);
}

void HRNetParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an HRNetParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Only one output layer supported for HRNetParser, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());

    // The keypoint-specific keys without typed metadata fields live in the head metadata extra
    // parameters.
    const auto& extraParams = head.metadata.extraParams;

    if(const auto scoreThreshold = utilities::KeypointsUtils::readFloatParam(extraParams, "score_threshold")) {
        setScoreThreshold(*scoreThreshold);
    }
    if(const auto labelNames = utilities::KeypointsUtils::readKeypointLabelsParam(extraParams)) {
        setLabelNames(*labelNames);
    }
    // An empty edge list does not overwrite explicitly configured edges, mirroring the source
    // parser behavior.
    const auto edges = utilities::KeypointsUtils::readSkeletonEdgesParam(extraParams);
    if(!edges.empty()) {
        setEdges(edges);
    }
}

void HRNetParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string HRNetParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void HRNetParser::setScoreThreshold(float threshold) {
    initialConfig->setScoreThreshold(threshold);
}

float HRNetParser::getScoreThreshold() const {
    return initialConfig->getScoreThreshold();
}

void HRNetParser::setLabelNames(const std::vector<std::string>& labelNames) {
    properties.labelNames = labelNames;
}

std::vector<std::string> HRNetParser::getLabelNames() const {
    return properties.labelNames;
}

void HRNetParser::setEdges(const std::vector<Edge>& edges) {
    properties.edges = edges;
}

std::vector<Edge> HRNetParser::getEdges() const {
    return properties.edges;
}

void HRNetParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool HRNetParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void HRNetParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHostVar ? "host" : "device");
    auto config = getProperties().initialConfig;
    DAI_CHECK(config.validate(), "HRNetParser initial configuration is invalid.");
    const bool inputConfigSync = inputConfig.getWaitForMessage();

    // The resolved layer name persists across messages once auto-selected from a
    // single-tensor NNData, mirroring the source parser behavior.
    std::string resolvedOutputLayerName = properties.outputLayerName;

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            std::shared_ptr<HRNetParserConfig> candidate;
            if(inputConfigSync) {
                candidate = inputConfig.get<HRNetParserConfig>();
            } else {
                auto candidates = inputConfig.tryGetAll<HRNetParserConfig>();
                if(!candidates.empty()) {
                    candidate = candidates.back();
                }
            }
            if(candidate) {
                if(candidate->validate()) {
                    config = *candidate;
                } else {
                    logger->warn("HRNetParser received an invalid configuration; retaining the previous configuration.");
                }
            }

            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }
        auto tGotInput = std::chrono::steady_clock::now();
        const HRNetParserConfig configSnapshot = config;

        // Extract
        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(
                layerNames.size() == 1, "HRNetParser: expected 1 output layer, got {} layers. Please provide the output layer name.", layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        auto heatmaps = utilities::ClassificationUtils::getNchwTensorData(*nnData, resolvedOutputLayerName);

        // Compute. The number of keypoints and the heatmap size are derived from the tensor shape.
        auto hrnetKeypoints = utilities::KeypointsUtils::computeHrnetKeypoints(heatmaps.values, heatmaps.dims);

        // Emit. Keypoints with a score below the score threshold are dropped and the edges are
        // filtered and remapped to the kept keypoints.
        auto message = utilities::KeypointsUtils::createKeypointsMessage(
            hrnetKeypoints.coordinates, std::move(hrnetKeypoints.scores), configSnapshot.scoreThreshold, properties.labelNames, properties.edges);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("HRNetParser created message with {} keypoints", message->keypointsList.size());
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
