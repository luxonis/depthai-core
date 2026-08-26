#include "depthai/beta/node/SuperAnimalParser.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
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

SuperAnimalParserProperties::~SuperAnimalParserProperties() = default;

namespace node {

SuperAnimalParser::SuperAnimalParser(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<BetaNode, SuperAnimalParser, SuperAnimalParserProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

SuperAnimalParser::Properties& SuperAnimalParser::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

NNArchive SuperAnimalParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive SuperAnimalParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to SuperAnimalParser::build");
    return *nnArchive;
}

std::shared_ptr<SuperAnimalParser> SuperAnimalParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<SuperAnimalParser>(shared_from_this());
}

std::shared_ptr<SuperAnimalParser> SuperAnimalParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<SuperAnimalParser>(shared_from_this());
}

void SuperAnimalParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void SuperAnimalParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void SuperAnimalParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int superAnimalHeads = 0;
    auto superAnimalHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            superAnimalHeads++;
            superAnimalHead = head;
        }
    }

    DAI_CHECK_V(superAnimalHeads > 0, "NNArchive does not contain a SuperAnimalParser head.");
    DAI_CHECK_V(superAnimalHeads == 1, "NNArchive contains {} SuperAnimalParser heads. Please build with a specific head.", superAnimalHeads);

    setConfig(superAnimalHead);
}

void SuperAnimalParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a SuperAnimalParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Only one output layer supported for SuperAnimalParser, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());

    // The keypoint-specific keys without typed metadata fields live in the head metadata extra
    // parameters.
    const auto& extraParams = head.metadata.extraParams;

    if(extraParams.is_object() && extraParams.contains("scale_factor") && !extraParams.at("scale_factor").is_null()) {
        const auto& scaleFactorJson = extraParams.at("scale_factor");
        DAI_CHECK(scaleFactorJson.is_number(), "scale_factor must be a number.");
        setScaleFactor(scaleFactorJson.get<float>());
    }
    if(head.metadata.nKeypoints.has_value()) {
        setNumKeypoints(*head.metadata.nKeypoints);
    }
    if(extraParams.is_object() && extraParams.contains("score_threshold") && !extraParams.at("score_threshold").is_null()) {
        const auto& scoreThresholdJson = extraParams.at("score_threshold");
        DAI_CHECK(scoreThresholdJson.is_number(), "score_threshold must be a number.");
        setScoreThreshold(scoreThresholdJson.get<float>());
    }
    if(extraParams.is_object() && extraParams.contains("keypoint_labels") && !extraParams.at("keypoint_labels").is_null()) {
        const auto& keypointLabelsJson = extraParams.at("keypoint_labels");
        DAI_CHECK(keypointLabelsJson.is_array(), "keypoint_labels must be a list.");
        std::vector<std::string> headLabelNames;
        headLabelNames.reserve(keypointLabelsJson.size());
        for(const auto& labelJson : keypointLabelsJson) {
            DAI_CHECK(labelJson.is_string(), "All keypoint labels must be strings.");
            headLabelNames.push_back(labelJson.get<std::string>());
        }
        setLabelNames(headLabelNames);
    }
    if(extraParams.is_object() && extraParams.contains("skeleton_edges") && !extraParams.at("skeleton_edges").is_null()) {
        const auto& skeletonEdgesJson = extraParams.at("skeleton_edges");
        DAI_CHECK(skeletonEdgesJson.is_array(), "skeleton_edges must be a list.");
        std::vector<Edge> headEdges;
        headEdges.reserve(skeletonEdgesJson.size());
        for(const auto& edgeJson : skeletonEdgesJson) {
            DAI_CHECK(edgeJson.is_array() && edgeJson.size() == 2, "All skeleton edges must be pairs of keypoint indices.");
            DAI_CHECK(edgeJson.at(0).is_number_integer() && edgeJson.at(1).is_number_integer() && edgeJson.at(0).get<std::int64_t>() >= 0
                          && edgeJson.at(1).get<std::int64_t>() >= 0,
                      "All skeleton edge indices must be non-negative integers.");
            headEdges.push_back(Edge{edgeJson.at(0).get<std::uint32_t>(), edgeJson.at(1).get<std::uint32_t>()});
        }
        // An empty edge list does not overwrite explicitly configured edges, mirroring the source
        // parser behavior.
        if(!headEdges.empty()) {
            setEdges(headEdges);
        }
    }
}

void SuperAnimalParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string SuperAnimalParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void SuperAnimalParser::setScaleFactor(float scaleFactor) {
    DAI_CHECK(scaleFactor > 0.0f, "Scale factor must be greater than 0.");
    properties.scaleFactor = scaleFactor;
}

float SuperAnimalParser::getScaleFactor() const {
    return properties.scaleFactor;
}

void SuperAnimalParser::setNumKeypoints(std::int64_t nKeypoints) {
    DAI_CHECK(nKeypoints > 0, "Number of keypoints must be greater than 0.");
    properties.nKeypoints = nKeypoints;
}

std::int64_t SuperAnimalParser::getNumKeypoints() const {
    return properties.nKeypoints;
}

void SuperAnimalParser::setScoreThreshold(float threshold) {
    initialConfig->setScoreThreshold(threshold);
}

float SuperAnimalParser::getScoreThreshold() const {
    return initialConfig->getScoreThreshold();
}

void SuperAnimalParser::setLabelNames(const std::vector<std::string>& labelNames) {
    properties.labelNames = labelNames;
}

std::vector<std::string> SuperAnimalParser::getLabelNames() const {
    return properties.labelNames;
}

void SuperAnimalParser::setEdges(const std::vector<Edge>& edges) {
    properties.edges = edges;
}

std::vector<Edge> SuperAnimalParser::getEdges() const {
    return properties.edges;
}

void SuperAnimalParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool SuperAnimalParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void SuperAnimalParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHostVar ? "host" : "device");
    auto config = getProperties().initialConfig;
    DAI_CHECK(config.validate(), "SuperAnimalParser initial configuration is invalid.");
    const bool inputConfigSync = inputConfig.getWaitForMessage();

    // Unlike KeypointParser, the number of keypoints is not required here: the decoding derives
    // it from the heatmap tensor's last dimension, mirroring the source parser behavior.

    // The resolved layer name persists across messages once auto-selected from a
    // single-tensor NNData, mirroring the source parser behavior.
    std::string resolvedOutputLayerName = properties.outputLayerName;

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            std::shared_ptr<SuperAnimalParserConfig> candidate;
            if(inputConfigSync) {
                candidate = inputConfig.get<SuperAnimalParserConfig>();
            } else {
                auto candidates = inputConfig.tryGetAll<SuperAnimalParserConfig>();
                if(!candidates.empty()) {
                    candidate = candidates.back();
                }
            }
            if(candidate) {
                if(candidate->validate()) {
                    config = *candidate;
                } else {
                    logger->warn("SuperAnimalParser received an invalid configuration; retaining the previous configuration.");
                }
            }

            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }
        auto tGotInput = std::chrono::steady_clock::now();
        const SuperAnimalParserConfig configSnapshot = config;

        // Extract
        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(
                layerNames.size() == 1, "SuperAnimalParser: expected 1 output layer, got {} layers. Please provide the output layer name.", layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        // The tensor is read in its stored order, matching the source parser's
        // output.getTensor(name, dequantize=True) with the default storage order.
        auto heatmaps = utilities::ClassificationUtils::getShapedTensorData(*nnData, resolvedOutputLayerName);

        // Compute. The number of keypoints and the heatmap size are derived from the tensor
        // shape.
        auto superAnimalKeypoints = utilities::KeypointsUtils::computeSuperAnimalKeypoints(heatmaps.values, heatmaps.dims, properties.scaleFactor);

        // Emit. Keypoints with a score below the score threshold are dropped and the edges are
        // filtered and remapped to the kept keypoints.
        auto message = utilities::KeypointsUtils::createKeypointsMessage(
            superAnimalKeypoints.coordinates, std::move(superAnimalKeypoints.scores), configSnapshot.scoreThreshold, properties.labelNames, properties.edges);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("SuperAnimalParser created message with {} keypoints", message->keypointsList.size());
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
