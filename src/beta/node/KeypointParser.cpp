#include "depthai/beta/node/KeypointParser.hpp"

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

KeypointParserProperties::~KeypointParserProperties() = default;

namespace node {

NNArchive KeypointParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive KeypointParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to KeypointParser::build");
    return *nnArchive;
}

std::shared_ptr<KeypointParser> KeypointParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<KeypointParser>(shared_from_this());
}

std::shared_ptr<KeypointParser> KeypointParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<KeypointParser>(shared_from_this());
}

void KeypointParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void KeypointParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void KeypointParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int keypointHeads = 0;
    auto keypointHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            keypointHeads++;
            keypointHead = head;
        }
    }

    DAI_CHECK_V(keypointHeads > 0, "NNArchive does not contain a KeypointParser head.");
    DAI_CHECK_V(keypointHeads == 1, "NNArchive contains {} KeypointParser heads. Please build with a specific head.", keypointHeads);

    setConfig(keypointHead);
}

void KeypointParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a KeypointParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Only one output layer supported for KeypointParser, got {} layers.", numOutputs);
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

void KeypointParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string KeypointParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void KeypointParser::setScaleFactor(float scaleFactor) {
    DAI_CHECK(scaleFactor > 0.0f, "Scale factor must be greater than 0.");
    properties.scaleFactor = scaleFactor;
}

float KeypointParser::getScaleFactor() const {
    return properties.scaleFactor;
}

void KeypointParser::setNumKeypoints(std::int64_t nKeypoints) {
    DAI_CHECK(nKeypoints > 0, "Number of keypoints must be greater than 0.");
    properties.nKeypoints = nKeypoints;
}

std::optional<std::int64_t> KeypointParser::getNumKeypoints() const {
    return properties.nKeypoints;
}

void KeypointParser::setScoreThreshold(float threshold) {
    DAI_CHECK(threshold >= 0.0f && threshold <= 1.0f, "Confidence threshold must be between 0 and 1.");
    properties.scoreThreshold = threshold;
}

std::optional<float> KeypointParser::getScoreThreshold() const {
    return properties.scoreThreshold;
}

void KeypointParser::setLabelNames(const std::vector<std::string>& labelNames) {
    properties.labelNames = labelNames;
}

std::vector<std::string> KeypointParser::getLabelNames() const {
    return properties.labelNames;
}

void KeypointParser::setEdges(const std::vector<Edge>& edges) {
    properties.edges = edges;
}

std::vector<Edge> KeypointParser::getEdges() const {
    return properties.edges;
}

void KeypointParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool KeypointParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void KeypointParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHostVar ? "host" : "device");

    DAI_CHECK(properties.nKeypoints.has_value(), "Number of keypoints must be specified!");

    // The resolved layer name persists across messages once auto-selected from a
    // single-tensor NNData, mirroring the source parser behavior.
    std::string resolvedOutputLayerName = properties.outputLayerName;

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }
        auto tGotInput = std::chrono::steady_clock::now();

        // Extract
        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(
                layerNames.size() == 1, "KeypointParser: expected 1 output layer, got {} layers. Please provide the output layer name.", layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        auto values = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, resolvedOutputLayerName);

        // Compute
        auto coordinates = utilities::KeypointsUtils::computeKeypoints(std::move(values), *properties.nKeypoints, properties.scaleFactor);

        // Emit. The source parser passes no scores and no confidence threshold to the message
        // creator, so every keypoint is kept with a confidence of -1 and the edges are only
        // filtered against out-of-range indices.
        auto message = utilities::KeypointsUtils::createKeypointsMessage(coordinates, std::nullopt, std::nullopt, properties.labelNames, properties.edges);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("KeypointParser created message with {} keypoints", message->keypointsList.size());
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
