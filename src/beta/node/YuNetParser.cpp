#include "depthai/beta/node/YuNetParser.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"
#include "beta/utilities/Detection/DetectionUtils.hpp"
#include "beta/utilities/YuNet/YuNetUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

YuNetParserProperties::~YuNetParserProperties() = default;

namespace node {

YuNetParser::YuNetParser(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<BetaNode, YuNetParser, YuNetParserProperties>(std::move(props)),
      initialConfig(std::make_shared<YuNetParserConfig>(properties.initialConfig)) {}

YuNetParser::Properties& YuNetParser::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

namespace {

/**
 * Resolve an output layer name: a configured name must exist in the model output; an empty name
 * is auto-detected as the single layer name starting with the prefix, mirroring the source
 * extract().
 */
std::string resolveOutputLayer(const std::vector<std::string>& layerNames, const std::string& configuredName, const std::string& prefix) {
    if(!configuredName.empty()) {
        DAI_CHECK_V(
            std::find(layerNames.begin(), layerNames.end(), configuredName) != layerNames.end(), "Layer {} not found in the model output.", configuredName);
        return configuredName;
    }
    std::vector<std::string> candidates;
    for(const auto& layerName : layerNames) {
        if(layerName.rfind(prefix, 0) == 0) {
            candidates.push_back(layerName);
        }
    }
    DAI_CHECK_V(!candidates.empty(), "No {} layer candidates found in the model output.", prefix);
    DAI_CHECK_V(candidates.size() == 1, "Multiple {} layer candidates found in the model output.", prefix);
    return candidates.front();
}

}  // namespace

NNArchive YuNetParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive YuNetParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to YuNetParser::build");
    return *nnArchive;
}

std::shared_ptr<YuNetParser> YuNetParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<YuNetParser>(shared_from_this());
}

std::shared_ptr<YuNetParser> YuNetParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<YuNetParser>(shared_from_this());
}

void YuNetParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void YuNetParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void YuNetParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int yunetHeads = 0;
    auto yunetHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            yunetHeads++;
            yunetHead = head;
        }
    }

    DAI_CHECK_V(yunetHeads > 0, "NNArchive does not contain a YuNetParser head.");
    DAI_CHECK_V(yunetHeads == 1, "NNArchive contains {} YuNetParser heads. Please build with a specific head.", yunetHeads);

    setConfig(yunetHead);

    // The input size is derived from the model input's declared shape and layout, mirroring the
    // source build(); a head metadata input_size key is not consumed, like the source. A head
    // carries no model input metadata, so this is only possible on the full-archive path.
    DAI_CHECK_V(configV1.model.inputs.size() == 1, "Only one input supported for YuNetParser, got {} inputs.", configV1.model.inputs.size());
    const auto& modelInput = configV1.model.inputs.front();
    const std::string layout = modelInput.layout.value_or("");
    DAI_CHECK_V(layout == "NHWC" || layout == "NCHW", "Input layout {} not supported for input_size extraction.", layout.empty() ? "None" : layout);
    DAI_CHECK_V(modelInput.shape.size() == 4, "Expected a 4D input shape for layout {}, got {} dimensions.", layout, modelInput.shape.size());

    const std::int64_t width = layout == "NHWC" ? modelInput.shape[2] : modelInput.shape[3];
    const std::int64_t height = layout == "NHWC" ? modelInput.shape[1] : modelInput.shape[2];
    DAI_CHECK_V(width > 0 && width <= std::numeric_limits<std::uint32_t>::max() && height > 0 && height <= std::numeric_limits<std::uint32_t>::max(),
                "Invalid input size ({}, {}) declared by the model input.",
                width,
                height);
    setInputSize(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height));
}

void YuNetParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a YuNetParser head, got '{}'.", head.parser);

    if(head.metadata.confThreshold.has_value()) {
        setConfidenceThreshold(static_cast<float>(*head.metadata.confThreshold));
    }
    if(head.metadata.iouThreshold.has_value()) {
        setIouThreshold(static_cast<float>(*head.metadata.iouThreshold));
    }
    if(head.metadata.maxDet.has_value()) {
        setMaxDetections(static_cast<int>(*head.metadata.maxDet));
    }
    if(head.metadata.classes.has_value()) {
        setLabelNames(*head.metadata.classes);
    }

    // Route the head output layer names by substring, in the source order (a name containing
    // "loc" is the loc layer, else "conf" the conf layer, else "iou" the iou layer); a later
    // match overwrites an earlier one, mirroring the source build().
    const std::vector<std::string> headOutputs = head.outputs.value_or(std::vector<std::string>{});
    for(const auto& outputLayer : headOutputs) {
        if(outputLayer.find("loc") != std::string::npos) {
            setOutputLayerLoc(outputLayer);
        } else if(outputLayer.find("conf") != std::string::npos) {
            setOutputLayerConf(outputLayer);
        } else if(outputLayer.find("iou") != std::string::npos) {
            setOutputLayerIou(outputLayer);
        } else {
            DAI_CHECK_V(false, "Unexpected output layer {}. Only loc, conf, and iou output layers are supported.", outputLayer);
        }
    }
}

void YuNetParser::setOutputLayerLoc(const std::string& locOutputLayerName) {
    properties.locOutputLayerName = locOutputLayerName;
}

std::string YuNetParser::getOutputLayerLoc() const {
    return properties.locOutputLayerName;
}

void YuNetParser::setOutputLayerConf(const std::string& confOutputLayerName) {
    properties.confOutputLayerName = confOutputLayerName;
}

std::string YuNetParser::getOutputLayerConf() const {
    return properties.confOutputLayerName;
}

void YuNetParser::setOutputLayerIou(const std::string& iouOutputLayerName) {
    properties.iouOutputLayerName = iouOutputLayerName;
}

std::string YuNetParser::getOutputLayerIou() const {
    return properties.iouOutputLayerName;
}

void YuNetParser::setConfidenceThreshold(float threshold) {
    initialConfig->setConfidenceThreshold(threshold);
}

float YuNetParser::getConfidenceThreshold() const {
    return initialConfig->getConfidenceThreshold();
}

void YuNetParser::setIouThreshold(float threshold) {
    initialConfig->setIouThreshold(threshold);
}

float YuNetParser::getIouThreshold() const {
    return initialConfig->getIouThreshold();
}

void YuNetParser::setMaxDetections(int maxDetections) {
    initialConfig->setMaxDetections(maxDetections);
}

int YuNetParser::getMaxDetections() const {
    return initialConfig->getMaxDetections();
}

void YuNetParser::setInputSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Input size must be greater than 0.");
    properties.inputSize = std::make_pair(width, height);
}

std::optional<std::pair<std::uint32_t, std::uint32_t>> YuNetParser::getInputSize() const {
    return properties.inputSize;
}

void YuNetParser::setLabelNames(const std::vector<std::string>& labelNames) {
    properties.labelNames = labelNames;
}

std::vector<std::string> YuNetParser::getLabelNames() const {
    return properties.labelNames;
}

void YuNetParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool YuNetParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void YuNetParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("YuNetParser started");

    YuNetParserConfig activeConfig = getProperties().initialConfig;
    DAI_CHECK(activeConfig.validate(), "YuNetParser initial configuration is invalid.");
    const bool inputConfigSync = inputConfig.getWaitForMessage();

    // The layer names resolved from the first incoming NNData persist across messages,
    // mirroring the source parser behavior.
    std::string resolvedLocLayerName = properties.locOutputLayerName;
    std::string resolvedConfLayerName = properties.confOutputLayerName;
    std::string resolvedIouLayerName = properties.iouOutputLayerName;

    // The anchors depend only on the input size; they are cached across messages and refreshed
    // when the input size changes, mirroring the source parser's anchor cache.
    bool anchorsCached = false;
    std::pair<std::uint32_t, std::uint32_t> cachedInputSize{0, 0};
    std::vector<std::array<float, 4>> anchors;

    while(mainLoop()) {
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            std::shared_ptr<YuNetParserConfig> candidate;
            if(inputConfigSync) {
                candidate = inputConfig.get<YuNetParserConfig>();
            } else {
                auto candidates = inputConfig.tryGetAll<YuNetParserConfig>();
                if(!candidates.empty()) {
                    candidate = candidates.back();
                }
            }
            if(candidate) {
                if(candidate->validate()) {
                    activeConfig = *candidate;
                } else {
                    logger->warn("YuNetParser ignored an invalid runtime configuration.");
                }
            }

            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }

        const YuNetParserConfig config = activeConfig;

        // The input size the anchors are generated from and the coordinates are normalized by
        // must be configured; the source parser requires it as well (its decoding fails with a
        // None input size).
        DAI_CHECK(properties.inputSize.has_value(), "YuNetParser: the input size is not set. Configure the parser from an NNArchive or with setInputSize().");
        const auto currentInputSize = *properties.inputSize;

        // Extract: resolve the loc, conf and iou layer names (auto-detected by the "loc",
        // "conf" and "iou" name prefixes when not configured) and read the dequantized
        // tensors. The candidate count is derived from the iou tensor (1 value per anchor);
        // the loc and conf tensors are paired 14 and 2 values per candidate, which accepts
        // both batched (1, N, C) and unbatched (N, C) tensors.
        const auto layerNames = nnData->getAllLayerNames();
        resolvedLocLayerName = resolveOutputLayer(layerNames, resolvedLocLayerName, "loc");
        resolvedConfLayerName = resolveOutputLayer(layerNames, resolvedConfLayerName, "conf");
        resolvedIouLayerName = resolveOutputLayer(layerNames, resolvedIouLayerName, "iou");

        const auto locTensor = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, resolvedLocLayerName);
        const auto confTensor = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, resolvedConfLayerName);
        const auto iouTensor = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, resolvedIouLayerName);
        const std::size_t numCandidates = iouTensor.size() / utilities::YuNetUtils::IOU_VALUES_PER_ANCHOR;
        DAI_CHECK_V(locTensor.size() == numCandidates * utilities::YuNetUtils::LOC_VALUES_PER_ANCHOR,
                    "YuNetParser: cannot reshape the {} tensor of size {} into shape ({}, {}).",
                    resolvedLocLayerName,
                    locTensor.size(),
                    numCandidates,
                    utilities::YuNetUtils::LOC_VALUES_PER_ANCHOR);
        DAI_CHECK_V(confTensor.size() == numCandidates * utilities::YuNetUtils::CONF_VALUES_PER_ANCHOR,
                    "YuNetParser: cannot reshape the {} tensor of size {} into shape ({}, {}).",
                    resolvedConfLayerName,
                    confTensor.size(),
                    numCandidates,
                    utilities::YuNetUtils::CONF_VALUES_PER_ANCHOR);

        // Compute
        if(!anchorsCached || cachedInputSize != currentInputSize) {
            anchors = utilities::YuNetUtils::generateAnchors(currentInputSize.first, currentInputSize.second);
            cachedInputSize = currentInputSize;
            anchorsCached = true;
        }
        const auto detections = utilities::YuNetUtils::computeYuNetDetections(locTensor,
                                                                              confTensor,
                                                                              iouTensor,
                                                                              anchors,
                                                                              currentInputSize.first,
                                                                              currentInputSize.second,
                                                                              config.confidenceThreshold,
                                                                              config.iouThreshold,
                                                                              config.maxDetections);

        // Emit. All detections carry label 0; the first label name (when configured) is mapped
        // to every detection. Every detection carries 5 keypoints without confidence scores
        // (confidence -1), mirroring the source message creator.
        const std::vector<std::uint32_t> labels(detections.bboxes.size(), 0);
        std::vector<std::string> mappedLabelNames;
        if(!properties.labelNames.empty()) {
            mappedLabelNames.assign(detections.bboxes.size(), properties.labelNames.front());
        }
        std::vector<std::vector<Keypoint>> keypoints;
        keypoints.reserve(detections.keypoints.size());
        for(const auto& detectionKeypoints : detections.keypoints) {
            std::vector<Keypoint> detectionKeypointList;
            detectionKeypointList.reserve(detectionKeypoints.size());
            for(const auto& coordinates : detectionKeypoints) {
                Keypoint keypoint;
                keypoint.imageCoordinates = Point3f(coordinates[0], coordinates[1], 0.0f);
                keypoint.confidence = -1.0f;
                detectionKeypointList.push_back(std::move(keypoint));
            }
            keypoints.push_back(std::move(detectionKeypointList));
        }
        auto message =
            utilities::DetectionUtils::createDetectionMessage(detections.bboxes, detections.scores, /*angles=*/{}, labels, mappedLabelNames, keypoints);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("YuNetParser created message with {} detections", message->detections.size());
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
