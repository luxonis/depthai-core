#include "depthai/beta/node/MPPalmDetectionParser.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"
#include "beta/utilities/Detection/DetectionUtils.hpp"
#include "beta/utilities/MediaPipe/MediaPipeUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace node {

NNArchive MPPalmDetectionParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive MPPalmDetectionParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to MPPalmDetectionParser::build");
    return *nnArchive;
}

std::shared_ptr<MPPalmDetectionParser> MPPalmDetectionParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<MPPalmDetectionParser>(shared_from_this());
}

std::shared_ptr<MPPalmDetectionParser> MPPalmDetectionParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<MPPalmDetectionParser>(shared_from_this());
}

void MPPalmDetectionParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void MPPalmDetectionParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void MPPalmDetectionParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int palmDetectionHeads = 0;
    auto palmDetectionHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            palmDetectionHeads++;
            palmDetectionHead = head;
        }
    }

    DAI_CHECK_V(palmDetectionHeads > 0, "NNArchive does not contain an MPPalmDetectionParser head.");
    DAI_CHECK_V(palmDetectionHeads == 1, "NNArchive contains {} MPPalmDetectionParser heads. Please build with a specific head.", palmDetectionHeads);

    setConfig(palmDetectionHead);
}

void MPPalmDetectionParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an MPPalmDetectionParser head, got '{}'.", head.parser);

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

    const std::vector<std::string> headOutputs = head.outputs.value_or(std::vector<std::string>{});
    DAI_CHECK_V(headOutputs.size() == 2, "Only two output layers are supported for MPPalmDetectionParser, got {} layers.", headOutputs.size());
    setOutputLayerNames(headOutputs);

    // The scale key without a typed metadata field lives in the head metadata extra parameters.
    const auto& extraParams = head.metadata.extraParams;
    if(extraParams.is_object() && extraParams.contains("scale") && !extraParams.at("scale").is_null()) {
        const auto& scaleJson = extraParams.at("scale");
        DAI_CHECK(scaleJson.is_number_integer(), "Scale must be an integer.");
        setScale(scaleJson.get<int>());
    }
}

void MPPalmDetectionParser::setOutputLayerNames(const std::vector<std::string>& outputLayerNames) {
    DAI_CHECK_V(outputLayerNames.size() == 2, "Only two output layers are supported for MPPalmDetectionParser, got {} layers.", outputLayerNames.size());
    this->outputLayerNames = outputLayerNames;
}

std::vector<std::string> MPPalmDetectionParser::getOutputLayerNames() const {
    return outputLayerNames;
}

void MPPalmDetectionParser::setConfidenceThreshold(float threshold) {
    this->confidenceThreshold = threshold;
}

float MPPalmDetectionParser::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void MPPalmDetectionParser::setIouThreshold(float threshold) {
    this->iouThreshold = threshold;
}

float MPPalmDetectionParser::getIouThreshold() const {
    return iouThreshold;
}

void MPPalmDetectionParser::setMaxDetections(int maxDetections) {
    this->maxDetections = maxDetections;
}

int MPPalmDetectionParser::getMaxDetections() const {
    return maxDetections;
}

void MPPalmDetectionParser::setScale(int scale) {
    this->scale = scale;
}

int MPPalmDetectionParser::getScale() const {
    return scale;
}

void MPPalmDetectionParser::setLabelNames(const std::vector<std::string>& labelNames) {
    this->labelNames = labelNames;
}

std::vector<std::string> MPPalmDetectionParser::getLabelNames() const {
    return labelNames;
}

void MPPalmDetectionParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("MPPalmDetectionParser started");

    // The SSD anchors depend only on the scale; they are generated once and cached across
    // messages, keyed on the scale.
    bool anchorsCached = false;
    int cachedAnchorScale = 0;
    std::vector<std::array<double, 4>> anchors;

    while(mainLoop()) {
        auto nnData = input.get<dai::NNData>();
        if(!nnData) {
            continue;
        }

        // Extract: identify the bboxes and scores tensors by their last dimension; the tensor
        // with the larger last dimension holds the bboxes and the tensor with the smaller last
        // dimension holds the scores, mirroring the source parser's tie behavior over the layer
        // order.
        const auto layerNames = nnData->getAllLayerNames();
        struct IdentifiedTensor {
            std::vector<float> values;
            std::size_t lastDim = 0;
        };
        std::optional<IdentifiedTensor> bboxesTensor;
        std::optional<IdentifiedTensor> scoresTensor;
        for(const auto& layerName : layerNames) {
            auto tensor = utilities::ClassificationUtils::getShapedTensorData(*nnData, layerName);
            const std::size_t lastDim = tensor.dims.empty() ? 0 : tensor.dims.back();
            if(!bboxesTensor.has_value()) {
                bboxesTensor = IdentifiedTensor{tensor.values, lastDim};
                scoresTensor = IdentifiedTensor{std::move(tensor.values), lastDim};
            } else {
                if(lastDim >= bboxesTensor->lastDim) {
                    bboxesTensor = IdentifiedTensor{tensor.values, lastDim};
                }
                if(lastDim < scoresTensor->lastDim) {
                    scoresTensor = IdentifiedTensor{std::move(tensor.values), lastDim};
                }
            }
        }
        DAI_CHECK(bboxesTensor.has_value() && scoresTensor.has_value(), "No valid output tensors found.");
        // Reshape the bboxes tensor to (-1, 18): bounding box center/size plus 7 palm keypoint
        // coordinate pairs per anchor. The scores tensor is flattened.
        DAI_CHECK_V(bboxesTensor->values.size() % 18 == 0,
                    "MPPalmDetectionParser: cannot reshape the bboxes tensor of size {} into shape (-1, 18).",
                    bboxesTensor->values.size());

        // Compute
        const int currentScale = scale;
        if(!anchorsCached || cachedAnchorScale != currentScale) {
            anchors = utilities::MediaPipeUtils::generateHandtrackerAnchors(currentScale, currentScale);
            cachedAnchorScale = currentScale;
            anchorsCached = true;
        }
        const auto detections = utilities::MediaPipeUtils::computeMediaPipePalmDetections(
            bboxesTensor->values, scoresTensor->values, anchors, confidenceThreshold, iouThreshold, maxDetections, currentScale);

        // Emit. All detections carry label 0; the first label name (when configured) is mapped
        // to every detection.
        const std::vector<std::uint32_t> labels(detections.bboxes.size(), 0);
        std::vector<std::string> mappedLabelNames;
        if(!labelNames.empty()) {
            mappedLabelNames.assign(detections.bboxes.size(), labelNames.front());
        }
        auto message = utilities::DetectionUtils::createDetectionMessage(detections.bboxes, detections.scores, detections.angles, labels, mappedLabelNames);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("MPPalmDetectionParser created message with {} detections", message->detections.size());
        out.send(message);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
