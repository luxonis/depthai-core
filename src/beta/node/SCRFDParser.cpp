#include "depthai/beta/node/SCRFDParser.hpp"

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
#include "beta/utilities/SCRFD/SCRFDUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace node {

NNArchive SCRFDParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive SCRFDParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to SCRFDParser::build");
    return *nnArchive;
}

std::shared_ptr<SCRFDParser> SCRFDParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<SCRFDParser>(shared_from_this());
}

std::shared_ptr<SCRFDParser> SCRFDParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<SCRFDParser>(shared_from_this());
}

void SCRFDParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void SCRFDParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void SCRFDParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int scrfdHeads = 0;
    auto scrfdHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            scrfdHeads++;
            scrfdHead = head;
        }
    }

    DAI_CHECK_V(scrfdHeads > 0, "NNArchive does not contain an SCRFDParser head.");
    DAI_CHECK_V(scrfdHeads == 1, "NNArchive contains {} SCRFDParser heads. Please build with a specific head.", scrfdHeads);

    setConfig(scrfdHead);

    // The input size is derived from the model input's declared shape and layout. A head
    // carries no model input metadata, so this is only possible on the full-archive path; the
    // source parser leaves the input size at its default instead.
    DAI_CHECK_V(configV1.model.inputs.size() == 1, "Only one input supported for SCRFDParser, got {} inputs.", configV1.model.inputs.size());
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

void SCRFDParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an SCRFDParser head, got '{}'.", head.parser);

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

    // The head must carry one score, bbox and kps layer per stride; the layer kinds are
    // identified by substring, mirroring the source parser.
    const std::vector<std::string> headOutputs = head.outputs.value_or(std::vector<std::string>{});
    std::size_t scoreLayers = 0;
    std::size_t bboxLayers = 0;
    std::size_t kpsLayers = 0;
    for(const auto& layer : headOutputs) {
        scoreLayers += layer.find("score") != std::string::npos ? 1 : 0;
        bboxLayers += layer.find("bbox") != std::string::npos ? 1 : 0;
        kpsLayers += layer.find("kps") != std::string::npos ? 1 : 0;
    }
    DAI_CHECK_V(scoreLayers == bboxLayers && scoreLayers == kpsLayers,
                "Number of score, bbox, and kps layers should be equal, got {}, {}, and {} layers.",
                scoreLayers,
                bboxLayers,
                kpsLayers);
    setOutputLayerNames(headOutputs);

    // The SCRFD-specific keys without typed metadata fields live in the head metadata extra
    // parameters. Missing keys keep the already configured values, mirroring the source parser.
    const auto& extraParams = head.metadata.extraParams;
    if(extraParams.is_object() && extraParams.contains("feat_stride_fpn") && !extraParams.at("feat_stride_fpn").is_null()) {
        const auto& stridesJson = extraParams.at("feat_stride_fpn");
        DAI_CHECK(stridesJson.is_array(), "feat_stride_fpn must be a list.");
        std::vector<std::int64_t> headStrides;
        headStrides.reserve(stridesJson.size());
        for(const auto& strideJson : stridesJson) {
            DAI_CHECK(strideJson.is_number_integer(), "Feature stride must be a list of integers.");
            headStrides.push_back(strideJson.get<std::int64_t>());
        }
        setFeatStrideFPN(headStrides);
    }
    if(extraParams.is_object() && extraParams.contains("num_anchors") && !extraParams.at("num_anchors").is_null()) {
        const auto& numAnchorsJson = extraParams.at("num_anchors");
        DAI_CHECK(numAnchorsJson.is_number_integer(), "Number of anchors must be an integer.");
        setNumAnchors(numAnchorsJson.get<std::int64_t>());
    }
}

void SCRFDParser::setOutputLayerNames(const std::vector<std::string>& outputLayerNames) {
    this->outputLayerNames = outputLayerNames;
}

std::vector<std::string> SCRFDParser::getOutputLayerNames() const {
    return outputLayerNames;
}

void SCRFDParser::setConfidenceThreshold(float threshold) {
    this->confidenceThreshold = threshold;
}

float SCRFDParser::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void SCRFDParser::setIouThreshold(float threshold) {
    this->iouThreshold = threshold;
}

float SCRFDParser::getIouThreshold() const {
    return iouThreshold;
}

void SCRFDParser::setMaxDetections(int maxDetections) {
    this->maxDetections = maxDetections;
}

int SCRFDParser::getMaxDetections() const {
    return maxDetections;
}

void SCRFDParser::setInputSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Input size must be greater than 0.");
    this->inputSize = std::make_pair(width, height);
}

std::pair<std::uint32_t, std::uint32_t> SCRFDParser::getInputSize() const {
    return inputSize;
}

void SCRFDParser::setFeatStrideFPN(const std::vector<std::int64_t>& featStrideFpn) {
    for(const std::int64_t stride : featStrideFpn) {
        DAI_CHECK_V(stride > 0, "Feature stride must be positive, got {}.", stride);
    }
    this->featStrideFpn = featStrideFpn;
}

std::vector<std::int64_t> SCRFDParser::getFeatStrideFPN() const {
    return featStrideFpn;
}

void SCRFDParser::setNumAnchors(std::int64_t numAnchors) {
    this->numAnchors = numAnchors;
}

std::int64_t SCRFDParser::getNumAnchors() const {
    return numAnchors;
}

void SCRFDParser::setLabelNames(const std::vector<std::string>& labelNames) {
    this->labelNames = labelNames;
}

std::vector<std::string> SCRFDParser::getLabelNames() const {
    return labelNames;
}

void SCRFDParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("SCRFDParser started");

    // The layer name list resolved from the first incoming NNData persists across messages,
    // mirroring the source parser behavior.
    std::vector<std::string> resolvedOutputLayerNames = outputLayerNames;

    // The anchor centers depend only on the input size, the feature strides and the number of
    // anchors; they are cached across messages and refreshed when any of the three changes,
    // mirroring the source parser's cache refresh in the setters and build.
    bool anchorsCached = false;
    std::pair<std::uint32_t, std::uint32_t> cachedInputSize{0, 0};
    std::vector<std::int64_t> cachedStrides;
    std::int64_t cachedNumAnchors = 0;
    std::vector<std::vector<std::array<float, 2>>> anchors;

    while(mainLoop()) {
        auto nnData = input.get<dai::NNData>();
        if(!nnData) {
            continue;
        }

        const auto currentInputSize = inputSize;
        const auto currentStrides = featStrideFpn;
        const std::int64_t currentNumAnchors = numAnchors;

        // Extract: one score/bbox/kps layer triple per stride. The score tensor is flattened;
        // the bbox and kps tensors are paired 4 and 10 values per score, which accepts both
        // batched (1, N, C) and unbatched (N, C) tensors like the source reshapes.
        if(resolvedOutputLayerNames.empty()) {
            resolvedOutputLayerNames = nnData->getAllLayerNames();
        }
        std::vector<std::vector<float>> scoresPerStride;
        std::vector<std::vector<float>> bboxesPerStride;
        std::vector<std::vector<float>> kpsPerStride;
        scoresPerStride.reserve(currentStrides.size());
        bboxesPerStride.reserve(currentStrides.size());
        kpsPerStride.reserve(currentStrides.size());
        for(const std::int64_t stride : currentStrides) {
            const std::string scoreLayerName = "score_" + std::to_string(stride);
            const std::string bboxLayerName = "bbox_" + std::to_string(stride);
            const std::string kpsLayerName = "kps_" + std::to_string(stride);
            for(const auto& layerName : {scoreLayerName, bboxLayerName, kpsLayerName}) {
                DAI_CHECK_V(std::find(resolvedOutputLayerNames.begin(), resolvedOutputLayerNames.end(), layerName) != resolvedOutputLayerNames.end(),
                            "Layer {} not found in the model output.",
                            layerName);
            }

            auto scoreTensor = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, scoreLayerName);
            auto bboxTensor = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, bboxLayerName);
            auto kpsTensor = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, kpsLayerName);
            DAI_CHECK_V(bboxTensor.size() == scoreTensor.size() * utilities::SCRFDUtils::BBOX_VALUES_PER_ANCHOR,
                        "SCRFDParser: cannot reshape the {} tensor of size {} into shape ({}, {}).",
                        bboxLayerName,
                        bboxTensor.size(),
                        scoreTensor.size(),
                        utilities::SCRFDUtils::BBOX_VALUES_PER_ANCHOR);
            DAI_CHECK_V(kpsTensor.size() == scoreTensor.size() * utilities::SCRFDUtils::KPS_VALUES_PER_ANCHOR,
                        "SCRFDParser: cannot reshape the {} tensor of size {} into shape ({}, {}).",
                        kpsLayerName,
                        kpsTensor.size(),
                        scoreTensor.size(),
                        utilities::SCRFDUtils::KPS_VALUES_PER_ANCHOR);

            scoresPerStride.push_back(std::move(scoreTensor));
            bboxesPerStride.push_back(std::move(bboxTensor));
            kpsPerStride.push_back(std::move(kpsTensor));
        }

        // Compute
        if(!anchorsCached || cachedInputSize != currentInputSize || cachedStrides != currentStrides || cachedNumAnchors != currentNumAnchors) {
            anchors.clear();
            anchors.reserve(currentStrides.size());
            for(const std::int64_t stride : currentStrides) {
                anchors.push_back(utilities::SCRFDUtils::computeAnchorCenters(stride, currentInputSize.first, currentInputSize.second, currentNumAnchors));
            }
            cachedInputSize = currentInputSize;
            cachedStrides = currentStrides;
            cachedNumAnchors = currentNumAnchors;
            anchorsCached = true;
        }
        const auto detections = utilities::SCRFDUtils::computeScrfdDetections(bboxesPerStride,
                                                                              scoresPerStride,
                                                                              kpsPerStride,
                                                                              currentStrides,
                                                                              currentInputSize.first,
                                                                              currentInputSize.second,
                                                                              confidenceThreshold,
                                                                              iouThreshold,
                                                                              anchors);

        // Emit. All detections carry label 0; the first label name (when configured) is mapped
        // to every detection. Every detection carries 5 keypoints without confidence scores
        // (confidence -1), mirroring the source message creator.
        const std::vector<std::uint32_t> labels(detections.bboxes.size(), 0);
        std::vector<std::string> mappedLabelNames;
        if(!labelNames.empty()) {
            mappedLabelNames.assign(detections.bboxes.size(), labelNames.front());
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

        logger->debug("SCRFDParser created message with {} detections", message->detections.size());
        out.send(message);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
