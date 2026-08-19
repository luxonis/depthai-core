#include "depthai/beta/node/RFDETRParser.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"
#include "beta/utilities/Detection/DetectionUtils.hpp"
#include "beta/utilities/RFDETR/RFDETRUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

RFDETRParserProperties::~RFDETRParserProperties() = default;

namespace node {

RFDETRParser::RFDETRParser(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<BetaNode, RFDETRParser, RFDETRParserProperties>(std::move(props)),
      initialConfig(std::make_shared<RFDETRParserConfig>(properties.initialConfig)) {}

RFDETRParser::Properties& RFDETRParser::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

NNArchive RFDETRParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive RFDETRParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to RFDETRParser::build");
    return *nnArchive;
}

std::shared_ptr<RFDETRParser> RFDETRParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<RFDETRParser>(shared_from_this());
}

std::shared_ptr<RFDETRParser> RFDETRParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<RFDETRParser>(shared_from_this());
}

void RFDETRParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void RFDETRParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void RFDETRParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int rfdetrHeads = 0;
    auto rfdetrHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            rfdetrHeads++;
            rfdetrHead = head;
        }
    }

    DAI_CHECK_V(rfdetrHeads > 0, "NNArchive does not contain an RFDETRParser head.");
    DAI_CHECK_V(rfdetrHeads == 1, "NNArchive contains {} RFDETRParser heads. Please build with a specific head.", rfdetrHeads);

    setConfig(rfdetrHead);

    // The input size is derived from the first model input's declared shape and layout,
    // mirroring the source build(). It stays unchanged when the shape or the layout is not
    // declared. A head carries no model input metadata, so this is only possible on the
    // full-archive path.
    if(!configV1.model.inputs.empty()) {
        const auto& modelInput = configV1.model.inputs.front();
        const std::string layout = modelInput.layout.value_or("");
        if(!modelInput.shape.empty() && !layout.empty()) {
            DAI_CHECK_V(layout == "NHWC" || layout == "NCHW", "Unsupported input layout: {}", layout);
            DAI_CHECK_V(modelInput.shape.size() == 4, "Expected a 4D input shape for layout {}, got {} dimensions.", layout, modelInput.shape.size());
            const std::int64_t width = layout == "NHWC" ? modelInput.shape[2] : modelInput.shape[3];
            const std::int64_t height = layout == "NHWC" ? modelInput.shape[1] : modelInput.shape[2];
            DAI_CHECK_V(width > 0 && width <= std::numeric_limits<std::uint32_t>::max() && height > 0 && height <= std::numeric_limits<std::uint32_t>::max(),
                        "Invalid input size ({}, {}) declared by the model input.",
                        width,
                        height);
            setInputSize(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height));
        }
    }
}

void RFDETRParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an RFDETRParser head, got '{}'.", head.parser);

    if(head.metadata.confThreshold.has_value()) {
        setConfidenceThreshold(static_cast<float>(*head.metadata.confThreshold));
    }
    if(head.metadata.maxDet.has_value()) {
        setMaxDetections(static_cast<int>(*head.metadata.maxDet));
    }
    if(head.metadata.classes.has_value()) {
        setLabelNames(*head.metadata.classes);
    }
    const auto& extraParams = head.metadata.extraParams;
    if(extraParams.is_object() && extraParams.contains("mask_conf") && !extraParams.at("mask_conf").is_null()) {
        const auto& maskConfJson = extraParams.at("mask_conf");
        DAI_CHECK(maskConfJson.is_number(), "mask_conf must be a number.");
        setMaskConfidence(maskConfJson.get<float>());
    }

    // The head output layers replace the configured output layer names, positionally (boxes,
    // class logits, optional mask logits), mirroring the source build() including its output
    // count validation.
    const std::vector<std::string> headOutputs = head.outputs.value_or(std::vector<std::string>{});
    DAI_CHECK_V(headOutputs.empty() || headOutputs.size() == 2 || headOutputs.size() == 3,
                "RFDETRParser expects 2 outputs for detection or 3 outputs for segmentation, got {} outputs: [{}].",
                headOutputs.size(),
                fmt::join(headOutputs, ", "));
    setOutputLayerNames(headOutputs);
}

void RFDETRParser::setConfidenceThreshold(float threshold) {
    initialConfig->setConfidenceThreshold(threshold);
}

float RFDETRParser::getConfidenceThreshold() const {
    return initialConfig->getConfidenceThreshold();
}

void RFDETRParser::setMaxDetections(int maxDetections) {
    initialConfig->setMaxDetections(maxDetections);
}

int RFDETRParser::getMaxDetections() const {
    return initialConfig->getMaxDetections();
}

void RFDETRParser::setLabelNames(const std::vector<std::string>& labelNames) {
    properties.labelNames = labelNames;
}

std::vector<std::string> RFDETRParser::getLabelNames() const {
    return properties.labelNames;
}

void RFDETRParser::setMaskConfidence(float maskConfidence) {
    initialConfig->setMaskConfidence(maskConfidence);
}

float RFDETRParser::getMaskConfidence() const {
    return initialConfig->getMaskConfidence();
}

void RFDETRParser::setOutputLayerNames(const std::vector<std::string>& outputLayerNames) {
    properties.outputLayerNames = outputLayerNames;
}

std::vector<std::string> RFDETRParser::getOutputLayerNames() const {
    return properties.outputLayerNames;
}

void RFDETRParser::setInputSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Input size must be greater than 0.");
    properties.inputSize = std::make_pair(width, height);
}

std::optional<std::pair<std::uint32_t, std::uint32_t>> RFDETRParser::getInputSize() const {
    return properties.inputSize;
}

void RFDETRParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool RFDETRParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void RFDETRParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("RFDETRParser started");

    RFDETRParserConfig activeConfig = getProperties().initialConfig;
    DAI_CHECK(activeConfig.validate(), "RFDETRParser initial configuration is invalid.");
    const bool inputConfigSync = inputConfig.getWaitForMessage();

    while(mainLoop()) {
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            std::shared_ptr<RFDETRParserConfig> candidate;
            if(inputConfigSync) {
                candidate = inputConfig.get<RFDETRParserConfig>();
            } else {
                auto candidates = inputConfig.tryGetAll<RFDETRParserConfig>();
                if(!candidates.empty()) {
                    candidate = candidates.back();
                }
            }
            if(candidate) {
                if(candidate->validate()) {
                    activeConfig = *candidate;
                } else {
                    logger->warn("RFDETRParser ignored an invalid runtime configuration.");
                }
            }

            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }

        const RFDETRParserConfig config = activeConfig;

        // Extract: the configured output layer names or, when not configured, all layer names
        // of the incoming NNData in their reported order; positionally boxes, class logits and
        // the optional mask logits, mirroring the source extract().
        const std::vector<std::string> layerNames = properties.outputLayerNames.empty() ? nnData->getAllLayerNames() : properties.outputLayerNames;
        DAI_CHECK_V(layerNames.size() >= 2 && layerNames.size() <= 3,
                    "Expected 2 or 3 output layers (boxes, logits, optional masks), got {} layers.",
                    layerNames.size());

        const auto boxesTensor = utilities::ClassificationUtils::getShapedTensorData(*nnData, layerNames[0]);
        const auto logitsTensor = utilities::ClassificationUtils::getShapedTensorData(*nnData, layerNames[1]);
        std::optional<utilities::ClassificationUtils::ShapedTensorData> masksTensor;
        if(layerNames.size() == 3) {
            masksTensor = utilities::ClassificationUtils::getShapedTensorData(*nnData, layerNames[2]);
        }

        // Compute
        const auto detections = utilities::RFDETRUtils::computeRfDetrDetections(boxesTensor,
                                                                                logitsTensor,
                                                                                masksTensor,
                                                                                config.confidenceThreshold,
                                                                                config.maxDetections,
                                                                                properties.labelNames,
                                                                                config.maskConfidence,
                                                                                properties.inputSize);
        if(detections.ignoredInstances > 0) {
            logger->warn("RFDETRParser can encode at most 255 instances in SegmentationMask; ignoring {} lowest-scoring instances.",
                         detections.ignoredInstances);
        }

        // Emit
        auto message = utilities::DetectionUtils::createDetectionMessage(detections.bboxes,
                                                                         detections.scores,
                                                                         /*angles=*/{},
                                                                         detections.labels,
                                                                         detections.labelNames,
                                                                         /*keypoints=*/{},
                                                                         /*keypointEdges=*/{},
                                                                         detections.segmentationMask,
                                                                         detections.maskWidth,
                                                                         detections.maskHeight);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("RFDETRParser created message with {} detections", message->detections.size());
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
