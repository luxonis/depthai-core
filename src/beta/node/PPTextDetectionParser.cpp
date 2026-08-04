#include "depthai/beta/node/PPTextDetectionParser.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "beta/utilities/Detection/DetectionUtils.hpp"
#include "beta/utilities/PPText/PPTextUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

PPTextDetectionParserProperties::~PPTextDetectionParserProperties() = default;

namespace node {

namespace {

/**
 * Predictions tensor values in NCHW orientation together with the tensor shape.
 */
struct NchwTensorData {
    /// Tensor values in row-major order over dims.
    std::vector<float> values;
    /// Tensor shape in NCHW orientation.
    std::vector<std::size_t> dims;
};

/**
 * Retrieve a tensor from NNData as dequantized FP32 values in NCHW orientation, matching the
 * source parser's output.getTensor(name, StorageOrder.NCHW, dequantize=True). The tensor's
 * stored order is honored; the data is permuted to NCHW, not just re-labeled.
 */
NchwTensorData getNchwTensorData(dai::NNData& nnData, const std::string& tensorName) {
#ifdef DEPTHAI_XTENSOR_SUPPORT
    xt::xarray<float> tensor = nnData.getTensor<float>(tensorName, TensorInfo::StorageOrder::NCHW, true);
    NchwTensorData result;
    result.dims.assign(tensor.shape().begin(), tensor.shape().end());
    result.values.assign(tensor.begin(), tensor.end());
    return result;
#else
    (void)nnData;
    (void)tensorName;
    throw std::runtime_error(
        "PPTextDetectionParser requires xtensor support to convert the predictions tensor to NCHW orientation, but xtensor support is not available.");
#endif
}

}  // namespace

NNArchive PPTextDetectionParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive PPTextDetectionParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to PPTextDetectionParser::build");
    return *nnArchive;
}

std::shared_ptr<PPTextDetectionParser> PPTextDetectionParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<PPTextDetectionParser>(shared_from_this());
}

std::shared_ptr<PPTextDetectionParser> PPTextDetectionParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<PPTextDetectionParser>(shared_from_this());
}

void PPTextDetectionParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void PPTextDetectionParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void PPTextDetectionParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int textDetectionHeads = 0;
    auto textDetectionHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            textDetectionHeads++;
            textDetectionHead = head;
        }
    }

    DAI_CHECK_V(textDetectionHeads > 0, "NNArchive does not contain a PPTextDetectionParser head.");
    DAI_CHECK_V(textDetectionHeads == 1, "NNArchive contains {} PPTextDetectionParser heads. Please build with a specific head.", textDetectionHeads);

    setConfig(textDetectionHead);
}

void PPTextDetectionParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a PPTextDetectionParser head, got '{}'.", head.parser);

    if(head.metadata.confThreshold.has_value()) {
        setConfidenceThreshold(static_cast<float>(*head.metadata.confThreshold));
    }
    if(head.metadata.maxDet.has_value()) {
        setMaxDetections(static_cast<int>(*head.metadata.maxDet));
    }

    // The mask threshold has no typed metadata field and lives in the head metadata extra
    // parameters.
    const auto& extraParams = head.metadata.extraParams;
    if(extraParams.is_object() && extraParams.contains("mask_threshold") && !extraParams.at("mask_threshold").is_null()) {
        const auto& maskThresholdJson = extraParams.at("mask_threshold");
        DAI_CHECK(maskThresholdJson.is_number(), "Mask threshold must be a float.");
        setMaskThreshold(maskThresholdJson.get<float>());
    }

    // The head's declared output names are intentionally not consumed: the source parser
    // resolves the output layer from the runtime NN results, which keeps archives whose head
    // output name differs from the model's declared output name working.
}

void PPTextDetectionParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string PPTextDetectionParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void PPTextDetectionParser::setConfidenceThreshold(float threshold) {
    properties.confidenceThreshold = threshold;
}

float PPTextDetectionParser::getConfidenceThreshold() const {
    return properties.confidenceThreshold;
}

void PPTextDetectionParser::setMaskThreshold(float maskThreshold) {
    properties.maskThreshold = maskThreshold;
}

float PPTextDetectionParser::getMaskThreshold() const {
    return properties.maskThreshold;
}

void PPTextDetectionParser::setMaxDetections(int maxDetections) {
    properties.maxDetections = maxDetections;
}

int PPTextDetectionParser::getMaxDetections() const {
    return properties.maxDetections;
}

void PPTextDetectionParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool PPTextDetectionParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void PPTextDetectionParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("PPTextDetectionParser started");

    // The resolved layer name persists across messages once auto-selected from a
    // single-tensor NNData, mirroring the source parser behavior.
    std::string resolvedOutputLayerName = properties.outputLayerName;

    while(mainLoop()) {
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }

        // Extract
        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(layerNames.size() == 1, "Expected 1 output layer, got {} layers. Please provide the output_layer_name.", layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }
        auto predictions = getNchwTensorData(*nnData, resolvedOutputLayerName);

        // Compute. The probability map height/width are derived from the tensor shape.
        const auto detections = utilities::PPTextUtils::parsePaddleDetectionOutputs(
            predictions.values, predictions.dims, properties.maskThreshold, properties.confidenceThreshold, properties.maxDetections);

        // Emit. The detections carry no labels, mirroring the source
        // create_detection_message(bboxes, scores, angles=angles) call.
        auto message = utilities::DetectionUtils::createDetectionMessage(detections.bboxes, detections.scores, detections.angles);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("PPTextDetectionParser created message with {} detections", message->detections.size());
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
