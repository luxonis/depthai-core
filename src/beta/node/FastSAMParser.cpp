#include "depthai/beta/node/FastSAMParser.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"
#include "beta/utilities/FastSAM/FastSAMUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace node {

namespace {

/**
 * Retrieve a tensor from NNData as dequantized FP32 values in NCHW orientation, matching the
 * source parser's output.getTensor(name, StorageOrder.NCHW, dequantize=True). The tensor's
 * stored order is honored; the data is permuted to NCHW, not just re-labeled.
 */
utilities::ClassificationUtils::ShapedTensorData getNchwTensorData(dai::NNData& nnData, const std::string& tensorName) {
#ifdef DEPTHAI_XTENSOR_SUPPORT
    xt::xarray<float> tensor = nnData.getTensor<float>(tensorName, TensorInfo::StorageOrder::NCHW, true);
    utilities::ClassificationUtils::ShapedTensorData result;
    result.dims.assign(tensor.shape().begin(), tensor.shape().end());
    result.values.assign(tensor.begin(), tensor.end());
    return result;
#else
    (void)nnData;
    (void)tensorName;
    throw std::runtime_error(
        "FastSAMParser requires xtensor support to convert the model output tensors to NCHW orientation, but xtensor support is not available.");
#endif
}

}  // namespace

NNArchive FastSAMParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive FastSAMParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to FastSAMParser::build");
    return *nnArchive;
}

std::shared_ptr<FastSAMParser> FastSAMParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<FastSAMParser>(shared_from_this());
}

std::shared_ptr<FastSAMParser> FastSAMParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<FastSAMParser>(shared_from_this());
}

void FastSAMParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void FastSAMParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void FastSAMParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int fastsamHeads = 0;
    auto fastsamHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            fastsamHeads++;
            fastsamHead = head;
        }
    }

    DAI_CHECK_V(fastsamHeads > 0, "NNArchive does not contain a FastSAMParser head.");
    DAI_CHECK_V(fastsamHeads == 1, "NNArchive contains {} FastSAMParser heads. Please build with a specific head.", fastsamHeads);

    setConfig(fastsamHead);
}

void FastSAMParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a FastSAMParser head, got '{}'.", head.parser);

    // The head output layer names configure the YOLO and mask output layers by the "_yolo" and
    // "_masks" substring filters of the source build(), each only when at least one matches.
    const std::vector<std::string> headOutputs = head.outputs.value_or(std::vector<std::string>{});
    std::vector<std::string> yoloLayerNames;
    std::vector<std::string> maskLayerNames;
    for(const auto& name : headOutputs) {
        if(name.find("_yolo") != std::string::npos) {
            yoloLayerNames.push_back(name);
        }
        if(name.find("_masks") != std::string::npos) {
            maskLayerNames.push_back(name);
        }
    }
    if(!yoloLayerNames.empty()) {
        setYoloOutputs(yoloLayerNames);
    }
    if(!maskLayerNames.empty()) {
        setMaskOutputs(maskLayerNames);
    }

    if(head.metadata.confThreshold.has_value()) {
        setConfidenceThreshold(static_cast<float>(*head.metadata.confThreshold));
    }
    if(head.metadata.nClasses.has_value()) {
        setNumClasses(static_cast<std::int32_t>(*head.metadata.nClasses));
    }
    if(head.metadata.iouThreshold.has_value()) {
        setIouThreshold(static_cast<float>(*head.metadata.iouThreshold));
    }

    // The remaining keys have no typed metadata fields and live in the head metadata extra
    // parameters. Like the source build(), the protos output layer name is read from the
    // "protos_output" key; the standard NN Archive YOLO metadata declares "protos_outputs"
    // (plural), which the source does not consume, so the default layer name applies to such
    // archives.
    const auto& extraParams = head.metadata.extraParams;
    if(extraParams.is_object()) {
        if(extraParams.contains("protos_output") && !extraParams.at("protos_output").is_null()) {
            const auto& protosJson = extraParams.at("protos_output");
            DAI_CHECK(protosJson.is_string(), "protos_output must be a string.");
            setProtosOutput(protosJson.get<std::string>());
        }
        if(extraParams.contains("mask_conf") && !extraParams.at("mask_conf").is_null()) {
            const auto& maskConfJson = extraParams.at("mask_conf");
            DAI_CHECK(maskConfJson.is_number(), "mask_conf must be a number.");
            setMaskConfidence(maskConfJson.get<float>());
        }
        if(extraParams.contains("prompt") && !extraParams.at("prompt").is_null()) {
            const auto& promptJson = extraParams.at("prompt");
            DAI_CHECK(promptJson.is_string(), "Prompt must be a string.");
            setPrompt(promptJson.get<std::string>());
        }
        if(extraParams.contains("points") && !extraParams.at("points").is_null()) {
            const auto& pointsJson = extraParams.at("points");
            DAI_CHECK(pointsJson.is_array() && pointsJson.size() == 2 && pointsJson.at(0).is_number_integer() && pointsJson.at(1).is_number_integer(),
                      "Points must be a list of 2 integers.");
            setPoints(pointsJson.at(0).get<std::int32_t>(), pointsJson.at(1).get<std::int32_t>());
        }
        if(extraParams.contains("point_label") && !extraParams.at("point_label").is_null()) {
            const auto& pointLabelJson = extraParams.at("point_label");
            DAI_CHECK(pointLabelJson.is_number_integer(), "Point label must be an integer.");
            setPointLabel(pointLabelJson.get<std::int32_t>());
        }
        if(extraParams.contains("bbox") && !extraParams.at("bbox").is_null()) {
            const auto& bboxJson = extraParams.at("bbox");
            DAI_CHECK(bboxJson.is_array() && bboxJson.size() == 4
                          && std::all_of(bboxJson.begin(), bboxJson.end(), [](const nlohmann::json& v) { return v.is_number_integer(); }),
                      "Bounding box must be a list of 4 integers.");
            setBoundingBox({bboxJson.at(0).get<std::int32_t>(),
                            bboxJson.at(1).get<std::int32_t>(),
                            bboxJson.at(2).get<std::int32_t>(),
                            bboxJson.at(3).get<std::int32_t>()});
        }
    }
}

void FastSAMParser::setConfidenceThreshold(float threshold) {
    DAI_CHECK(threshold >= 0.0f && threshold <= 1.0f, "Confidence threshold must be between 0 and 1.");
    this->confidenceThreshold = threshold;
}

float FastSAMParser::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void FastSAMParser::setNumClasses(std::int32_t numClasses) {
    DAI_CHECK(numClasses >= 1, "Number of classes must be greater than 0.");
    this->numClasses = numClasses;
}

std::int32_t FastSAMParser::getNumClasses() const {
    return numClasses;
}

void FastSAMParser::setIouThreshold(float iouThreshold) {
    DAI_CHECK(iouThreshold >= 0.0f && iouThreshold <= 1.0f, "IOU threshold must be between 0 and 1.");
    this->iouThreshold = iouThreshold;
}

float FastSAMParser::getIouThreshold() const {
    return iouThreshold;
}

void FastSAMParser::setMaskConfidence(float maskConfidence) {
    DAI_CHECK(maskConfidence >= 0.0f && maskConfidence <= 1.0f, "Mask confidence must be between 0 and 1.");
    this->maskConfidence = maskConfidence;
}

float FastSAMParser::getMaskConfidence() const {
    return maskConfidence;
}

void FastSAMParser::setPrompt(const std::string& prompt) {
    DAI_CHECK(prompt == "everything" || prompt == "bbox" || prompt == "point", "Prompt must be one of 'everything', 'bbox', or 'point'");
    this->prompt = prompt;
}

std::string FastSAMParser::getPrompt() const {
    return prompt;
}

void FastSAMParser::setPoints(std::int32_t x, std::int32_t y) {
    this->points = std::make_pair(x, y);
}

std::optional<std::pair<std::int32_t, std::int32_t>> FastSAMParser::getPoints() const {
    return points;
}

void FastSAMParser::setPointLabel(std::int32_t pointLabel) {
    this->pointLabel = pointLabel;
}

std::optional<std::int32_t> FastSAMParser::getPointLabel() const {
    return pointLabel;
}

void FastSAMParser::setBoundingBox(const std::array<std::int32_t, 4>& bbox) {
    this->boundingBox = bbox;
}

std::optional<std::array<std::int32_t, 4>> FastSAMParser::getBoundingBox() const {
    return boundingBox;
}

void FastSAMParser::setYoloOutputs(const std::vector<std::string>& yoloOutputs) {
    this->yoloOutputs = yoloOutputs;
}

std::vector<std::string> FastSAMParser::getYoloOutputs() const {
    return yoloOutputs;
}

void FastSAMParser::setMaskOutputs(const std::vector<std::string>& maskOutputs) {
    this->maskOutputs = maskOutputs;
}

std::vector<std::string> FastSAMParser::getMaskOutputs() const {
    return maskOutputs;
}

void FastSAMParser::setProtosOutput(const std::string& protosOutput) {
    this->protosOutput = protosOutput;
}

std::string FastSAMParser::getProtosOutput() const {
    return protosOutput;
}

void FastSAMParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("FastSAMParser started");

    DAI_CHECK(prompt == "everything" || prompt == "bbox" || prompt == "point", "Prompt must be one of 'everything', 'bbox', or 'point'");

    while(mainLoop()) {
        auto nnData = input.get<dai::NNData>();
        if(!nnData) {
            continue;
        }

        // Extract: the YOLO output layers sorted by name, the mask output layers (all layer
        // names of the incoming NNData when none are configured) filtered by the "mask"
        // substring and sorted by name, and the protos output layer ("protos_output" when not
        // configured), each as dequantized FP32 NCHW tensors, mirroring the source extract()
        // and get_segmentation_outputs().
        std::vector<std::string> yoloLayerNames = yoloOutputs;
        std::sort(yoloLayerNames.begin(), yoloLayerNames.end());
        std::vector<utilities::ClassificationUtils::ShapedTensorData> yoloTensors;
        yoloTensors.reserve(yoloLayerNames.size());
        for(const auto& name : yoloLayerNames) {
            yoloTensors.push_back(getNchwTensorData(*nnData, name));
        }

        std::vector<std::string> maskLayerNames = maskOutputs.empty() ? nnData->getAllLayerNames() : maskOutputs;
        maskLayerNames.erase(
            std::remove_if(maskLayerNames.begin(), maskLayerNames.end(), [](const std::string& name) { return name.find("mask") == std::string::npos; }),
            maskLayerNames.end());
        std::sort(maskLayerNames.begin(), maskLayerNames.end());
        std::vector<utilities::ClassificationUtils::ShapedTensorData> maskTensors;
        maskTensors.reserve(maskLayerNames.size());
        for(const auto& name : maskLayerNames) {
            maskTensors.push_back(getNchwTensorData(*nnData, name));
        }

        const std::string protosLayerName = protosOutput.empty() ? "protos_output" : protosOutput;
        const auto protosTensor = getNchwTensorData(*nnData, protosLayerName);
        DAI_CHECK_V(protosTensor.dims.size() == 4,
                    "FastSAM protos output '{}' must be a 4D NCHW tensor, got {} dimensions.",
                    protosLayerName,
                    protosTensor.dims.size());
        const std::size_t protosLen = protosTensor.dims[1];

        // Compute
        utilities::FastSAMUtils::PromptConfig promptConfig;
        promptConfig.prompt = prompt;
        promptConfig.points = points;
        promptConfig.pointLabel = pointLabel;
        promptConfig.bbox = boundingBox;
        const auto result = utilities::FastSAMUtils::computeFastsamMask(
            yoloTensors, maskTensors, protosTensor, protosLen, confidenceThreshold, numClasses, iouThreshold, maskConfidence, promptConfig);

        // Emit
        auto message = std::make_shared<dai::SegmentationMask>();
        message->setMask(result.mergedMask, result.width, result.height);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("FastSAMParser created segmentation message with {} masks", result.maskCount);
        out.send(message);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
