#include "depthai/beta/node/XFeatStereoParser.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
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

namespace {

/**
 * Read a (width, height) size from a head metadata extra parameter holding a list of two
 * positive integral numbers, or std::nullopt when the key is absent or null.
 */
std::optional<std::pair<std::uint32_t, std::uint32_t>> readSizeParam(const nlohmann::json& extraParams, const char* key) {
    if(!extraParams.is_object() || !extraParams.contains(key) || extraParams.at(key).is_null()) {
        return std::nullopt;
    }
    const auto& sizeJson = extraParams.at(key);
    DAI_CHECK_V(sizeJson.is_array() && sizeJson.size() == 2, "{} must be a list of two positive integers.", key);
    std::uint32_t size[2];
    for(std::size_t i = 0; i < 2; i++) {
        const auto& valueJson = sizeJson.at(i);
        DAI_CHECK_V(valueJson.is_number(), "{} must be a list of two positive integers.", key);
        const double value = valueJson.get<double>();
        DAI_CHECK_V(value > 0 && value <= std::numeric_limits<std::uint32_t>::max() && value == std::floor(value),
                    "{} must be a list of two positive integers, got {}.",
                    key,
                    value);
        size[i] = static_cast<std::uint32_t>(value);
    }
    return std::make_pair(size[0], size[1]);
}

}  // namespace

NNArchive XFeatStereoParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive XFeatStereoParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to XFeatStereoParser::build");
    return *nnArchive;
}

std::shared_ptr<XFeatStereoParser> XFeatStereoParser::build(Node::Output& reference, Node::Output& target, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    reference.link(referenceInput);
    target.link(targetInput);
    return std::static_pointer_cast<XFeatStereoParser>(shared_from_this());
}

std::shared_ptr<XFeatStereoParser> XFeatStereoParser::build(Node::Output& reference, Node::Output& target, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    reference.link(referenceInput);
    target.link(targetInput);
    return std::static_pointer_cast<XFeatStereoParser>(shared_from_this());
}

void XFeatStereoParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void XFeatStereoParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void XFeatStereoParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
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

    DAI_CHECK_V(xfeatHeads > 0, "NNArchive does not contain an XFeatStereoParser head.");
    DAI_CHECK_V(xfeatHeads == 1, "NNArchive contains {} XFeatStereoParser heads. Please build with a specific head.", xfeatHeads);

    setConfig(xfeatHead);

    // When the head metadata carries no input size, it is derived from the model input's
    // declared shape and layout. All known XFeat archives carry the input_size metadata key,
    // which then takes precedence like in the source parser; a head alone carries no model
    // input metadata, so this fallback is only possible on the full-archive path. The stereo
    // model declares one input; both sources run the same model and share the input size.
    if(!readSizeParam(xfeatHead.metadata.extraParams, "input_size").has_value()) {
        DAI_CHECK_V(configV1.model.inputs.size() == 1, "Only one input supported for XFeatStereoParser, got {} inputs.", configV1.model.inputs.size());
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
}

void XFeatStereoParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an XFeatStereoParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 3, "Only three output layers supported for XFeat, got {} layers.", numOutputs);
    // The layers are assigned by substring: a layer whose name contains "feats" is the feature
    // map layer, otherwise one containing "keypoints" is the keypoint logit layer and otherwise
    // one containing "heatmaps" is the reliability heat map layer, mirroring the source parser.
    // A layer matching none keeps the corresponding configured name.
    for(const auto& layer : *head.outputs) {
        if(layer.find("feats") != std::string::npos) {
            setOutputLayerFeats(layer);
        } else if(layer.find("keypoints") != std::string::npos) {
            setOutputLayerKeypoints(layer);
        } else if(layer.find("heatmaps") != std::string::npos) {
            setOutputLayerHeatmaps(layer);
        }
    }

    // The XFeat-specific keys without typed metadata fields live in the head metadata extra
    // parameters. Missing keys keep the already configured values, mirroring the source parser
    // behavior.
    const auto& extraParams = head.metadata.extraParams;

    if(const auto headOriginalSize = readSizeParam(extraParams, "original_size")) {
        setOriginalSize(headOriginalSize->first, headOriginalSize->second);
    }
    if(const auto headInputSize = readSizeParam(extraParams, "input_size")) {
        setInputSize(headInputSize->first, headInputSize->second);
    }
    if(extraParams.is_object() && extraParams.contains("max_keypoints") && !extraParams.at("max_keypoints").is_null()) {
        const auto& maxKeypointsJson = extraParams.at("max_keypoints");
        DAI_CHECK(maxKeypointsJson.is_number_integer(), "Maximum number of keypoints must be an int!");
        setMaxKeypoints(maxKeypointsJson.get<int>());
    }
}

void XFeatStereoParser::setOutputLayerFeats(const std::string& outputLayerFeats) {
    this->outputLayerFeats = outputLayerFeats;
}

std::string XFeatStereoParser::getOutputLayerFeats() const {
    return outputLayerFeats;
}

void XFeatStereoParser::setOutputLayerKeypoints(const std::string& outputLayerKeypoints) {
    this->outputLayerKeypoints = outputLayerKeypoints;
}

std::string XFeatStereoParser::getOutputLayerKeypoints() const {
    return outputLayerKeypoints;
}

void XFeatStereoParser::setOutputLayerHeatmaps(const std::string& outputLayerHeatmaps) {
    this->outputLayerHeatmaps = outputLayerHeatmaps;
}

std::string XFeatStereoParser::getOutputLayerHeatmaps() const {
    return outputLayerHeatmaps;
}

void XFeatStereoParser::setOriginalSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Original image size must be greater than 0.");
    this->originalSize = std::make_pair(width, height);
}

std::optional<std::pair<std::uint32_t, std::uint32_t>> XFeatStereoParser::getOriginalSize() const {
    return originalSize;
}

void XFeatStereoParser::setInputSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Input image size must be greater than 0.");
    this->inputSize = std::make_pair(width, height);
}

std::pair<std::uint32_t, std::uint32_t> XFeatStereoParser::getInputSize() const {
    return inputSize;
}

void XFeatStereoParser::setMaxKeypoints(int maxKeypoints) {
    this->maxKeypoints = maxKeypoints;
}

int XFeatStereoParser::getMaxKeypoints() const {
    return maxKeypoints;
}

void XFeatStereoParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("XFeatStereoParser started");

    DAI_CHECK(originalSize.has_value(), "Original image size must be specified!");
    DAI_CHECK(!outputLayerFeats.empty(), "Output layer containing features must be specified!");
    DAI_CHECK(!outputLayerKeypoints.empty(), "Output layer containing keypoints must be specified!");
    DAI_CHECK(!outputLayerHeatmaps.empty(), "Output layer containing heatmaps must be specified!");

    // The resize rates are computed once before the loop and shared by both sources, like the
    // source run().
    const double resizeRateW = static_cast<double>(originalSize->first) / static_cast<double>(inputSize.first);
    const double resizeRateH = static_cast<double>(originalSize->second) / static_cast<double>(inputSize.second);

    while(mainLoop()) {
        // One reference message followed by one target message per iteration; two sequential
        // blocking reads without further synchronization, like the source run().
        auto referenceOutput = referenceInput.get<dai::NNData>();
        auto targetOutput = targetInput.get<dai::NNData>();
        if(!referenceOutput || !targetOutput) {
            continue;
        }

        // Extract
        auto referenceTensors = utilities::XFeatUtils::extractXFeatTensors(*referenceOutput, outputLayerFeats, outputLayerKeypoints, outputLayerHeatmaps);
        auto targetTensors = utilities::XFeatUtils::extractXFeatTensors(*targetOutput, outputLayerFeats, outputLayerKeypoints, outputLayerHeatmaps);

        // Compute
        const auto referenceResult = utilities::XFeatUtils::detectAndCompute(referenceTensors.feats,
                                                                             referenceTensors.keypoints,
                                                                             referenceTensors.heatmaps,
                                                                             resizeRateW,
                                                                             resizeRateH,
                                                                             inputSize.first,
                                                                             inputSize.second,
                                                                             maxKeypoints);
        const auto targetResult = utilities::XFeatUtils::detectAndCompute(
            targetTensors.feats, targetTensors.keypoints, targetTensors.heatmaps, resizeRateW, resizeRateH, inputSize.first, inputSize.second, maxKeypoints);

        // Emit. The reference frame is checked first, like the source compute(); an empty
        // message carries the missing frame's timestamps and the reference frame's sequence
        // number.
        if(!referenceResult.has_value()) {
            auto message = std::make_shared<dai::TrackedFeatures>();
            message->setBufferMetadataFrom(referenceOutput);
            logger->debug("XFeatStereoParser: no reference keypoints found, sending empty TrackedFeatures message");
            out.send(message);
            continue;
        }
        if(!targetResult.has_value()) {
            auto message = std::make_shared<dai::TrackedFeatures>();
            message->setBufferMetadataFrom(targetOutput);
            message->setSequenceNum(referenceOutput->getSequenceNum());
            logger->debug("XFeatStereoParser: no target keypoints found, sending empty TrackedFeatures message");
            out.send(message);
            continue;
        }

        const auto matchedPoints = utilities::XFeatUtils::matchResults(*referenceResult, *targetResult);
        auto message = utilities::XFeatUtils::createTrackedFeaturesMessage(matchedPoints);
        // The matched message carries the target frame's timestamps and the reference frame's
        // sequence number, like the source emit().
        message->setBufferMetadataFrom(targetOutput);
        message->setSequenceNum(referenceOutput->getSequenceNum());
        logger->debug("XFeatStereoParser created message with {} tracked features", message->trackedFeatures.size());
        out.send(message);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
