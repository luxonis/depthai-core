#include "depthai/beta/node/LaneDetectionParser.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"
#include "beta/utilities/LaneDetection/LaneDetectionUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

LaneDetectionParserProperties::~LaneDetectionParserProperties() = default;

namespace node {

NNArchive LaneDetectionParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive LaneDetectionParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to LaneDetectionParser::build");
    return *nnArchive;
}

std::shared_ptr<LaneDetectionParser> LaneDetectionParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<LaneDetectionParser>(shared_from_this());
}

std::shared_ptr<LaneDetectionParser> LaneDetectionParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<LaneDetectionParser>(shared_from_this());
}

void LaneDetectionParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void LaneDetectionParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void LaneDetectionParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int laneDetectionHeads = 0;
    auto laneDetectionHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            laneDetectionHeads++;
            laneDetectionHead = head;
        }
    }

    DAI_CHECK_V(laneDetectionHeads > 0, "NNArchive does not contain a LaneDetectionParser head.");
    DAI_CHECK_V(laneDetectionHeads == 1, "NNArchive contains {} LaneDetectionParser heads. Please build with a specific head.", laneDetectionHeads);

    setConfig(laneDetectionHead);

    // The input size is derived from the model input's declared shape and layout. A head carries
    // no model input metadata, so this is only possible on the full-archive path.
    DAI_CHECK_V(configV1.model.inputs.size() == 1, "Only one input supported for LaneDetectionParser, got {} inputs.", configV1.model.inputs.size());
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

void LaneDetectionParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a LaneDetectionParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Only one output layer supported for LaneDetectionParser, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());

    // The lane-detection-specific keys without typed metadata fields live in the head metadata
    // extra parameters. Missing keys keep the already configured values, mirroring the source
    // parser behavior.
    const auto& extraParams = head.metadata.extraParams;

    if(extraParams.is_object() && extraParams.contains("row_anchors") && !extraParams.at("row_anchors").is_null()) {
        const auto& rowAnchorsJson = extraParams.at("row_anchors");
        DAI_CHECK(rowAnchorsJson.is_array(), "row_anchors must be a list.");
        std::vector<std::int64_t> headRowAnchors;
        headRowAnchors.reserve(rowAnchorsJson.size());
        for(const auto& anchorJson : rowAnchorsJson) {
            DAI_CHECK(anchorJson.is_number_integer(), "Row anchors must be a list of integers.");
            headRowAnchors.push_back(anchorJson.get<std::int64_t>());
        }
        setRowAnchors(headRowAnchors);
    }
    if(extraParams.is_object() && extraParams.contains("griding_num") && !extraParams.at("griding_num").is_null()) {
        const auto& gridingNumJson = extraParams.at("griding_num");
        DAI_CHECK(gridingNumJson.is_number_integer(), "Griding number must be an integer.");
        setGridingNum(gridingNumJson.get<std::int64_t>());
    }
    if(extraParams.is_object() && extraParams.contains("cls_num_per_lane") && !extraParams.at("cls_num_per_lane").is_null()) {
        const auto& clsNumPerLaneJson = extraParams.at("cls_num_per_lane");
        DAI_CHECK(clsNumPerLaneJson.is_number_integer(), "Number of points per lane must be an integer.");
        setClsNumPerLane(clsNumPerLaneJson.get<std::int64_t>());
    }
}

void LaneDetectionParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string LaneDetectionParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void LaneDetectionParser::setRowAnchors(const std::vector<std::int64_t>& rowAnchors) {
    DAI_CHECK(!rowAnchors.empty(), "Row anchors must not be empty.");
    properties.rowAnchors = rowAnchors;
}

std::vector<std::int64_t> LaneDetectionParser::getRowAnchors() const {
    return properties.rowAnchors;
}

void LaneDetectionParser::setGridingNum(std::int64_t gridingNum) {
    DAI_CHECK(gridingNum > 1, "Griding number must be greater than 1.");
    properties.gridingNum = gridingNum;
}

std::optional<std::int64_t> LaneDetectionParser::getGridingNum() const {
    return properties.gridingNum;
}

void LaneDetectionParser::setClsNumPerLane(std::int64_t clsNumPerLane) {
    DAI_CHECK(clsNumPerLane > 0, "Number of points per lane must be greater than 0.");
    properties.clsNumPerLane = clsNumPerLane;
}

std::optional<std::int64_t> LaneDetectionParser::getClsNumPerLane() const {
    return properties.clsNumPerLane;
}

void LaneDetectionParser::setInputSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Input size must be greater than 0.");
    properties.inputSize = std::make_pair(width, height);
}

std::optional<std::pair<std::uint32_t, std::uint32_t>> LaneDetectionParser::getInputSize() const {
    return properties.inputSize;
}

void LaneDetectionParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool LaneDetectionParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void LaneDetectionParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("LaneDetectionParser started");

    DAI_CHECK(!properties.rowAnchors.empty(), "Row anchors must be specified!");
    DAI_CHECK(properties.gridingNum.has_value(), "Griding number must be specified!");
    DAI_CHECK(properties.clsNumPerLane.has_value(), "Number of points per lane must be specified!");
    DAI_CHECK(properties.inputSize.has_value(), "Input size must be specified! Configure it from a full NNArchive or with setInputSize().");
    DAI_CHECK_V(properties.rowAnchors.size() >= static_cast<std::size_t>(*properties.clsNumPerLane),
                "Expected at least clsNumPerLane = {} row anchors, got {} row anchors.",
                *properties.clsNumPerLane,
                properties.rowAnchors.size());

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
            DAI_CHECK_V(layerNames.size() == 1,
                        "LaneDetectionParser: expected 1 output layer, got {} layers. Please provide the output layer name.",
                        layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        // The tensor is read in its stored order, matching the source parser's
        // output.getTensor(name, dequantize=True) with the default storage order.
        auto tensor = utilities::ClassificationUtils::getShapedTensorData(*nnData, resolvedOutputLayerName);

        // Compute. The number of lanes is derived from the tensor's last dimension.
        auto points = utilities::LaneDetectionUtils::decodeUfld(tensor.values,
                                                                tensor.dims,
                                                                properties.rowAnchors,
                                                                *properties.gridingNum,
                                                                *properties.clsNumPerLane,
                                                                properties.inputSize->first,
                                                                properties.inputSize->second);

        // Emit. Every lane produces one cluster, including empty lanes, labeled sequentially
        // from 0.
        auto message = utilities::LaneDetectionUtils::createClustersMessage(points);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("LaneDetectionParser created message with {} clusters", message->clusters.size());
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
