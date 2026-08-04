#include "depthai/beta/node/HRNetParser.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Keypoints/KeypointsUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

HRNetParserProperties::~HRNetParserProperties() = default;

namespace node {

namespace {

/**
 * Heatmap tensor values in NCHW orientation together with the tensor shape.
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
    throw std::runtime_error("HRNetParser requires xtensor support to convert the heatmap tensor to NCHW orientation, but xtensor support is not available.");
#endif
}

}  // namespace

NNArchive HRNetParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive HRNetParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to HRNetParser::build");
    return *nnArchive;
}

std::shared_ptr<HRNetParser> HRNetParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<HRNetParser>(shared_from_this());
}

std::shared_ptr<HRNetParser> HRNetParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<HRNetParser>(shared_from_this());
}

void HRNetParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void HRNetParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void HRNetParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int hrnetHeads = 0;
    auto hrnetHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            hrnetHeads++;
            hrnetHead = head;
        }
    }

    DAI_CHECK_V(hrnetHeads > 0, "NNArchive does not contain an HRNetParser head.");
    DAI_CHECK_V(hrnetHeads == 1, "NNArchive contains {} HRNetParser heads. Please build with a specific head.", hrnetHeads);

    setConfig(hrnetHead);
}

void HRNetParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an HRNetParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Only one output layer supported for HRNetParser, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());

    // The keypoint-specific keys without typed metadata fields live in the head metadata extra
    // parameters.
    const auto& extraParams = head.metadata.extraParams;

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

void HRNetParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string HRNetParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void HRNetParser::setScoreThreshold(float threshold) {
    DAI_CHECK(threshold >= 0.0f && threshold <= 1.0f, "Confidence threshold must be between 0 and 1.");
    properties.scoreThreshold = threshold;
}

float HRNetParser::getScoreThreshold() const {
    return properties.scoreThreshold;
}

void HRNetParser::setLabelNames(const std::vector<std::string>& labelNames) {
    properties.labelNames = labelNames;
}

std::vector<std::string> HRNetParser::getLabelNames() const {
    return properties.labelNames;
}

void HRNetParser::setEdges(const std::vector<Edge>& edges) {
    properties.edges = edges;
}

std::vector<Edge> HRNetParser::getEdges() const {
    return properties.edges;
}

void HRNetParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool HRNetParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void HRNetParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("HRNetParser started");

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
            DAI_CHECK_V(
                layerNames.size() == 1, "HRNetParser: expected 1 output layer, got {} layers. Please provide the output layer name.", layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        auto heatmaps = getNchwTensorData(*nnData, resolvedOutputLayerName);

        // Compute. The number of keypoints and the heatmap size are derived from the tensor shape.
        auto hrnetKeypoints = utilities::KeypointsUtils::computeHrnetKeypoints(heatmaps.values, heatmaps.dims);

        // Emit. Keypoints with a score below the score threshold are dropped and the edges are
        // filtered and remapped to the kept keypoints.
        auto message = utilities::KeypointsUtils::createKeypointsMessage(
            hrnetKeypoints.coordinates, std::move(hrnetKeypoints.scores), properties.scoreThreshold, properties.labelNames, properties.edges);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("HRNetParser created message with {} keypoints", message->keypointsList.size());
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
