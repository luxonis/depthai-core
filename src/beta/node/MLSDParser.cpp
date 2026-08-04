#include "depthai/beta/node/MLSDParser.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"
#include "beta/utilities/MLSD/MLSDUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

MLSDParserProperties::~MLSDParserProperties() = default;

namespace node {

namespace {

/**
 * tpMap tensor values in NCHW orientation together with the tensor shape.
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
    throw std::runtime_error("MLSDParser requires xtensor support to convert the tpMap tensor to NCHW orientation, but xtensor support is not available.");
#endif
}

}  // namespace

NNArchive MLSDParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive MLSDParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to MLSDParser::build");
    return *nnArchive;
}

std::shared_ptr<MLSDParser> MLSDParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<MLSDParser>(shared_from_this());
}

std::shared_ptr<MLSDParser> MLSDParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<MLSDParser>(shared_from_this());
}

void MLSDParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void MLSDParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void MLSDParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int mlsdHeads = 0;
    auto mlsdHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            mlsdHeads++;
            mlsdHead = head;
        }
    }

    DAI_CHECK_V(mlsdHeads > 0, "NNArchive does not contain an MLSDParser head.");
    DAI_CHECK_V(mlsdHeads == 1, "NNArchive contains {} MLSDParser heads. Please build with a specific head.", mlsdHeads);

    setConfig(mlsdHead);

    // The input size is derived from the model input's declared shape and layout. A head carries
    // no model input metadata, so this is only possible on the full-archive path; the source
    // parser instead hard-codes the 512x512 input size of all known M-LSD models, which is kept
    // as the default.
    DAI_CHECK_V(configV1.model.inputs.size() == 1, "Only one input supported for MLSDParser, got {} inputs.", configV1.model.inputs.size());
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

void MLSDParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an MLSDParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 2, "Only two output layers are supported for MLSDParser, got {} layers.", numOutputs);
    // The layers are assigned by substring: a layer whose name contains "tpMap" is the tpMap
    // layer and a layer whose name contains "heat" is the heat layer, mirroring the source
    // parser. A layer matching neither leaves the corresponding name unconfigured.
    for(const auto& layer : *head.outputs) {
        if(layer.find("tpMap") != std::string::npos) {
            setOutputLayerTPMap(layer);
        } else if(layer.find("heat") != std::string::npos) {
            setOutputLayerHeat(layer);
        }
    }

    // The M-LSD-specific keys without typed metadata fields live in the head metadata extra
    // parameters. Missing keys keep the already configured values, mirroring the source parser
    // behavior.
    const auto& extraParams = head.metadata.extraParams;

    if(extraParams.is_object() && extraParams.contains("topk_n") && !extraParams.at("topk_n").is_null()) {
        const auto& topkJson = extraParams.at("topk_n");
        DAI_CHECK(topkJson.is_number_integer(), "topk_n must be an integer.");
        setTopK(topkJson.get<int>());
    }
    if(extraParams.is_object() && extraParams.contains("score_thr") && !extraParams.at("score_thr").is_null()) {
        const auto& scoreThresholdJson = extraParams.at("score_thr");
        DAI_CHECK(scoreThresholdJson.is_number(), "score_thr must be a float.");
        setScoreThreshold(scoreThresholdJson.get<float>());
    }
    if(extraParams.is_object() && extraParams.contains("dist_thr") && !extraParams.at("dist_thr").is_null()) {
        const auto& distanceThresholdJson = extraParams.at("dist_thr");
        DAI_CHECK(distanceThresholdJson.is_number(), "dist_thr must be a float.");
        setDistanceThreshold(distanceThresholdJson.get<float>());
    }
}

void MLSDParser::setOutputLayerTPMap(const std::string& outputLayerTPMap) {
    properties.outputLayerTPMap = outputLayerTPMap;
}

std::string MLSDParser::getOutputLayerTPMap() const {
    return properties.outputLayerTPMap;
}

void MLSDParser::setOutputLayerHeat(const std::string& outputLayerHeat) {
    properties.outputLayerHeat = outputLayerHeat;
}

std::string MLSDParser::getOutputLayerHeat() const {
    return properties.outputLayerHeat;
}

void MLSDParser::setTopK(int topK) {
    DAI_CHECK(topK > 0, "topk_n must be a positive integer.");
    properties.topK = topK;
}

int MLSDParser::getTopK() const {
    return properties.topK;
}

void MLSDParser::setScoreThreshold(float scoreThreshold) {
    properties.scoreThreshold = scoreThreshold;
}

float MLSDParser::getScoreThreshold() const {
    return properties.scoreThreshold;
}

void MLSDParser::setDistanceThreshold(float distanceThreshold) {
    properties.distanceThreshold = distanceThreshold;
}

float MLSDParser::getDistanceThreshold() const {
    return properties.distanceThreshold;
}

void MLSDParser::setInputSize(std::uint32_t width, std::uint32_t height) {
    DAI_CHECK(width > 0 && height > 0, "Input size must be greater than 0.");
    properties.inputSize = std::make_pair(width, height);
}

std::pair<std::uint32_t, std::uint32_t> MLSDParser::getInputSize() const {
    return properties.inputSize;
}

void MLSDParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool MLSDParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void MLSDParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("MLSDParser started");

    DAI_CHECK(!properties.outputLayerTPMap.empty(),
              "Output layer containing the tpMap tensor is not set. Please use setOutputLayerTPMap method or correct NN archive.");
    DAI_CHECK(!properties.outputLayerHeat.empty(),
              "Output layer containing the heat tensor is not set. Please use setOutputLayerHeat method or correct NN archive.");

    while(mainLoop()) {
        auto nnData = input.get<dai::NNData>();
        if(!nnData) {
            continue;
        }

        // Extract. The tpMap tensor is permuted to NCHW orientation; the heat tensor is read in
        // its stored order and flattened.
        auto tpMap = getNchwTensorData(*nnData, properties.outputLayerTPMap);
        auto heat = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, properties.outputLayerHeat);

        // Compute. The heat map grid size is derived from the tpMap tensor shape.
        auto decoded = utilities::MLSDUtils::computeMlsdLines(tpMap.values,
                                                              tpMap.dims,
                                                              heat,
                                                              properties.topK,
                                                              properties.scoreThreshold,
                                                              properties.distanceThreshold,
                                                              properties.inputSize.first,
                                                              properties.inputSize.second);

        // Emit. Lines are ordered by descending confidence score.
        auto message = utilities::MLSDUtils::createLinesMessage(decoded);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("MLSDParser created message with {} lines", message->lines.size());
        out.send(message);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
