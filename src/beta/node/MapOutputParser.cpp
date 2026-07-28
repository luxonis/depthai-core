#include "depthai/beta/node/MapOutputParser.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
 * Tensor values in the tensor's stored order together with the tensor shape.
 */
struct TensorData {
    /// Tensor values in row-major order over dims.
    std::vector<float> values;
    /// Tensor shape in the tensor's stored order.
    std::vector<std::size_t> dims;
};

/**
 * Retrieve a tensor from NNData as dequantized FP32 values in the tensor's stored order, matching
 * the source parser's output.getTensor(name, dequantize=True).
 */
TensorData getTensorData(dai::NNData& nnData, const std::string& tensorName) {
#ifdef DEPTHAI_XTENSOR_SUPPORT
    xt::xarray<float> tensor = nnData.getTensor<float>(tensorName, true);
    TensorData result;
    result.dims.assign(tensor.shape().begin(), tensor.shape().end());
    result.values.assign(tensor.begin(), tensor.end());
    return result;
#else
    (void)nnData;
    (void)tensorName;
    throw std::runtime_error("MapOutputParser requires xtensor support to retrieve the output map tensor, but xtensor support is not available.");
#endif
}

/**
 * Format a tensor shape like a Python tuple, e.g. (2, 240, 320) or (4,), for source-identical
 * error messages.
 */
std::string formatShape(const std::vector<std::size_t>& dims) {
    std::string result = "(";
    for(std::size_t i = 0; i < dims.size(); i++) {
        if(i > 0) {
            result += ", ";
        }
        result += std::to_string(dims[i]);
    }
    if(dims.size() == 1) {
        result += ",";
    }
    result += ")";
    return result;
}

/**
 * Reduce a model map output tensor to a 2D HW map, mirroring the source compute_map_output()
 * utility. Leading dimensions of 1 are squeezed while the tensor has more than two dimensions;
 * the tensor must then be 2D, or 3D with a singleton trailing dimension that is squeezed as well.
 */
TensorData computeMapOutput(TensorData tensor) {
    while(tensor.dims.size() > 2 && tensor.dims[0] == 1) {
        tensor.dims.erase(tensor.dims.begin());
    }

    if(tensor.dims.size() == 2) {
        return tensor;
    }

    if(tensor.dims.size() == 3 && tensor.dims.back() == 1) {
        tensor.dims.pop_back();
        return tensor;
    }

    throw std::runtime_error("Expected HW, NHW, or HWN with singleton N; got " + formatShape(tensor.dims) + ".");
}

/**
 * Create the output Map2D message from a 2D HW map, mirroring the source create_map_message()
 * creator. When min-max scaling is requested, the values are scaled to [0, 1]; a constant map is
 * left unchanged.
 */
std::shared_ptr<Map2D> createMapMessage(TensorData map, bool minMaxScaling) {
    if(minMaxScaling) {
        DAI_CHECK(!map.values.empty(), "Cannot min-max scale an empty map.");
        const auto [minIt, maxIt] = std::minmax_element(map.values.begin(), map.values.end());
        const float minValue = *minIt;
        const float maxValue = *maxIt;
        if(minValue != maxValue) {
            const float range = maxValue - minValue;
            for(auto& value : map.values) {
                value = (value - minValue) / range;
            }
        }
    }

    auto message = std::make_shared<Map2D>();
    message->setMap(map.values, map.dims[1], map.dims[0]);
    return message;
}

}  // namespace

NNArchive MapOutputParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive MapOutputParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to MapOutputParser::build");
    return *nnArchive;
}

std::shared_ptr<MapOutputParser> MapOutputParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<MapOutputParser>(shared_from_this());
}

std::shared_ptr<MapOutputParser> MapOutputParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<MapOutputParser>(shared_from_this());
}

void MapOutputParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void MapOutputParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void MapOutputParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int mapOutputHeads = 0;
    auto mapOutputHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            mapOutputHeads++;
            mapOutputHead = head;
        }
    }

    DAI_CHECK_V(mapOutputHeads > 0, "NNArchive does not contain a MapOutputParser head.");
    DAI_CHECK_V(mapOutputHeads == 1, "NNArchive contains {} MapOutputParser heads. Please build with a specific head.", mapOutputHeads);

    setConfig(mapOutputHead);
}

void MapOutputParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a MapOutputParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "MapOutputParser expects exactly 1 output layer, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());

    // The min_max_scaling key without a typed metadata field lives in the head metadata extra
    // parameters.
    const auto& extraParams = head.metadata.extraParams;

    if(extraParams.is_object() && extraParams.contains("min_max_scaling") && !extraParams.at("min_max_scaling").is_null()) {
        const auto& minMaxScalingJson = extraParams.at("min_max_scaling");
        DAI_CHECK(minMaxScalingJson.is_boolean(), "min_max_scaling must be a boolean.");
        setMinMaxScaling(minMaxScalingJson.get<bool>());
    }
}

void MapOutputParser::setOutputLayerName(const std::string& outputLayerName) {
    this->outputLayerName = outputLayerName;
}

std::string MapOutputParser::getOutputLayerName() const {
    return outputLayerName;
}

void MapOutputParser::setMinMaxScaling(bool minMaxScaling) {
    this->minMaxScaling = minMaxScaling;
}

bool MapOutputParser::getMinMaxScaling() const {
    return minMaxScaling;
}

void MapOutputParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("MapOutputParser started");

    // The resolved layer name persists across messages once auto-selected from a single-tensor
    // NNData, mirroring the source parser behavior.
    std::string resolvedOutputLayerName = outputLayerName;

    while(mainLoop()) {
        auto nnData = input.get<dai::NNData>();
        if(!nnData) {
            continue;
        }

        // Extract
        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(
                layerNames.size() == 1, "MapOutputParser: expected 1 output layer, got {} layers. Please provide the output layer name.", layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        auto tensor = getTensorData(*nnData, resolvedOutputLayerName);

        // Compute. The map dimensions are derived from the tensor shape.
        auto map = computeMapOutput(std::move(tensor));

        // Emit
        auto message = createMapMessage(std::move(map), minMaxScaling);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("MapOutputParser created map message with size {}x{}", message->getWidth(), message->getHeight());
        out.send(message);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
