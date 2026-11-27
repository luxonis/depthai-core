#include "depthai/beta/node/RegressionParser.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"
#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

namespace node {

namespace {

std::vector<float> computeRegressionPredictions(utilities::ClassificationUtils::ShapedTensorData tensor) {
    std::size_t nonSingletonDims = 0;
    for(const auto dim : tensor.dims) {
        if(dim != 1) {
            nonSingletonDims++;
        }
    }
    DAI_CHECK_V(nonSingletonDims <= 1,
                "RegressionParser: each prediction should be a float, but the tensor of shape ({}) is still multi-dimensional after squeezing its singleton "
                "dimensions.",
                fmt::join(tensor.dims, ", "));
    return std::move(tensor.values);
}

std::shared_ptr<Predictions> createRegressionMessage(const std::vector<float>& predictions) {
    auto message = std::make_shared<Predictions>();
    message->predictions.reserve(predictions.size());
    for(const auto prediction : predictions) {
        Prediction predictionObject;
        predictionObject.prediction = prediction;
        message->predictions.push_back(predictionObject);
    }
    return message;
}

}  // namespace

NNArchive RegressionParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive RegressionParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to RegressionParser::build");
    return *nnArchive;
}

std::shared_ptr<RegressionParser> RegressionParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<RegressionParser>(shared_from_this());
}

std::shared_ptr<RegressionParser> RegressionParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<RegressionParser>(shared_from_this());
}

void RegressionParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void RegressionParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void RegressionParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int regressionHeads = 0;
    auto regressionHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            regressionHeads++;
            regressionHead = head;
        }
    }

    DAI_CHECK_V(regressionHeads > 0, "NNArchive does not contain a RegressionParser head.");
    DAI_CHECK_V(regressionHeads == 1, "NNArchive contains {} RegressionParser heads. Please build with a specific head.", regressionHeads);

    setConfig(regressionHead);
}

void RegressionParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a RegressionParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Only one output layer supported for RegressionParser, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());
}  // The score_threshold key of some regression heads is ignored, as in the source parser

void RegressionParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string RegressionParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void RegressionParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool RegressionParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void RegressionParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHostVar ? "host" : "device");

    std::string resolvedOutputLayerName = properties.outputLayerName;

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }
        auto tGotInput = std::chrono::steady_clock::now();

        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(
                layerNames.size() == 1, "RegressionParser: expected 1 output layer, got {} layers. Please provide the output layer name.", layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        auto tensor = utilities::ClassificationUtils::getShapedTensorData(*nnData, resolvedOutputLayerName);

        auto predictions = computeRegressionPredictions(std::move(tensor));

        auto message = createRegressionMessage(predictions);  // Predictions are kept in the order the model emitted them
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("RegressionParser created message with {} values", message->predictions.size());
        auto tProcessed = std::chrono::steady_clock::now();
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }
        auto tAbsoluteEnd = std::chrono::steady_clock::now();
        this->logTiming(logger, tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
