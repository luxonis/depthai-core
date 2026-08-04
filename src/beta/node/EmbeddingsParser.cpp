#include "depthai/beta/node/EmbeddingsParser.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

EmbeddingsParserProperties::~EmbeddingsParserProperties() = default;

namespace node {

NNArchive EmbeddingsParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive EmbeddingsParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to EmbeddingsParser::build");
    return *nnArchive;
}

std::shared_ptr<EmbeddingsParser> EmbeddingsParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<EmbeddingsParser>(shared_from_this());
}

std::shared_ptr<EmbeddingsParser> EmbeddingsParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<EmbeddingsParser>(shared_from_this());
}

void EmbeddingsParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void EmbeddingsParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void EmbeddingsParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int embeddingsHeads = 0;
    auto embeddingsHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            embeddingsHeads++;
            embeddingsHead = head;
        }
    }

    DAI_CHECK_V(embeddingsHeads > 0, "NNArchive does not contain an EmbeddingsParser head.");
    DAI_CHECK_V(embeddingsHeads == 1, "NNArchive contains {} EmbeddingsParser heads. Please build with a specific head.", embeddingsHeads);

    setConfig(embeddingsHead);
}

void EmbeddingsParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an EmbeddingsParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Embeddings head should have only one output layer, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());
}

void EmbeddingsParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string EmbeddingsParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void EmbeddingsParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool EmbeddingsParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void EmbeddingsParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("EmbeddingsParser started");

    while(mainLoop()) {
        auto nnData = input.get<dai::NNData>();
        if(!nnData) {
            continue;
        }

        // Extract: validate that exactly one output layer is selected. When the output layer name
        // is configured, the selection is trivially unambiguous and no per-message check is needed,
        // mirroring the source parser behavior.
        if(properties.outputLayerName.empty()) {
            const auto layerNames = nnData->getAllLayerNames();
            DAI_CHECK_V(layerNames.size() == 1,
                        "EmbeddingsParser: Embeddings head should have only one output layer, got {} layers. Please provide the output layer name.",
                        layerNames.size());
        }

        // Compute is the identity and emit forwards the same message unchanged, so all tensors,
        // sequence number, timestamps, and image transformation metadata are preserved.
        logger->debug("EmbeddingsParser forwarding NNData message");
        out.send(nnData);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
