#include "depthai/beta/node/ClassificationParser.hpp"

#include <cstdint>
#include <memory>
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

ClassificationParserProperties::~ClassificationParserProperties() = default;

namespace node {

NNArchive ClassificationParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive ClassificationParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to ClassificationParser::build");
    return *nnArchive;
}

std::shared_ptr<ClassificationParser> ClassificationParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<ClassificationParser>(shared_from_this());
}

std::shared_ptr<ClassificationParser> ClassificationParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<ClassificationParser>(shared_from_this());
}

void ClassificationParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void ClassificationParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void ClassificationParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int classificationHeads = 0;
    auto classificationHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            classificationHeads++;
            classificationHead = head;
        }
    }

    DAI_CHECK_V(classificationHeads > 0, "NNArchive does not contain a ClassificationParser head.");
    DAI_CHECK_V(classificationHeads == 1, "NNArchive contains {} ClassificationParser heads. Please build with a specific head.", classificationHeads);

    setConfig(classificationHead);
}

void ClassificationParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a ClassificationParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Only one output layer supported for ClassificationParser, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());

    if(head.metadata.classes.has_value()) {
        setClasses(*head.metadata.classes);
    }
    if(head.metadata.nClasses.has_value()) {
        properties.nClasses = *head.metadata.nClasses;
    }
    if(head.metadata.isSoftmax.has_value()) {
        setSoftmax(*head.metadata.isSoftmax);
    }
}

void ClassificationParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string ClassificationParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void ClassificationParser::setClasses(const std::vector<std::string>& classes) {
    properties.classes = classes;
    properties.nClasses = static_cast<std::int64_t>(classes.size());
}

std::vector<std::string> ClassificationParser::getClasses() const {
    return properties.classes;
}

void ClassificationParser::setSoftmax(bool isSoftmax) {
    properties.isSoftmax = isSoftmax;
}

bool ClassificationParser::getSoftmax() const {
    return properties.isSoftmax;
}

void ClassificationParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool ClassificationParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void ClassificationParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("ClassificationParser started");

    // The resolved layer name persists across messages once auto-selected from a
    // single-tensor NNData, mirroring the source parser behavior.
    std::string resolvedOutputLayerName = properties.outputLayerName;

    while(mainLoop()) {
        auto nnData = input.get<dai::NNData>();
        if(!nnData) {
            continue;
        }

        // Extract
        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(layerNames.size() == 1,
                        "ClassificationParser: expected 1 output layer, got {} layers. Please provide the output layer name.",
                        layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        auto scores = utilities::ClassificationUtils::getFlattenedTensorData(*nnData, resolvedOutputLayerName);
        DAI_CHECK_V(properties.nClasses == 0 || static_cast<std::int64_t>(scores.size()) == properties.nClasses,
                    "ClassificationParser: number of labels and scores mismatch. Provided {} class names and {} scores.",
                    properties.nClasses,
                    scores.size());

        // Compute
        scores = utilities::ClassificationUtils::computeClassificationScores(std::move(scores), properties.isSoftmax);

        // Emit
        auto message = utilities::ClassificationUtils::createClassificationMessage(properties.classes, scores);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("ClassificationParser created message with {} classes", message->classes.size());
        out.send(message);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
