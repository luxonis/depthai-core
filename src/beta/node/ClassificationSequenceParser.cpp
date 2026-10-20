#include "depthai/beta/node/ClassificationSequenceParser.hpp"

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

ClassificationSequenceParserProperties::~ClassificationSequenceParserProperties() = default;

namespace node {

ClassificationSequenceParser::ClassificationSequenceParser(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<BetaNode, ClassificationSequenceParser, ClassificationSequenceParserProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

ClassificationSequenceParser::Properties& ClassificationSequenceParser::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

NNArchive ClassificationSequenceParser::createNNArchive(NNModelDescription& modelDesc) {
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

NNArchive ClassificationSequenceParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to ClassificationSequenceParser::build");
    return *nnArchive;
}

std::shared_ptr<ClassificationSequenceParser> ClassificationSequenceParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<ClassificationSequenceParser>(shared_from_this());
}

std::shared_ptr<ClassificationSequenceParser> ClassificationSequenceParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<ClassificationSequenceParser>(shared_from_this());
}

void ClassificationSequenceParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void ClassificationSequenceParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void ClassificationSequenceParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int classificationSequenceHeads = 0;
    auto classificationSequenceHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            classificationSequenceHeads++;
            classificationSequenceHead = head;
        }
    }

    DAI_CHECK_V(classificationSequenceHeads > 0, "NNArchive does not contain a ClassificationSequenceParser head.");
    DAI_CHECK_V(classificationSequenceHeads == 1,
                "NNArchive contains {} ClassificationSequenceParser heads. Please build with a specific head.",
                classificationSequenceHeads);

    setConfig(classificationSequenceHead);
}

void ClassificationSequenceParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not a ClassificationSequenceParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "Only one output layer supported for ClassificationSequenceParser, got {} layers.", numOutputs);
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

    // The sequence-specific keys live in the head metadata extra parameters.
    const auto& extraParams = head.metadata.extraParams;

    // Ignored indexes are reset when the head does not provide them, mirroring the source parser.
    std::vector<std::int32_t> headIgnoredIndexes;
    if(extraParams.is_object() && extraParams.contains("ignored_indexes") && !extraParams.at("ignored_indexes").is_null()) {
        const auto& ignoredIndexesJson = extraParams.at("ignored_indexes");
        DAI_CHECK(ignoredIndexesJson.is_array(), "Ignored indexes must be a list.");
        for(const auto& ignoredIndexJson : ignoredIndexesJson) {
            DAI_CHECK(ignoredIndexJson.is_number_integer(), "All ignored indexes must be integers.");
            headIgnoredIndexes.push_back(ignoredIndexJson.get<std::int32_t>());
        }
    }
    setIgnoredIndexes(headIgnoredIndexes);

    if(extraParams.is_object() && extraParams.contains("remove_duplicates") && !extraParams.at("remove_duplicates").is_null()) {
        const auto& removeDuplicatesJson = extraParams.at("remove_duplicates");
        DAI_CHECK(removeDuplicatesJson.is_boolean(), "remove_duplicates must be a boolean.");
        setRemoveDuplicates(removeDuplicatesJson.get<bool>());
    }
    if(extraParams.is_object() && extraParams.contains("concatenate_classes") && !extraParams.at("concatenate_classes").is_null()) {
        const auto& concatenateClassesJson = extraParams.at("concatenate_classes");
        DAI_CHECK(concatenateClassesJson.is_boolean(), "concatenate_classes must be a boolean.");
        setConcatenateClasses(concatenateClassesJson.get<bool>());
    }
}

void ClassificationSequenceParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string ClassificationSequenceParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void ClassificationSequenceParser::setClasses(const std::vector<std::string>& classes) {
    properties.classes = classes;
    properties.nClasses = static_cast<std::int64_t>(classes.size());
}

std::vector<std::string> ClassificationSequenceParser::getClasses() const {
    return properties.classes;
}

void ClassificationSequenceParser::setSoftmax(bool isSoftmax) {
    properties.isSoftmax = isSoftmax;
}

bool ClassificationSequenceParser::getSoftmax() const {
    return properties.isSoftmax;
}

void ClassificationSequenceParser::setIgnoredIndexes(const std::vector<std::int32_t>& ignoredIndexes) {
    initialConfig->setIgnoredIndexes(ignoredIndexes);
}

std::vector<std::int32_t> ClassificationSequenceParser::getIgnoredIndexes() const {
    return initialConfig->getIgnoredIndexes();
}

void ClassificationSequenceParser::setRemoveDuplicates(bool removeDuplicates) {
    initialConfig->setRemoveDuplicates(removeDuplicates);
}

bool ClassificationSequenceParser::getRemoveDuplicates() const {
    return initialConfig->getRemoveDuplicates();
}

void ClassificationSequenceParser::setConcatenateClasses(bool concatenateClasses) {
    initialConfig->setConcatenateClasses(concatenateClasses);
}

bool ClassificationSequenceParser::getConcatenateClasses() const {
    return initialConfig->getConcatenateClasses();
}

void ClassificationSequenceParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool ClassificationSequenceParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void ClassificationSequenceParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHostVar ? "host" : "device");
    auto config = getProperties().initialConfig;
    DAI_CHECK(config.validate(), "ClassificationSequenceParser initial configuration is invalid.");
    const bool inputConfigSync = inputConfig.getWaitForMessage();

    // The resolved layer name persists across messages once auto-selected from a
    // single-tensor NNData, mirroring the source parser behavior.
    std::string resolvedOutputLayerName = properties.outputLayerName;

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            std::shared_ptr<ClassificationSequenceParserConfig> candidate;
            if(inputConfigSync) {
                candidate = inputConfig.get<ClassificationSequenceParserConfig>();
            } else {
                auto candidates = inputConfig.tryGetAll<ClassificationSequenceParserConfig>();
                if(!candidates.empty()) {
                    candidate = candidates.back();
                }
            }
            if(candidate) {
                if(candidate->validate()) {
                    config = *candidate;
                } else {
                    logger->warn("ClassificationSequenceParser received an invalid configuration; retaining the previous configuration.");
                }
            }

            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }
        auto tGotInput = std::chrono::steady_clock::now();
        const ClassificationSequenceParserConfig configSnapshot = config;

        // Extract
        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(layerNames.size() == 1,
                        "ClassificationSequenceParser: expected 1 output layer, got {} layers. Please provide the output layer name.",
                        layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        DAI_CHECK(properties.nClasses != 0, "ClassificationSequenceParser: classes must be provided for classification.");

        auto tensor = utilities::ClassificationUtils::getShapedTensorData(*nnData, resolvedOutputLayerName);

        // Compute
        auto scores = utilities::ClassificationUtils::computeClassificationSequenceScores(std::move(tensor), properties.isSoftmax);

        // Emit
        auto message = utilities::ClassificationUtils::createClassificationSequenceMessage(
            properties.classes, scores, configSnapshot.ignoredIndexes, configSnapshot.removeDuplicates, configSnapshot.concatenateClasses);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("ClassificationSequenceParser created message with {} classes", message->classes.size());
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
