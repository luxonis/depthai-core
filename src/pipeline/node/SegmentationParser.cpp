#include "depthai/pipeline/node/SegmentationParser.hpp"

#include <fmt/format.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "common/TensorInfo.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "nn_archive/NNArchive.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/NNData.hpp"
#include "pipeline/datatype/SegmentationMask.hpp"
#include "pipeline/datatype/SegmentationParserConfig.hpp"
#include "pipeline/utilities/SegmentationParser/SegmentationParserUtils.hpp"
#include "properties/Properties.hpp"
#include "utility/ErrorMacros.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/imgproc.hpp>
#endif

namespace dai {
namespace node {

SegmentationParser::~SegmentationParser() = default;

void SegmentationParser::buildInternal() {
    auto device = getDevice();
    if(device) {
        auto platform = device->getPlatform();
        if(platform == Platform::RVC2) {
            setRunOnHost(true);
            std::cout << "SegmentationParser: For RVC2 platform, running on host." << std::endl;
        }
    }
}

SegmentationParser::Properties& SegmentationParser::getProperties() {
    return properties;
}

NNArchive SegmentationParser::createNNArchive(NNModelDescription& modelDesc) {
    // Download model from zoo
    if(modelDesc.platform.empty()) {
        DAI_CHECK(getDevice() != nullptr, "Device is not set.");
        modelDesc.platform = getDevice()->getPlatformAsString();
    }
    auto path = getModelFromZoo(modelDesc);
    auto modelType = model::readModelType(path);
    DAI_CHECK(modelType == model::ModelType::NNARCHIVE, "Model from zoo is not NNArchive - it needs to be a NNArchive to use the build function");
    auto nnArchive = NNArchive(path);
    return nnArchive;
}

NNArchive SegmentationParser::decodeModel(const Model& model) {
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

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to SegmentationParser::build");
    return *nnArchive;
}

std::shared_ptr<SegmentationParser> SegmentationParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<SegmentationParser>(shared_from_this());
}

void SegmentationParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    archiveConfig = config;

    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int segmentationHeads = 0;
    auto segHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == "SegmentationParser") {
            segmentationHeads++;
            segHead = head;
        }
    }

    DAI_CHECK_V(segmentationHeads > 0, "NNArchive does not contain a segmentation head.");
    DAI_CHECK_V(segmentationHeads == 1, "NNArchive contains " + std::to_string(segmentationHeads) + " segmentation heads. Please build with a specific head.");

    setConfig(segHead);
}

std::shared_ptr<SegmentationParser> SegmentationParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<SegmentationParser>(shared_from_this());
}

void SegmentationParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == "SegmentationParser", "The provided head is not a SegmentationParser head.");

    if(head.outputs->empty()) {
        properties.networkOutputName = "";
    } else {
        std::vector<std::string> networkOutputs = *head.outputs;
        DAI_CHECK_V(networkOutputs.size() <= 1, "SegmentationParser supports only single output.");
        properties.networkOutputName = networkOutputs[0];
    }

    if(head.metadata.extraParams.contains("classes_in_one_layer")) {
        properties.classesInOneLayer = head.metadata.extraParams.at("classes_in_one_layer").get<bool>();
    }

    if(head.metadata.extraParams.contains("background_class")) {
        properties.backgroundClass = head.metadata.extraParams.at("background_class").get<bool>();
    }

    if(head.metadata.classes) {
        properties.labels = *head.metadata.classes;
    }

    if(head.metadata.confThreshold) {
        initialConfig->setConfidenceThreshold(static_cast<float>(*head.metadata.confThreshold));
    }
}

void SegmentationParser::setLabels(const std::vector<std::string>& labels) {
    properties.labels = labels;
}

std::vector<std::string> SegmentationParser::getLabels() const {
    return properties.labels;
}

void SegmentationParser::setBackgroundClass(bool backgroundClass) {
    properties.backgroundClass = backgroundClass;
}

bool SegmentationParser::getBackgroundClass() const {
    return properties.backgroundClass;
}

void SegmentationParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool SegmentationParser::runOnHost() const {
    return runOnHostVar;
}

std::string checkTensorName(const dai::NNData& nnData, const std::string& preferredName, std::shared_ptr<spdlog::async_logger>& logger) {
    const auto layerNames = nnData.getAllLayerNames();
    DAI_CHECK_V(!layerNames.empty(), "No tensors available in NNData.");

    if(preferredName != "") {
        auto it = std::find(layerNames.begin(), layerNames.end(), preferredName);
        DAI_CHECK_V(it != layerNames.end(), "Preferred Segmentation tensor name '" + preferredName + "' not found in NNData outputs.");
        return preferredName;
    }

    logger->debug("No network outputs specified, using first output only.");
    return layerNames.front();
}

void SegmentationParser::validateTensor(std::optional<TensorInfo>& info) {
    DAI_CHECK_V(info, "Tensor info for output layer is null.");

    auto maskWidth = static_cast<size_t>(info->getWidth());
    auto maskHeight = static_cast<size_t>(info->getHeight());
    int channels = info->getChannels();

    DAI_CHECK_V(maskWidth > 0 && maskHeight > 0 && channels > 0,
                "Invalid tensor dimensions retrieved for segmentation. Channels: " + std::to_string(channels) + ", height: " + std::to_string(maskHeight)
                    + ", width: " + std::to_string(maskWidth) + ".");
    DAI_CHECK(channels <= 256, "SegmentationParser supports a maximum of 256 channels.");

    if(!properties.classesInOneLayer && properties.labels.size() > 0) {
        int expectedNumLabels = properties.labels.size();
        DAI_CHECK_V(expectedNumLabels == channels,
                    fmt::format("Number of provided labels ({}) does not match number of channels ({}).{}",
                                expectedNumLabels,
                                channels,
                                properties.backgroundClass
                                    ? " Note: background_class is set to true, make sure to add a background label to the beginning of the labels list."
                                    : ""));
    }
}

void SegmentationParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    using std::chrono::duration_cast;
    using std::chrono::microseconds;
    using std::chrono::steady_clock;
    logger->debug("Start SegmentationParser");

    const bool inputConfigSync = inputConfig.getWaitForMessage();
    const bool classesInSingleLayer = properties.classesInOneLayer;
    std::string preferredTensorName = properties.networkOutputName;
    if(!inConfig) {
        inConfig = initialConfig;
    }
    DAI_CHECK_V(inConfig, "SegmentationParser config is not initialized.");

    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        std::shared_ptr<dai::NNData> sharedNNData;
        {
            auto blockEvent = this->inputBlockEvent();
            auto cfg = inputConfigSync ? inputConfig.get<dai::SegmentationParserConfig>() : inputConfig.tryGet<dai::SegmentationParserConfig>();
            if(cfg) {
                inConfig = cfg;
            } else if(inputConfigSync) {
                logger->error("Invalid input config.");
            }

            sharedNNData = input.get<dai::NNData>();
            if(!sharedNNData) {
                logger->error("NN Data is empty. Skipping processing.");
                continue;
            }
        }
        auto tAfterMessageBeginning = steady_clock::now();

        std::string networkOutputName = checkTensorName(*sharedNNData, preferredTensorName, logger);
        auto tensorInfo = sharedNNData->getTensorInfo(networkOutputName);
        validateTensor(tensorInfo);

        if(properties.backgroundClass && tensorInfo->getChannels() <= 1) {
            logger->warn("backgroundClass is set to true, but the output tensor has {} channel. Ignoring backgroundClass setting.", tensorInfo->getChannels());
            properties.backgroundClass = false;
        }

        auto outMask = std::make_shared<dai::SegmentationMask>();
        if(!classesInSingleLayer) {
            utilities::SegmentationParserUtils::computeSegmentationMask(*outMask, *sharedNNData, *tensorInfo, *inConfig, properties.backgroundClass, logger);
        } else {
            // assume data is stored as INT in shape N x H x W  with N = 1
            DAI_CHECK_V(tensorInfo->dataType == dai::TensorInfo::DataType::INT, "When classes_in_one_layer is true, only INT data type is supported.");
            utilities::SegmentationParserUtils::copySingleLayerMaskData(*outMask, *sharedNNData, *tensorInfo, *inConfig, logger);
        }

        auto tBeforeSend = steady_clock::now();
        if(properties.labels.size() > 0) {
            outMask->setLabels(properties.labels);
        }
        outMask->setSequenceNum(sharedNNData->getSequenceNum());
        outMask->setTimestamp(sharedNNData->getTimestamp());
        outMask->setTimestampDevice(sharedNNData->getTimestampDevice());
        outMask->transformation = sharedNNData->transformation;
        outMask->transformation->setSize(outMask->getWidth(), outMask->getHeight());

        {
            auto blockEvent = this->outputBlockEvent();
            out.send(outMask);
        }

        auto tAbsoluteEnd = steady_clock::now();
        logger->trace("Seg parser {}ms, processing {}ms, getting_frames {}ms, sending_frames {}ms",
                      duration_cast<microseconds>(tAbsoluteEnd - tAbsoluteBeginning).count() / 1000,
                      duration_cast<microseconds>(tBeforeSend - tAfterMessageBeginning).count() / 1000,
                      duration_cast<microseconds>(tAfterMessageBeginning - tAbsoluteBeginning).count() / 1000,
                      duration_cast<microseconds>(tAbsoluteEnd - tBeforeSend).count() / 1000);
    }
}

}  // namespace node
}  // namespace dai
