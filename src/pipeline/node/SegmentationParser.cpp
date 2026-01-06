#include "depthai/pipeline/node/SegmentationParser.hpp"

#include <fmt/format.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "common/ModelType.hpp"
#include "common/TensorInfo.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/NNData.hpp"
#include "pipeline/datatype/SegmentationMask.hpp"
#include "pipeline/utilities/NNDataViewer.hpp"
#include "pipeline/utilities/SegmentationParser/SegmentationParserUtils.hpp"
#include "properties/Properties.hpp"
#include "utility/ErrorMacros.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/imgproc.hpp>
#endif

namespace dai {
namespace node {

SegmentationParser::~SegmentationParser() = default;

SegmentationParser::Properties& SegmentationParser::getProperties() {
    properties.parserConfig = *initialConfig;
    return properties;
}

void SegmentationParser::setNNArchive(const NNArchive& nnArchive) {
    constexpr int DEFAULT_SUPERBLOB_NUM_SHAVES = 8;
    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::BLOB:
            setNNArchiveBlob(nnArchive);
            break;
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, DEFAULT_SUPERBLOB_NUM_SHAVES);
            break;
        case dai::model::ModelType::DLC:
        case dai::model::ModelType::OTHER:
            setNNArchiveOther(nnArchive);
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. Please unpack the inner archive first.");
            break;
    }
}

std::shared_ptr<SegmentationParser> SegmentationParser::build(Node::Output& nnInput, const NNArchive& nnArchive) {
    setNNArchive(nnArchive);
    nnInput.link(input);
    return std::static_pointer_cast<SegmentationParser>(shared_from_this());
}

void SegmentationParser::setNNArchiveBlob(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::BLOB, "NNArchive type is not BLOB");
    mArchive = nnArchive;
    setConfig(nnArchive.getVersionedConfig());
}

void SegmentationParser::setNNArchiveSuperblob(const NNArchive& nnArchive, int /*numShaves*/) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB, "NNArchive type is not SUPERBLOB");
    mArchive = nnArchive;
    setConfig(nnArchive.getVersionedConfig());
}

void SegmentationParser::setNNArchiveOther(const NNArchive& nnArchive) {
    mArchive = nnArchive;
    setConfig(nnArchive.getVersionedConfig());
}

void SegmentationParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    archiveConfig = config;

    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto configV1 = config.getConfig<nn_archive::v1::Config>();

    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int segmentationHeads = 0;
    auto seghead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == "SegmentationParser") {
            segmentationHeads++;
            seghead = head;
        }
    }

    DAI_CHECK_V(segmentationHeads > 0, "NNArchive does not contain a segmentation head.");

    if(segmentationHeads > 1) {
        auto& logger = ThreadedNode::pimpl->logger;
        logger->warn("NNArchive contains multiple ({}) segmentation heads. Using the last one found. If multiple heads are expected, build with specific head.",
                     segmentationHeads);
    }

    setConfig(seghead);
}

std::shared_ptr<SegmentationParser> SegmentationParser::build(Node::Output& nnInput, const std::optional<dai::nn_archive::v1::Head>& head) {
    if(!head) {
        throw std::runtime_error("Head is null when building SegmentationParser with specific head.");
    }
    setConfig(*head);
    nnInput.link(input);
    return std::static_pointer_cast<SegmentationParser>(shared_from_this());
}

void SegmentationParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == "SegmentationParser", "Head parser is not SegmentationParser");
    DAI_CHECK_V(head.outputs != std::nullopt, "SegmentationParser head outputs need to be defined when building automatically.");
    std::vector<std::string> networkOutputs = *head.outputs;
    DAI_CHECK_V(networkOutputs.size() <= 1, "SegmentationParser supports only single output.");

    properties.networkOutputName = networkOutputs.size() == 1 ? networkOutputs[0] : "";

    properties.classesInOneLayer =
        head.metadata.extraParams.contains("classes_in_one_layer") ? head.metadata.extraParams.at("classes_in_one_layer").get<bool>() : false;

    if(head.metadata.confThreshold) {
        initialConfig->setConfidenceThreshold(static_cast<float>(*head.metadata.confThreshold));
    }

    if(head.metadata.classes) {
        initialConfig->setLabels(*head.metadata.classes);
    }
}

void SegmentationParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool SegmentationParser::runOnHost() const {
    return runOnHostVar;
}

std::string checkTensorName(const dai::NNData& nnData, const std::string& preferredName, std::shared_ptr<spdlog::async_logger>& logger) {
    const auto layerNames = nnData.getAllLayerNames();
    if(layerNames.empty()) {
        throw std::runtime_error("No tensors available in NNData.");
    }
    if(preferredName != "") {
        auto it = std::find(layerNames.begin(), layerNames.end(), preferredName);
        if(it != layerNames.end()) {
            return preferredName;
        } else {
            throw std::runtime_error("Preferred Segmentation tensor name '" + preferredName + "' not found in NNData outputs.");
        }
    }

    logger->debug("No network outputs specified, using first output only.");
    return layerNames.front();
}

void SegmentationParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    using namespace std::chrono;
    // logger->trace("Start SegmentationParser");

    const bool inputConfigSync = inputConfig.getWaitForMessage();
    const bool classesInSingleLayer = properties.classesInOneLayer;
    std::string preferredTensorName = properties.networkOutputName;
    inConfig = initialConfig;
    if(!inConfig) {
        throw std::runtime_error("SegmentationParser config is not initialized.");
    }

    while(isRunning()) {
        logger->debug("Running");
        auto tAbsoluteBeginning = steady_clock::now();

        // // Start with the current config (default to the initial parser config)
        // if(!inConfig) {
        //     inConfig = std::make_shared<SegmentationParserConfig>(properties.parserConfig);
        // }

        if(inputConfigSync) {
            auto cfg = inputConfig.get<dai::SegmentationParserConfig>();
            if(cfg) {
                inConfig = cfg;
            } else {
                logger->error("Invalid input config.");
            }
        } else {
            auto cfg = inputConfig.tryGet<dai::SegmentationParserConfig>();
            if(cfg) {
                inConfig = cfg;
            }
        }

        // auto config = inConfig ? inConfig : std::make_shared<SegmentationParserConfig>(properties.parserConfig);
        // const float confidenceThr = config->getConfidenceThreshold();

        std::shared_ptr<dai::NNData> sharedNNData = input.get<dai::NNData>();
        if(!sharedNNData) {
            logger->error("NN Data is empty. Skipping processing.");
            continue;
        }
        auto tAfterMessageBeginning = steady_clock::now();

        std::string networkOutputName = checkTensorName(*sharedNNData, preferredTensorName, logger);
        auto tensorInfo = sharedNNData->getTensorInfo(networkOutputName);
        if(!tensorInfo) {
            throw std::runtime_error("Tensor info for output layer " + networkOutputName + " is null.");
        }

        size_t maskWidth = static_cast<size_t>(tensorInfo->getWidth());
        size_t maskHeight = static_cast<size_t>(tensorInfo->getHeight());
        int channels = tensorInfo->getChannels();
        if(maskWidth <= 0 || maskHeight <= 0 || channels <= 0) {
            std::string errorMsg = "Invalid tensor dimensions retrieved for segmentation. Channels: " + std::to_string(channels)
                                   + ", height: " + std::to_string(maskHeight) + ", width: " + std::to_string(maskWidth) + ".";
            throw std::runtime_error(errorMsg);
        }

        auto outMask = std::make_shared<dai::SegmentationMask>();

        auto tParsingStart = steady_clock::now();

        if(!classesInSingleLayer) {  // standard case
            switch((*tensorInfo).dataType) {
                case dai::TensorInfo::DataType::I8:
                    utilities::SegmentationParserUtils::thresholdAndArgmaxTensor<int8_t>(*outMask, *sharedNNData, *tensorInfo, *inConfig, logger);
                    break;
                case dai::TensorInfo::DataType::U8F:
                    utilities::SegmentationParserUtils::thresholdAndArgmaxTensor<uint8_t>(*outMask, *sharedNNData, *tensorInfo, *inConfig, logger);
                    break;
                case dai::TensorInfo::DataType::INT:
                    utilities::SegmentationParserUtils::thresholdAndArgmaxTensor<int32_t>(*outMask, *sharedNNData, *tensorInfo, *inConfig, logger);
                    break;
                case dai::TensorInfo::DataType::FP32:
                    utilities::SegmentationParserUtils::thresholdAndArgmaxTensor<float>(*outMask, *sharedNNData, *tensorInfo, *inConfig, logger);
                    break;
                case dai::TensorInfo::DataType::FP16:
                    logger->trace("Parsing FP16 segmentation mask");
                    utilities::SegmentationParserUtils::thresholdAndArgmaxFp16Tensor(*outMask, *sharedNNData, *tensorInfo, *inConfig);
                    break;
                case dai::TensorInfo::DataType::FP64:
                default:
                    logger->error("Unsupported data type for segmentation parsing: {}", static_cast<int>(tensorInfo->dataType));
                    return;
            }
        } else {  // assume data is stored as INT in shape N x H x W  with N = 1
            DAI_CHECK_V(tensorInfo->dataType == dai::TensorInfo::DataType::INT, "When classes_in_one_layer is true, only INT data type is supported.");
            const auto src = sharedNNData->data->getData();
            const size_t bytes = maskWidth * maskHeight;
            outMask->setMask(src.subspan(tensorInfo->offset, bytes), maskWidth, maskHeight);
        }

        logger->warn("Segmentation mask parsing took {}ms", duration_cast<microseconds>(steady_clock::now() - tParsingStart).count() / 1000);

        auto tBeforeSend = steady_clock::now();
        if(inConfig && inConfig->getLabels().size() > 0) {
            outMask->setLabels(inConfig->getLabels());
        }
        outMask->setSequenceNum(sharedNNData->getSequenceNum());
        outMask->setTimestamp(sharedNNData->getTimestamp());
        outMask->setTimestampDevice(sharedNNData->getTimestampDevice());
        outMask->transformation = sharedNNData->transformation;
        outMask->transformation->setSize(outMask->getWidth(), outMask->getHeight());  // need to readjust size due to possible downsampling in parsing

        out.send(outMask);

        auto tAbsoluteEnd = steady_clock::now();
        logger->warn("Seg parser {}ms, processing {}ms, getting_frames {}ms, sending_frames {}ms",
                     duration_cast<microseconds>(tAbsoluteEnd - tAbsoluteBeginning).count() / 1000,
                     duration_cast<microseconds>(tBeforeSend - tAfterMessageBeginning).count() / 1000,
                     duration_cast<microseconds>(tAfterMessageBeginning - tAbsoluteBeginning).count() / 1000,
                     duration_cast<microseconds>(tAbsoluteEnd - tBeforeSend).count() / 1000);
    }
}

}  // namespace node
}  // namespace dai
