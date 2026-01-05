#include "depthai/pipeline/node/SegmentationParser.hpp"

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

    DAI_CHECK_V(segmentationHeads > 0, "NNArchive does not contain a segmentation head. Found {} segmentation heads.", segmentationHeads);

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
    } else {
        printf("SegmentationParser: No class labels defined in the head metadata.\n");
    }
}

void SegmentationParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool SegmentationParser::runOnHost() const {
    return runOnHostVar;
}

uint8_t clampClassIndex(int idx) {
    if(idx < 0 || idx > MAX_CLASS_INDEX) {
        return BACKGROUND_INDEX;
    }
    return idx;
}

void SegmentationParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    using namespace std::chrono;
    logger->warn("Start SegmentationParser");

    const bool inputConfigSync = inputConfig.getWaitForMessage();
    const bool classesInSingleLayer = properties.classesInOneLayer;
    std::string networkOutputName = properties.networkOutputName;  // move to a resolveNetworkOutputName function alongside the layerNames etc. part of the code
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

        std::shared_ptr<dai::NNData> sharedInputData = input.get<dai::NNData>();
        if(!sharedInputData) {
            logger->error("NN Data is empty. Skipping processing.");
            continue;
        }
        auto tAfterMessageBeginning = steady_clock::now();

        const auto layerNames = sharedInputData->getAllLayerNames();
        if(layerNames.empty()) {
            logger->error("No tensors available in NNData. Skipping processing.");
            continue;
        }
        if(networkOutputName == "") {
            logger->debug("No network outputs specified, using first output only.");
            networkOutputName = layerNames.front();
        }
        auto tensorInfo = sharedInputData->getTensorInfo(networkOutputName);
        if(!tensorInfo) {
            logger->error("Tensor info for specified output layer {} is null. Skipping processing.", networkOutputName);
            continue;
        }

        size_t maskWidth = static_cast<size_t>(tensorInfo->getWidth());
        size_t maskHeight = static_cast<size_t>(tensorInfo->getHeight());
        std::vector<uint8_t> maskData;
        int channels = tensorInfo->getChannels();

        if(maskWidth <= 0 || maskHeight <= 0 || channels <= 0) {
            std::string errorMsg = "Invalid tensor dimensions retrieved for segmentation parsing. Channels: " + std::to_string(channels)
                                   + ", height: " + std::to_string(maskHeight) + ", width: " + std::to_string(maskWidth) + ".";
            throw std::runtime_error(errorMsg);
        }

        const size_t planeSize = maskWidth * maskHeight;
        maskData.assign(planeSize, BACKGROUND_INDEX);
        auto tNNviewerStart = steady_clock::now();
        NNDataViewer viewer(*tensorInfo, sharedInputData->data, logger);
        if(!viewer.build()) {
            logger->error("Failed to build NNDataViewer for tensor {}. Skipping processing.", tensorInfo->name);
            continue;
        }

        auto tParsingStart = steady_clock::now();
        if(classesInSingleLayer) {  // assume data is stored as INT in shape N x H x W  with N = 1
            // logger->debug("Parsing segmentation mask with classes in single layer. Assuming INT data.");
            // for(size_t idx = 0; idx < planeSize; ++idx) {
            //     maskData[idx] = clampClassIndex(static_cast<int>(std::lround(logitsBuffer[idx])));
            // }
        } else {  // move to an argmax callable function
            utilities::SegmentationParserUtils::parseSegmentationMask(*sharedInputData, *tensorInfo, maskData, *inConfig, logger);
            // std::vector<float> bestVals(planeSize, -1.0f);
            // const float* logitsPtr = logitsBuffer.data();

            // // 2. Iterate Channels FIRST (Outer loop), then Pixels (Inner Loop)
            // for(int ch = 0; ch < channels; ++ch) {
            //     // Pointer to the start of this channel plane
            //     const float* channelPtr = logitsPtr + (static_cast<size_t>(ch) * planeSize);

            //     // Linear scan over the plane
            //     for(size_t idx = 0; idx < planeSize; ++idx) {
            //         float val = channelPtr[idx];

            //         // Update max if this channel is better
            //         if(val > bestVals[idx]) {
            //             bestVals[idx] = val;
            //             // Only update class if confidence threshold met
            //             // Note: You can also apply threshold at the very end to save checks
            //             if(val >= confidenceThr) {
            //                 maskData[idx] = clampClassIndex(ch);
            //             }
            //         }
            //     }
        }
        // }
        logger->warn("Segmentation mask parsing took {}ms", duration_cast<microseconds>(steady_clock::now() - tParsingStart).count() / 1000);  // 7ms

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        size_t targetWidth = maskWidth;
        size_t targetHeight = maskHeight;
        if(inConfig && inConfig->outputWidth > 0 && inConfig->outputHeight > 0) {
            targetWidth = inConfig->outputWidth;
            targetHeight = inConfig->outputHeight;
        }

        if(targetWidth != maskWidth || targetHeight != maskHeight) {
            logger->debug("Resizing segmentation mask from {}x{} to {}x{}", maskWidth, maskHeight, targetWidth, targetHeight);
            cv::Mat maskMat(static_cast<int>(maskHeight), static_cast<int>(maskWidth), CV_8UC1, maskData.data());
            cv::Mat resized;
            int interpolation = cv::INTER_NEAREST;
            if(inConfig && inConfig->resizeMode == SegmentationParserConfig::ResizeMode::INTER_LINEAR) {
                interpolation = cv::INTER_LINEAR;
            }
            cv::resize(maskMat, resized, cv::Size(static_cast<int>(targetWidth), static_cast<int>(targetHeight)), 0, 0, interpolation);
            maskData.assign(resized.data, resized.data + resized.total());
            maskWidth = targetWidth;
            maskHeight = targetHeight;
        }
#endif

        auto tBeforeSend = steady_clock::now();

        auto outMask = std::make_shared<dai::SegmentationMask>();
        try {
            outMask->setMask(maskData, maskWidth, maskHeight);
        } catch(const std::exception& exc) {
            logger->error("Failed to set segmentation mask: {}", exc.what());
            continue;
        }
        if(inConfig && inConfig->getLabels().size() > 0) {
            outMask->setLabels(inConfig->getLabels());
        }
        outMask->setSequenceNum(sharedInputData->getSequenceNum());
        outMask->setTimestamp(sharedInputData->getTimestamp());
        outMask->setTimestampDevice(sharedInputData->getTimestampDevice());
        outMask->transformation = sharedInputData->transformation;

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
