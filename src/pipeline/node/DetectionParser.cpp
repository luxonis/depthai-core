#include "depthai/pipeline/node/DetectionParser.hpp"

#include <algorithm>
#include <cctype>
#include <csignal>
#include <cstddef>
#include <memory>
#include <vector>

#include "common/DetectionNetworkType.hpp"
#include "common/ModelType.hpp"
#include "common/YoloDecodingFamily.hpp"
#include "nn_archive/NNArchive.hpp"
#include "nn_archive/v1/Head.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/NNData.hpp"
#include "pipeline/utilities/DetectionParser/DetectionParserUtils.hpp"

// internal headers
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

DetectionParser::~DetectionParser() = default;

void DetectionParser::setNNArchive(const NNArchive& nnArchive) {
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

std::shared_ptr<DetectionParser> DetectionParser::build(Node::Output& nnInput, const NNArchive& nnArchive) {
    setNNArchive(nnArchive);

    const auto connections = nnInput.getConnections();
    const bool alreadyLinked = std::any_of(connections.begin(), connections.end(), [this](const auto& conn) { return conn.in == &input; });
    if(!alreadyLinked) {
        printf("Linking DetectionParser input\n");
        nnInput.link(input);
    }
    return std::static_pointer_cast<DetectionParser>(shared_from_this());
}

void DetectionParser::setModelPath(const std::filesystem::path& modelPath) {
    switch(model::readModelType(modelPath.string())) {
        case model::ModelType::BLOB:
        case model::ModelType::SUPERBLOB:
        case model::ModelType::DLC:
        case model::ModelType::OTHER:
            break;  // Just do nothing
        case model::ModelType::NNARCHIVE:
            setNNArchive(dai::NNArchive(modelPath.string()));
            break;
        default:
            DAI_CHECK_V(false, "Unknown model type");
            break;
    }
}

const NNArchiveVersionedConfig& DetectionParser::getNNArchiveVersionedConfig() const {
    DAI_CHECK_V(archiveConfig.has_value(), "NNArchiveVersionedConfig is not set. Use setNNArchive(...) first.");
    return archiveConfig.value();
}

void DetectionParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    archiveConfig = config;

    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    auto configV1 = config.getConfig<nn_archive::v1::Config>();

    const auto model = configV1.model;
    // TODO(jakgra) is NN Archive valid without this? why is this optional?
    DAI_CHECK(model.heads, "Heads array is not defined in the NN Archive config file.");

    std::vector<nn_archive::v1::Head> modelHeads = *model.heads;
    int yoloHeadIndex = 0;
    int numYoloHeads = 0;
    int numMobilenetHeads = 0;
    int mobilenetHeadIndex = 0;
    for(size_t i = 0; i < modelHeads.size(); i++) {
        if(modelHeads[i].parser == "YOLO" || modelHeads[i].parser == "YOLOExtendedParser") {
            yoloHeadIndex = static_cast<int>(i);
            numYoloHeads++;
        } else if(modelHeads[i].parser == "SSD" || modelHeads[i].parser == "MOBILENET") {
            numMobilenetHeads++;
            mobilenetHeadIndex = static_cast<int>(i);
        }
    }

    DAI_CHECK_V(numYoloHeads > 0 || numMobilenetHeads > 0, "NNArchive should contain at least one detection head (YOLO or Mobilenet-SSD).");
    DAI_CHECK_V(!(numYoloHeads > 0 && numMobilenetHeads > 0),
                "NNArchive should contain only one type of detection head (YOLO or Mobilenet-SSD). Found {} YOLO heads and {} Mobilenet-SSD heads.",
                numYoloHeads,
                numMobilenetHeads);

    int headIndex = (numYoloHeads > 0) ? yoloHeadIndex : mobilenetHeadIndex;

    const auto head = (*model.heads)[headIndex];

    if(head.parser == "YOLO" || head.parser == "YOLOExtendedParser") {
        properties.parser.nnFamily = DetectionNetworkType::YOLO;
        if(head.metadata.subtype) {
            properties.parser.subtype = *head.metadata.subtype;
            properties.parser.decodingFamily = yoloDecodingFamilyResolver(*head.metadata.subtype);
        }

        // check if there are keypoints or segmentations to decode
        if(head.outputs && !head.outputs->empty()) {
            properties.parser.decodeSegmentation = decodeSegmentationResolver(*head.outputs);
        }

        if(head.metadata.nKeypoints) {
            properties.parser.decodeKeypoints = true;
            properties.parser.nKeypoints = head.metadata.nKeypoints;
        }

        const auto keypointNamesIt = head.metadata.extraParams.find("keypoint_label_names");
        if(keypointNamesIt != head.metadata.extraParams.end() && keypointNamesIt->is_array() && !keypointNamesIt->empty()) {
            pimpl->logger->debug("Found keypoint_label_names in extraParams");
            std::vector<std::string> keypointLabelNames;
            for(const auto& labelName : *keypointNamesIt) {
                if(labelName.is_string()) {
                    keypointLabelNames.emplace_back(labelName.get<std::string>());
                } else {
                    throw std::runtime_error("Non-string value found in keypoint_label_names array. keypoint_label_names should be an array of only strings.");
                }
            }
            properties.parser.nKeypoints = static_cast<int>(keypointLabelNames.size());  // prefer keypoint label names size
            properties.parser.decodeKeypoints = true;
            properties.parser.keypointLabelNames = keypointLabelNames;
        }

        if(head.metadata.yoloOutputs) {
            properties.parser.outputNamesToUse = *head.metadata.yoloOutputs;
        }
    } else if(head.parser == "SSD" || head.parser == "MOBILENET") {
        properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
        properties.parser.subtype.clear();
        properties.parser.decodingFamily = YoloDecodingFamily::TLBR;
        properties.parser.decodeSegmentation = false;
        properties.parser.decodeKeypoints = false;
        properties.parser.nKeypoints.reset();
        properties.parser.outputNamesToUse.clear();
        properties.parser.anchors.clear();
        properties.parser.anchorsV2.clear();
        properties.parser.anchorMasks.clear();
        properties.parser.keypointEdges.clear();
    } else {
        DAI_CHECK_V(false, "Unsupported parser: {}", head.parser);
    }

    if(head.metadata.extraParams.contains("skeleton_edges")) {
        pimpl->logger->debug("Found skeleton_edges in extraParams");
        auto skeletonEdgesJson = head.metadata.extraParams["skeleton_edges"];
        if(skeletonEdgesJson.is_array()) {
            std::vector<dai::Edge> skeletonEdges;
            for(const auto& edge : skeletonEdgesJson) {
                if(edge.is_array() && edge.size() == 2) {
                    skeletonEdges.emplace_back(dai::Edge{edge[0].get<uint32_t>(), edge[1].get<uint32_t>()});
                }
            }
            properties.parser.keypointEdges = skeletonEdges;
        }
    }

    if(head.metadata.classes) {
        if(head.metadata.nClasses) {
            DAI_CHECK_V(*head.metadata.nClasses == static_cast<long>(head.metadata.classes->size()),
                        "Number of classes does not match the size of class names array");
        }
        setNumClasses(static_cast<int>(head.metadata.classes->size()));
        setClasses(*head.metadata.classes);
    } else if(head.metadata.nClasses) {
        setNumClasses(static_cast<int>(*head.metadata.nClasses));
    }
    if(head.metadata.iouThreshold) {
        properties.parser.iouThreshold = static_cast<float>(*head.metadata.iouThreshold);
    }
    if(head.metadata.confThreshold) {
        setConfidenceThreshold(static_cast<float>(*head.metadata.confThreshold));
    }

    setCoordinateSize(4);
    if(head.metadata.anchors) {
        const auto anchorsIn = *head.metadata.anchors;
        std::vector<std::vector<std::vector<float>>> anchorsOut(anchorsIn.size());
        for(size_t layer = 0; layer < anchorsOut.size(); ++layer) {
            std::vector<std::vector<float>> layerOut(anchorsIn[layer].size());
            for(size_t anchor = 0; anchor < layerOut.size(); ++anchor) {
                std::vector<float> anchorOut(anchorsIn[layer][anchor].size());
                if(anchorOut.size() != 2) {
                    throw std::runtime_error("Each anchor should have exactly 2 dimensions (width and height).");
                }
                for(size_t dim = 0; dim < anchorOut.size(); ++dim) {
                    anchorOut[dim] = static_cast<float>(anchorsIn[layer][anchor][dim]);
                }
                layerOut[anchor] = anchorOut;
            }
            anchorsOut[layer] = layerOut;
        }
        setAnchors(anchorsOut);
    }
}

YoloDecodingFamily DetectionParser::yoloDecodingFamilyResolver(const std::string& name) {
    // convert string to lower case
    std::string subtypeStr = name;
    std::transform(subtypeStr.begin(), subtypeStr.end(), subtypeStr.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if(subtypeStr == "yolov6r1") return YoloDecodingFamily::R1AF;
    if(subtypeStr == "yolov6r2" || subtypeStr == "yolov8n" || subtypeStr == "yolov6" || subtypeStr == "yolov8" || subtypeStr == "yolov10"
       || subtypeStr == "yolov11")
        return YoloDecodingFamily::TLBR;
    if(subtypeStr == "yolov3" || subtypeStr == "yolov3-tiny") return YoloDecodingFamily::v3AB;
    if(subtypeStr == "yolov5" || subtypeStr == "yolov7" || subtypeStr == "yolo-p" || subtypeStr == "yolov5-u") return YoloDecodingFamily::v5AB;

    pimpl->logger->error("Unknown YOLO subtype '{}', defaulting to TLBR decoding family.", name);
    return YoloDecodingFamily::TLBR;  // default
}

bool DetectionParser::decodeSegmentationResolver(const std::vector<std::string>& outputs) {
    for(const auto& output : outputs) {
        if(output.find("_masks") != std::string::npos) {
            return true;
        }
    }
    return false;
}

void DetectionParser::setNNArchiveBlob(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::BLOB, "NNArchive type is not BLOB");
    setConfig(nnArchive.getVersionedConfig());
    setBlob(nnArchive.getBlob().value());
}

void DetectionParser::setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB, "NNArchive type is not SUPERBLOB");
    setConfig(nnArchive.getVersionedConfig());
    setBlob(nnArchive.getSuperBlob()->getBlobWithNumShaves(numShaves));
}

void DetectionParser::setNNArchiveOther(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void DetectionParser::setBlob(OpenVINO::Blob blob) {
    properties.networkInputs = blob.networkInputs;
}

void DetectionParser::setBlobPath(const std::filesystem::path& path) {
    setBlob(OpenVINO::Blob(path));
}

void DetectionParser::setBlob(const std::filesystem::path& path) {
    setBlobPath(path);
}

void DetectionParser::setInputImageSize(std::tuple<int, int> size) {
    setInputImageSize(std::get<0>(size), std::get<1>(size));
}

void DetectionParser::setInputImageSize(int width, int height) {
    dai::TensorInfo tensorInfo{};
    tensorInfo.dims = std::vector<unsigned int>{static_cast<unsigned int>(width), static_cast<unsigned int>(height)};
    tensorInfo.numDimensions = 2;
    if(properties.networkInputs.size() != 0) {
        pimpl->logger->error("setInputImageSize(...) can only be used if setBlob(...) is not in use. Otherwise input sizes are parsed from the blob.");
        return;
    }
    properties.networkInputs.emplace("input", tensorInfo);
}

void DetectionParser::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

int DetectionParser::getNumFramesPool() {
    return properties.numFramesPool;
}

void DetectionParser::setNNFamily(DetectionNetworkType type) {
    properties.parser.nnFamily = type;
}

DetectionNetworkType DetectionParser::getNNFamily() {
    return properties.parser.nnFamily;
}

void DetectionParser::setConfidenceThreshold(float thresh) {
    properties.parser.confidenceThreshold = thresh;
}

float DetectionParser::getConfidenceThreshold() const {
    return properties.parser.confidenceThreshold;
}

void DetectionParser::setNumClasses(const int numClasses) {
    properties.parser.classes = numClasses;
}

void DetectionParser::setCoordinateSize(const int coordinates) {
    properties.parser.coordinates = coordinates;
}

void DetectionParser::setAnchors(std::vector<float> anchors) {
    properties.parser.anchors = anchors;
}

void DetectionParser::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    properties.parser.anchorMasks = anchorMasks;
}

void DetectionParser::setAnchors(const std::vector<std::vector<std::vector<float>>>& anchors) {
    properties.parser.anchorsV2 = anchors;
}

void DetectionParser::setIouThreshold(float thresh) {
    properties.parser.iouThreshold = thresh;
}

void DetectionParser::setSubtype(const std::string& subtype) {
    properties.parser.subtype = subtype;
    properties.parser.decodingFamily = yoloDecodingFamilyResolver(subtype);
}

void DetectionParser::setDecodeKeypoints(bool decode) {
    if(decode && !properties.parser.nKeypoints.has_value()) {
        throw std::runtime_error("Number of keypoints not set. Please also specify number of keypoints");
    }
    properties.parser.decodeKeypoints = decode;
}

void DetectionParser::setDecodeSegmentation(bool decode) {
    properties.parser.decodeSegmentation = decode;
}

void DetectionParser::setNumKeypoints(int numKeypoints) {
    properties.parser.decodeKeypoints = true;
    properties.parser.nKeypoints = numKeypoints;
}

/// Get num classes
int DetectionParser::getNumClasses() const {
    return properties.parser.classes;
}

std::optional<std::vector<std::string>> DetectionParser::getClasses() const {
    return properties.parser.classNames;
}

void DetectionParser::setClasses(const std::vector<std::string>& classes) {
    properties.parser.classes = static_cast<int>(classes.size());
    properties.parser.classNames = classes;
}

void DetectionParser::setStrides(const std::vector<int>& strides) {
    properties.parser.strides = strides;
}

void DetectionParser::setKeypointEdges(const std::vector<dai::Edge>& edges) {
    properties.parser.keypointEdges = edges;
}

/// Get coordianate size
int DetectionParser::getCoordinateSize() const {
    return properties.parser.coordinates;
}

/// Get anchors
std::vector<float> DetectionParser::getAnchors() const {
    return properties.parser.anchors;
}

/// Get anchor masks
std::map<std::string, std::vector<int>> DetectionParser::getAnchorMasks() const {
    return properties.parser.anchorMasks;
}

/// Get Iou threshold
float DetectionParser::getIouThreshold() const {
    return properties.parser.iouThreshold;
}

std::string DetectionParser::getSubtype() const {
    return properties.parser.subtype;
}

int DetectionParser::getNKeypoints() const {
    if(properties.parser.nKeypoints.has_value()) {
        return properties.parser.nKeypoints.value();
    } else {
        return 0;
    }
}

bool DetectionParser::getDecodeKeypoints() const {
    return properties.parser.decodeKeypoints;
}
bool DetectionParser::getDecodeSegmentation() const {
    return properties.parser.decodeSegmentation;
}

std::vector<int> DetectionParser::getStrides() const {
    return properties.parser.strides;
}

void DetectionParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool DetectionParser::runOnHost() const {
    return runOnHostVar;
}

void DetectionParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->info("Detection parser running on host.");

    using namespace std::chrono;
    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        std::shared_ptr<dai::NNData> sharedInputData;
        {
            auto blockEvent = this->inputBlockEvent();
            sharedInputData = input.get<dai::NNData>();
        }
        auto outDetections = std::make_shared<dai::ImgDetections>();

        if(!sharedInputData) {
            logger->error("NN Data is empty. Skipping processing.");
            continue;
        }
        auto tAfterMessageBeginning = steady_clock::now();
        dai::NNData& inputData = *sharedInputData;

        if(!imgSizesSet) {
            const bool containsTransformation = inputData.transformation.has_value();
            if(containsTransformation) {
                std::tie(imgWidth, imgHeight) = inputData.transformation->getSize();
            } else {
                logger->warn("No image size provided for detection parser. Skipping processing and sending empty detections.");
                continue;
            }
            // We have determined the image size, no need to try again in the future
            imgSizesSet = true;
        }

        // Parse detections
        switch(properties.parser.nnFamily) {
            case DetectionNetworkType::YOLO: {
                decodeYolo(inputData, *outDetections);
                break;
            }
            case DetectionNetworkType::MOBILENET: {
                decodeMobilenet(inputData, *outDetections, properties.parser.confidenceThreshold);
                break;
            }
            default: {
                logger->error("Unknown NN family. 'YOLO' and 'MOBILENET' are supported.");
                break;
            }
        }

        auto tBeforeSend = steady_clock::now();

        // Copy over seq and ts
        outDetections->setSequenceNum(inputData.getSequenceNum());
        outDetections->setTimestamp(inputData.getTimestamp());
        outDetections->setTimestampDevice(inputData.getTimestampDevice());
        outDetections->transformation = inputData.transformation;

        {
            auto blockEvent = this->outputBlockEvent();
            // Send detections
            out.send(outDetections);
        }

        auto tAbsoluteEnd = steady_clock::now();
        logger->debug("Detection parser total took {}ms, processing {}ms, getting_frames {}ms, sending_frames {}ms",
                      duration_cast<microseconds>(tAbsoluteEnd - tAbsoluteBeginning).count() / 1000,
                      duration_cast<microseconds>(tBeforeSend - tAfterMessageBeginning).count() / 1000,
                      duration_cast<microseconds>(tAfterMessageBeginning - tAbsoluteBeginning).count() / 1000,
                      duration_cast<microseconds>(tAbsoluteEnd - tBeforeSend).count() / 1000);
    }
}

void DetectionParser::buildStage1() {
    auto& logger = ThreadedNode::pimpl->logger;

    // Grab dimensions from input tensor info
    if(properties.networkInputs.size() > 0) {
        if(properties.networkInputs.size() > 1) {
            logger->warn("Detection parser supports only single input networks, assuming first input");
        }
        for(const auto& kv : properties.networkInputs) {
            const dai::TensorInfo& tensorInfo = kv.second;
            inTensorInfo.push_back(tensorInfo);
        }
    }
    if(inTensorInfo.size() > 0) {
        int numDimensions = inTensorInfo[0].numDimensions;
        if(numDimensions < 2) {
            logger->error("Number of input dimensions is less than 2");
        } else {
            imgSizesSet = true;
            imgWidth = inTensorInfo[0].dims[numDimensions - 1];
            imgHeight = inTensorInfo[0].dims[numDimensions - 2];
        }
    } else {
        logger->info("Unable to read input tensor height and width from static inputs. The node will try to get input sizes at runtime.");
    }
}

void DetectionParser::decodeMobilenet(dai::NNData& nnData, dai::ImgDetections& outDetections, float confidenceThr) {
    auto& logger = ThreadedNode::pimpl->logger;

    int maxDetections = 100;
    std::vector<dai::ImgDetection> detections;
    std::string tensorName;
    for(const auto& tensor : nnData.getAllLayers()) {
        if(tensor.offset == 0) {
            // // The tensor we want to checkout
            // if(tensor.numDimensions != 4) {
            //     std::cout << "ERROR while decoding Mobilenet. Output tensor has incorrect dimensions. Number of dimensions: " << tensor.numDimensions
            //               << std::endl;
            // }
            // // Get tensor output size in Bytes
            // // Expected dimensions are [1, 1, N, 7] where N is number of detections
            // if(tensor.dims[3] != 7) {
            //     std::cout << "ERROR while decoding Mobilenet. Expecting 7 fields for every detection but: " << tensor.dims[3] << " found.\n";
            // }
            // maxDetections = tensor.dims[tensor.numDimensions - 2];
            tensorName = tensor.name;
        }
    }

    auto tensorData = nnData.getTensor<float>(tensorName);
    maxDetections = tensorData.size() / 7;
    if(static_cast<int>(tensorData.size()) < maxDetections * 7) {
        logger->error("Error while parsing Mobilenet. Vector not long enough, expected size: {}, real size {}", maxDetections * 7, tensorData.size());
        return;
    }

    struct raw_Detection {  // need to update it to include more
        float header;
        float label;
        float confidence;
        float xmin;
        float ymin;
        float xmax;
        float ymax;
    };

    float* rawPtr = tensorData.data();
    for(int i = 0; i < maxDetections; i++) {
        raw_Detection temp;
        // TODO This is likely unnecessary optimisation
        memcpy(&temp, &rawPtr[i * 7], sizeof(raw_Detection));

        // if header == -1, stop sooner
        if(temp.header == -1.0f) break;

        float currentConfidence = temp.confidence;
        if(currentConfidence >= confidenceThr) {
            dai::ImgDetection d;
            d.label = temp.label;

            d.confidence = currentConfidence;

            d.xmin = temp.xmin;
            d.ymin = temp.ymin;
            d.xmax = temp.xmax;
            d.ymax = temp.ymax;

            outDetections.detections.push_back(d);
        }
    }
}

void DetectionParser::decodeYolo(dai::NNData& nnData, dai::ImgDetections& outDetections) {
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    switch(properties.parser.decodingFamily) {
        case YoloDecodingFamily::R1AF:  // anchor free: yolo v6r1
            utilities::DetectionParserUtils::decodeR1AF(nnData, outDetections, properties, logger);
            break;
        case YoloDecodingFamily::v3AB:  // anchor based yolo v3 v3-Tiny
            utilities::DetectionParserUtils::decodeV3AB(nnData, outDetections, properties, logger);
            break;
        case YoloDecodingFamily::v5AB:  // anchor based yolo v5, v7, P
            utilities::DetectionParserUtils::decodeV5AB(nnData, outDetections, properties, logger);
            break;
        case YoloDecodingFamily::TLBR:  // top left bottom right anchor free: yolo v6r2, v8 v10 v11
            utilities::DetectionParserUtils::decodeTLBR(nnData, outDetections, properties, logger);
            break;
        default:
            logger->error("Unknown Yolo decoding family. 'R1AF', 'v3AB', 'v5AB' and 'TLBR' are supported.");
            throw std::runtime_error("Unknown Yolo decoding family");
    }
}

}  // namespace node
}  // namespace dai
