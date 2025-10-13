#include "depthai/pipeline/node/DetectionParser.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <memory>
#include <vector>

#include "common/DetectionNetworkType.hpp"
#include "common/ModelType.hpp"
#include "common/YoloDecodingFamily.hpp"
#include "common/YoloSubtype.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "nn_archive/NNArchive.hpp"
#include "nn_archive/v1/Head.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "spdlog/fmt/fmt.h"

// internal headers
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

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
    nnInput.link(input);
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
    for(size_t i = 0; i < modelHeads.size(); i++) {
        if(modelHeads[i].parser == "YOLO" || modelHeads[i].parser == "YOLOExtendedParser") {
            yoloHeadIndex = static_cast<int>(i);
            numYoloHeads++;
        }
    }
    std::cout << fmt::format("Found {} YOLO heads in NNArchive\n", numYoloHeads);

    DAI_CHECK_V(numYoloHeads == 1, "NNArchive should contain exactly one YOLO head. Found {} YOLO heads.", numYoloHeads);  // no support for multi-head YOLO
    const auto head = (*model.heads)[yoloHeadIndex];

    if(head.parser == "YOLO" || head.parser == "YOLOExtendedParser") {
        properties.parser.nnFamily = DetectionNetworkType::YOLO;
        if(head.metadata.subtype) {
            properties.parser.subtype = *head.metadata.subtype;
            properties.parser.decodingFamily = yoloDecodingFamilyResolver(*head.metadata.subtype);
        }

        // check if there are keypoints or segmentations to decode
        if(head.outputs && !head.outputs->empty()) {
            // assert outputs is a non-empty vector
            properties.parser.decodeSegmentation = decodeSegmentationResolver(*head.outputs);
        }

        if(head.metadata.nKeypoints) {
            properties.parser.decodeKeypoints = true;
            properties.parser.nKeypoints = head.metadata.nKeypoints;
        }

        if(properties.parser.decodingFamily == YoloDecodingFamily::v3AB) {
            properties.parser.strides = {16, 32};
        }

        if(head.metadata.yoloOutputs) {
            properties.parser.outputNames = *head.metadata.yoloOutputs;
        }

    } else if(head.parser == "SSD" || head.parser == "MOBILENET") {
        properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
    } else {
        DAI_CHECK_V(false, "Unsupported parser: {}", head.parser);
    }

    if(head.metadata.classes) {
        setClasses(*head.metadata.classes);
    }
    if(head.metadata.nClasses) {
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

void DetectionParser::setSubtype(const std::string& subtype) {  // TODO add bindings
    properties.parser.subtype = subtype;
    properties.parser.decodingFamily = yoloDecodingFamilyResolver(subtype);
}

void DetectionParser::setDecodeKeypoints(bool decode) {
    properties.parser.decodeKeypoints = decode;
}

void DetectionParser::setDecodeSegmentation(bool decode) {
    properties.parser.decodeSegmentation = decode;
}

void DetectionParser::setNKeypoints(int nKeypoints) {
    properties.parser.nKeypoints = nKeypoints;
}

/// Get num classes
int DetectionParser::getNumClasses() const {
    return properties.parser.classes;
}

std::optional<std::vector<std::string>> DetectionParser::getClasses() const {
    return properties.parser.classNames;
}

void DetectionParser::setClasses(const std::vector<std::string>& classes) {
    properties.parser.classNames = classes;
}

void DetectionParser::setStrides(const std::vector<int>& strides) {
    properties.parser.strides = strides;
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

}  // namespace node
}  // namespace dai
