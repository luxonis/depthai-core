#pragma once

#include <optional>

#include "depthai/common/DetectionNetworkType.hpp"
#include "depthai/common/YoloDecodingFamily.hpp"
#include "depthai/common/YoloSubtype.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * DetectionParserOptions
 *
 * Specifies how to parse output of detection networks
 */
struct DetectionParserOptions {
    /// Generic Neural Network properties
    DetectionNetworkType nnFamily;
    std::string subtype;
    float confidenceThreshold;
    int inputWidth;
    int inputHeight;

    /// YOLO specific network properties
    YoloDecodingFamily decodingFamily = YoloDecodingFamily::TLBR;  // top left bottom right anchor free
    bool decodeKeypoints = false;
    bool decodeSegmentation = false;

    int classes;
    std::optional<std::vector<std::string>> classNames;
    int coordinates;
    std::optional<int> nKeypoints;
    std::vector<int> strides = {8, 16, 32};
    std::vector<float> anchors;
    std::map<std::string, std::vector<int>> anchorMasks;
    /// see YoloDetectionNetwork::setAnchors() for format
    std::vector<std::vector<std::vector<float>>> anchorsV2;
    float iouThreshold;
};

DEPTHAI_SERIALIZE_EXT(DetectionParserOptions,
                      nnFamily,
                      subtype,
                      confidenceThreshold,
                      inputWidth,
                      inputHeight,
                      decodingFamily,
                      decodeKeypoints,
                      decodeSegmentation,
                      classes,
                      classNames,
                      coordinates,
                      nKeypoints,
                      strides,
                      anchors,
                      anchorMasks,
                      anchorsV2,
                      iouThreshold);

}  // namespace dai
