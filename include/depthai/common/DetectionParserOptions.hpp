#pragma once

#include <optional>

#include "depthai/common/DetectionNetworkType.hpp"
#include "depthai/common/KeypointsListT.hpp"
#include "depthai/common/YoloDecodingFamily.hpp"
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
    std::vector<std::string> outputNamesToUse;
    /// see YoloDetectionNetwork::setAnchors() for format
    std::vector<std::vector<std::vector<float>>> anchorsV2;
    float iouThreshold;
    std::vector<dai::Edge> keypointEdges = {};
    std::vector<std::string> keypointLabelNames = {};
};

DEPTHAI_SERIALIZE_EXT(DetectionParserOptions,
                      nnFamily,
                      subtype,
                      confidenceThreshold,
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
                      outputNamesToUse,
                      anchorsV2,
                      iouThreshold,
                      keypointEdges,
                      keypointLabelNames);

}  // namespace dai
