#pragma once

#include "depthai/common/DetectionNetworkType.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/common/optional.hpp"

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
    int classes;
    std::optional<std::vector<std::string>> classNames;
    int coordinates;
    std::vector<float> anchors;
    std::map<std::string, std::vector<int>> anchorMasks;
    /// see YoloDetectionNetwork::setAnchors() for format
    std::vector<std::vector<std::vector<float>>> anchorsV2;
    float iouThreshold;
};

DEPTHAI_SERIALIZE_EXT(DetectionParserOptions, nnFamily, subtype, confidenceThreshold, classes, classNames, coordinates, anchors, anchorMasks, anchorsV2, iouThreshold);

}  // namespace dai
