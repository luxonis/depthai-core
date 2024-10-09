#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for KeypointsParser.
 */
struct KeypointsParserProperties : PropertiesSerializable<Properties, KeypointsParserProperties> {
    /**
     * The scale factor to divide the keypoints by.
     */
    float scaleFactor = 1;

    /**
     * The number of keypoints. Default value is invalid, must be specified.
     */
    int numKeypoints = -1;
};

DEPTHAI_SERIALIZE_EXT(KeypointsParserProperties, scaleFactor, numKeypoints);

}  // namespace dai
