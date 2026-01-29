#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for SegmentationParser
 * @ingroup properties
 * @property labels Vector of class labels associated with the segmentation mask. The label at index $i$ in the `labels` vector corresponds to the value $i$-th
 channel in the segmentation mask data array.
 * @param networkOutputName Name of the output tensor from the neural network to parse. If empty, the first output will be used.
 * @param classesInOneLayer If true, assumes that the segmentation classes are already encoded in a single layer as integer values. If false, an argmax
 * operaiton is perfromed across multiple channels.
 * @param backgroundClass If true, the first class (index 0) is considered as background.
 */
struct SegmentationParserProperties : PropertiesSerializable<Properties, SegmentationParserProperties> {
    std::vector<std::string> labels;
    std::string networkOutputName;
    bool classesInOneLayer = false;
    bool backgroundClass = false;

    ~SegmentationParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(SegmentationParserProperties, labels, networkOutputName, classesInOneLayer, backgroundClass);

}  // namespace dai
