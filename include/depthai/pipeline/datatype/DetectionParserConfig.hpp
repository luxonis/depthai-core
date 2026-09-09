#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Runtime thresholds for DetectionParser. Send to inputConfig to update subsequent detections.
 * Model layout, labels and decoding family remain configured on the node.
 */
class DetectionParserConfig : public Buffer {
   public:
    /// Minimum detection confidence. Default matches DetectionParser's startup threshold.
    float confidenceThreshold = 0.0f;
    /// IoU threshold for YOLO non-maximum suppression. Not used by Mobilenet-SSD or end-to-end YOLO models.
    float iouThreshold = 0.0f;

    /** Construct a detection parser configuration message. */
    DetectionParserConfig() = default;
    ~DetectionParserConfig() override;

    /** Set the minimum detection confidence. */
    void setConfidenceThreshold(float threshold);
    /** Get the minimum detection confidence. */
    float getConfidenceThreshold() const;
    /** Set the IoU threshold for non-maximum suppression. */
    void setIouThreshold(float threshold);
    /** Get the IoU threshold for non-maximum suppression. */
    float getIouThreshold() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::DetectionParserConfig;
    }

    DEPTHAI_SERIALIZE(DetectionParserConfig, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, confidenceThreshold, iouThreshold);
};

}  // namespace dai
