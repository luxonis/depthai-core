#pragma once

#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

struct ImgDetection {
    uint32_t label = 0;
    float confidence = 0.f;
    float xmin = 0.f;
    float ymin = 0.f;
    float xmax = 0.f;
    float ymax = 0.f;
};

DEPTHAI_SERIALIZE_EXT(ImgDetection, label, confidence, xmin, ymin, xmax, ymax);

/**
 * ImgDetections message. Carries normalized detection results
 */
class ImgDetections : public Buffer {
   public:
    /**
     * Construct ImgDetections message.
     */
    ImgDetections() = default;
    virtual ~ImgDetections() = default;

    /// Detections
    std::vector<ImgDetection> detections;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImgDetections;
    };

    DEPTHAI_SERIALIZE(ImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections);
};

}  // namespace dai
