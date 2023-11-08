#pragma once

#include "depthai-shared/common/Point3f.hpp"
#include "depthai-shared/common/Timestamp.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// ImgDetection structure
struct ImgDetection {
    uint32_t label = 0;
    float confidence = 0.f;
    float xmin = 0.f;
    float ymin = 0.f;
    float xmax = 0.f;
    float ymax = 0.f;
};

DEPTHAI_SERIALIZE_EXT(ImgDetection, label, confidence, xmin, ymin, xmax, ymax);

/// RawImgDetections structure
struct RawImgDetections : public RawBuffer {
    std::vector<ImgDetection> detections;

    // Related to input ImgFrame
    int64_t sequenceNum = 0;  // increments for each frame
    Timestamp ts = {};        // generation timestamp, synced to host time
    Timestamp tsDevice = {};  // generation timestamp, direct device monotonic clock

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImgDetections;
    };

    DEPTHAI_SERIALIZE(RawImgDetections, detections, sequenceNum, ts, tsDevice);
};

}  // namespace dai
