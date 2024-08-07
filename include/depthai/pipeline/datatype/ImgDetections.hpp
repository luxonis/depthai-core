#pragma once

#include <vector>
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/protos/ImgDetections.pb.h"

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
class ImgDetections : public Buffer, public utility::ProtoSerializable {
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

    google::protobuf::Message getProtoMessage() const override {
        proto::ImgDetectionsProto imgDets;

        imgDets.set_sequenceNum(this->sequenceNum);
        
        proto::Timestamp ts;
        ts.set_sec(this->ts.sec);
        ts.set_ns(this->ts.sec);
        imgDets->set_ts(ts);
        
        proto::Timestamp tsDevice;
        tsDevice.set_sec(this->tsDevice.sec);
        tsDevice.set_ns(this->tsDevice.nsec);
        imgDets->set_tsDevice(tsDevice);

        for(const auto& detection : this->detections) {
            proto::ImgDetection det = imgDets.add_detections();
            det->set_label(detection.label);
            det->set_confidence(detection.confidence);
            det->set_xmin(detection.xmin);
            det->set_ymin(detection.ymin);
            det->set_xmax(detection.xmax);
            det->set_ymax(detection.ymax);
        }
        
        return imgDets;
    }

    DEPTHAI_SERIALIZE(ImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections);
};

}  // namespace dai
