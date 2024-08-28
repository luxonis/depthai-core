#pragma once

#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/schemas/ImgDetections.pb.h"
#include "depthai/utility/ProtoSerializable.hpp"

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

    std::unique_ptr<google::protobuf::Message> getProtoMessage() const override {
        auto imgDetections = std::make_unique<proto::ImgDetections>();

        imgDetections->set_sequencenum(this->sequenceNum);
        proto::Timestamp* ts = imgDetections->mutable_ts();
        ts->set_sec(this->ts.sec);
        ts->set_nsec(this->ts.nsec);
        proto::Timestamp* tsDevice = imgDetections->mutable_tsdevice();
        tsDevice->set_sec(this->tsDevice.sec);
        tsDevice->set_nsec(this->tsDevice.nsec);

        for(const auto& detection : this->detections) {
            proto::ImgDetection* imgDetection = imgDetections->add_detections();
            imgDetection->set_label(detection.label);
            imgDetection->set_confidence(detection.confidence);
            imgDetection->set_xmin(detection.xmin);
            imgDetection->set_ymin(detection.ymin);
            imgDetection->set_xmax(detection.xmax);
            imgDetection->set_ymax(detection.ymax);
        }
        return imgDetections;
    }

    DEPTHAI_SERIALIZE(ImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections);
};

}  // namespace dai
