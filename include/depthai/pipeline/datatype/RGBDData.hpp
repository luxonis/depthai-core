#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {

/**
 * RGBD message. Carries RGB and Depth frames.
 */
class RGBDData : public Buffer {
   public:
    /**
     * Construct RGBD message.
     */
    RGBDData() = default;

    virtual ~RGBDData() = default;

    ImgFrame rgbFrame;
    ImgFrame depthFrame;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::RGBDData;
    };
    DEPTHAI_SERIALIZE(RGBDData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, rgbFrame, depthFrame);
};

}  // namespace dai
