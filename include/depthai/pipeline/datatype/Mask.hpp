#pragma once

#include <stdexcept>

#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {

/**
 * Depth validity mask in the final depth image coordinates and size.
 * Zero pixels invalidate depth; nonzero pixels retain depth.
 * Preserves its Mask datatype when sent between host and device.
 */
class Mask : public ImgFrame {
   public:
    Mask() {
        setType(Type::RAW8);
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Mask;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        ImgFrame::serialize(metadata, datatype);
        datatype = DatatypeEnum::Mask;
    }

    bool empty() const {
        return getData().empty();
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /** Copies a nonempty, single-channel 8-bit mask, including its dimensions. */
    void update(const cv::Mat& mask) {
        if(mask.empty() || mask.type() != CV_8UC1) {
            throw std::invalid_argument("Mask requires a nonempty RAW8/GRAY8 image");
        }
        // clone() also handles continuous ROIs whose allocation extends beyond the mask.
        setFrame(mask.clone());
        setSize(mask.cols, mask.rows);
        setType(Type::RAW8);
        fb.stride = mask.cols;
    }

    /** Zeros invalid pixels in place. An empty mask leaves depth unchanged. */
    void apply(cv::Mat depth) {
        if(empty()) return;
        if(getType() != Type::RAW8 && getType() != Type::GRAY8) {
            throw std::invalid_argument("Mask requires a RAW8/GRAY8 image");
        }
        if(depth.cols != static_cast<int>(getWidth()) || depth.rows != static_cast<int>(getHeight())) {
            throw std::invalid_argument("Mask dimensions must match the final depth image");
        }
        depth.setTo(0, getFrame(false) == 0);
    }
#endif
};

}  // namespace dai
