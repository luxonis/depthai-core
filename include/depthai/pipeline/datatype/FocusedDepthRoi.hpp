#pragma once

#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Region-of-interest message for FocusedDepth.
 *
 * Coordinates are relative to the left stereo frame (rectified-left perspective) unless
 * ``referenceFrame`` is extended later; axis-aligned rectangle only for v1.
 */
class FocusedDepthRoi : public Buffer {
   public:
    enum class ReferenceFrame : std::uint32_t { LEFT_RECTIFIED = 0 };

    Rect roi{};
    bool normalizedCoords = false;
    ReferenceFrame referenceFrame = ReferenceFrame::LEFT_RECTIFIED;
    /// Optional hint for application/debugging; not used by the node for pairing.
    std::uint32_t sequenceHint = 0;

    FocusedDepthRoi() = default;
    ~FocusedDepthRoi() override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::FocusedDepthRoi;
    }

    DEPTHAI_SERIALIZE_EXT(FocusedDepthRoi, roi, normalizedCoords, referenceFrame, sequenceHint);
};

}  // namespace dai
