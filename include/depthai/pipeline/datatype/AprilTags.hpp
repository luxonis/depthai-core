#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/common/Point2f.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

namespace dai {

/**
 * AprilTag structure.
 */
struct AprilTag {
    /**
     * The decoded ID of the tag
     */
    int id = 0;

    /**
     * How many error bits were corrected? Note: accepting large numbers of
     * corrected errors leads to greatly increased false positive rates.
     * As of this implementation, the detector cannot detect tags with
     * a hamming distance greater than 2.
     */
    int hamming = 0;

    /**
     * A measure of the quality of the binary decoding process; the
     * average difference between the intensity of a data bit versus
     * the decision threshold. Higher numbers roughly indicate better
     * decodes. This is a reasonable measure of detection accuracy
     * only for very small tags-- not effective for larger tags (where
     * we could have sampled anywhere within a bit cell and still
     * gotten a good detection.
     */
    float decisionMargin = 0.f;

    /**
     * The detected top left coordinates.
     */
    Point2f topLeft;

    /**
     * The detected top right coordinates.
     */
    Point2f topRight;

    /**
     * The detected bottom right coordinates.
     */
    Point2f bottomRight;

    /**
     * The detected bottom left coordinates.
     */
    Point2f bottomLeft;
};
DEPTHAI_SERIALIZE_EXT(AprilTag, id, hamming, decisionMargin, topLeft, topRight, bottomRight, bottomLeft);

/**
 * AprilTags message.
 */
class AprilTags : public Buffer, public TransformableCRTP<AprilTags> {
   protected:
    /**
     * Internal transform hook used by transformTo() to apply AprilTags-specific transformation logic.
     */
    void transformToInternal(const ImgTransformation& target) override;

   public:
    friend class TransformableCRTP<AprilTags>;
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Transformable::transformation;
    /**
     * Construct AprilTags message.
     */
    AprilTags() = default;

    ~AprilTags() override;

    /**
     * Returns a new AprilTags message with the tags transformed into the target image transformation.
     *
     * If the target transformation has a different coordinate system source (eg. different camera socket) then the remapping will be inaccurate due to
     * the lack of depth information.
     *
     * @param target Target image transformation.
     */
    AprilTags transformTo(const ImgTransformation& target);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::AprilTags;
    }

    std::vector<AprilTag> aprilTags;
    DEPTHAI_SERIALIZE(AprilTags, sequenceNum, ts, tsDevice, transformation, aprilTags);
};

}  // namespace dai
