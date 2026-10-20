#pragma once

#include <cstdint>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace beta {

/**
 * Detected line segment. Serialized value type contained by the Lines message.
 */
struct Line {
    /**
     * Start point of the line with x and y coordinate.
     */
    Point2f startPoint;

    /**
     * End point of the line with x and y coordinate.
     */
    Point2f endPoint;

    /**
     * Confidence of the line, in [0, 1].
     */
    float confidence = 0.0f;

    DEPTHAI_SERIALIZE(Line, startPoint, endPoint, confidence);
};

/**
 * Lines message. Carries detected line segments, each with a start point, an end point and a
 * confidence score.
 *
 * Parsers emit line point image coordinates normalized to [0, 1] and confidences clipped to
 * [0, 1]. The message may carry no lines when nothing passes the detection thresholds.
 */
class Lines : public Buffer, public TransformableCRTP<Lines> {
   protected:
    /**
     * Internal transform hook used by transformTo().
     *
     * Remaps the start and end point image coordinates of every line from the source
     * transformation carried by this message into the target transformation.
     */
    void transformToInternal(const ImgTransformation& target) override;

   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Buffer::tsSystem;
    using Transformable::transformation;

    friend class TransformableCRTP<Lines>;

    /**
     * Construct Lines message.
     */
    Lines() = default;
    ~Lines() override;

    /**
     * Detected lines.
     */
    std::vector<Line> lines;

    /**
     * Returns a new Lines message with the line point image coordinates remapped from this
     * message's transformation into the target transformation.
     *
     * @param target Target image transformation.
     * @throws std::runtime_error if this message carries no transformation metadata.
     */
    Lines transformTo(const ImgTransformation& target) const;

    /**
     * Returns an ImgAnnotations visualization with each line drawn as a two-point line strip.
     */
    dai::VisualizeType getVisualizationMessage() const override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Lines;
    }

    DEPTHAI_SERIALIZE(Lines, sequenceNum, ts, tsDevice, tsSystem, transformation, lines);
};

}  // namespace beta
}  // namespace dai
