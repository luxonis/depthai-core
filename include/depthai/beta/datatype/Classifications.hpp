#pragma once

#include <string>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

namespace dai {
namespace beta {

/**
 * Classifications message. Carries classification class names and their corresponding scores.
 *
 * The classes and scores vectors are index-aligned. Parsers emit them sorted in descending
 * order of score, so the first entry is the most probable class.
 */
class Classifications : public Buffer, public TransformableCRTP<Classifications> {
   protected:
    /**
     * Internal transform hook used by transformTo().
     *
     * Classification results carry no spatial data, so only the transformation metadata is
     * replaced with the target transformation.
     */
    void transformToInternal(const ImgTransformation& target) override;

   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Buffer::tsSystem;
    using Transformable::transformation;

    friend class TransformableCRTP<Classifications>;

    /**
     * Construct Classifications message.
     */
    Classifications() = default;
    ~Classifications() override;

    /**
     * Class names, index-aligned with the scores vector.
     */
    std::vector<std::string> classes;

    /**
     * Classification scores, index-aligned with the classes vector.
     */
    std::vector<float> scores;

    /**
     * Returns the most probable class name.
     *
     * Assumes the classes are sorted in descending order of score, which holds for
     * parser-emitted messages.
     *
     * @throws std::runtime_error if the message contains no classes.
     */
    std::string getTopClass() const;

    /**
     * Returns the score of the most probable class.
     *
     * Assumes the scores are sorted in descending order, which holds for parser-emitted
     * messages.
     *
     * @throws std::runtime_error if the message contains no scores.
     */
    float getTopScore() const;

    /**
     * Returns a new Classifications message with the transformation metadata replaced by the
     * target transformation. Classification results carry no spatial data, so classes and
     * scores are unchanged.
     *
     * @param target Target image transformation.
     */
    Classifications transformTo(const ImgTransformation& target) const;

    /**
     * Returns an ImgAnnotations visualization with up to the top five classes and their
     * scores, or std::monostate when no transformation metadata is available to derive the
     * annotation layout from.
     */
    dai::VisualizeType getVisualizationMessage() const override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Classifications;
    }

    DEPTHAI_SERIALIZE(Classifications, sequenceNum, ts, tsDevice, tsSystem, transformation, classes, scores);
};

}  // namespace beta
}  // namespace dai
