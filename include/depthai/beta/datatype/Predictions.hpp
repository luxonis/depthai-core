#pragma once

#include <cstdint>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace beta {

/**
 * Single predicted value. Serialized value type contained by the Predictions message.
 */
struct Prediction {
    /**
     * The predicted value.
     */
    float prediction = 0.0f;

    DEPTHAI_SERIALIZE(Prediction, prediction);
};

/**
 * Predictions message. Carries the predicted value(s) of a regression model in the order the
 * model emitted them.
 *
 * The message may carry no predictions when the parsed tensor is empty.
 */
class Predictions : public Buffer, public TransformableCRTP<Predictions> {
   protected:
    /**
     * Internal transform hook used by transformTo().
     *
     * Regression results carry no spatial data, so only the transformation metadata is
     * replaced with the target transformation.
     */
    void transformToInternal(const ImgTransformation& target) override;

   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Buffer::tsSystem;
    using Transformable::transformation;

    friend class TransformableCRTP<Predictions>;

    /**
     * Construct Predictions message.
     */
    Predictions() = default;
    ~Predictions() override;

    /**
     * Predicted values, in the order the model emitted them.
     */
    std::vector<Prediction> predictions;

    /**
     * Returns the first predicted value. Useful for single-prediction models.
     *
     * @throws std::runtime_error if the message contains no predictions.
     */
    float getFirstPrediction() const;

    /**
     * Returns a new Predictions message with the transformation metadata replaced by the
     * target transformation. Regression results carry no spatial data, so the predictions
     * are unchanged.
     *
     * @param target Target image transformation.
     */
    Predictions transformTo(const ImgTransformation& target) const;

    /**
     * Returns an ImgAnnotations visualization with each predicted value drawn as text, one
     * below the other, or std::monostate when no transformation metadata is available to
     * derive the annotation layout from.
     */
    dai::VisualizeType getVisualizationMessage() const override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Predictions;
    }

    DEPTHAI_SERIALIZE(Predictions, sequenceNum, ts, tsDevice, tsSystem, transformation, predictions);
};

}  // namespace beta
}  // namespace dai
