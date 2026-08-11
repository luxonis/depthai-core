#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for FastSAMParser.
 */
class FastSAMParserConfig : public Buffer {
   public:
    /** Prompt mode used to select emitted segmentation masks. */
    enum class Prompt : std::int32_t {
        EVERYTHING,   ///< Keep all detected instances.
        POINT,        ///< Select instances using a point and point label.
        BOUNDING_BOX  ///< Select an instance using a bounding box.
    };

    float confidenceThreshold = 0.5f;

    float iouThreshold = 0.5f;

    float maskConfidence = 0.5f;

    Prompt prompt = Prompt::EVERYTHING;

    std::optional<std::pair<std::int32_t, std::int32_t>> points;

    std::optional<std::int32_t> pointLabel;

    std::optional<std::array<std::int32_t, 4>> boundingBox;

    FastSAMParserConfig() = default;

    ~FastSAMParserConfig() override;

    /**
     * Sets the minimum detection confidence.
     * @param threshold Confidence threshold in the range [0, 1]
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Gets the minimum detection confidence.
     * @return Confidence threshold
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the intersection-over-union threshold used by non-maximum suppression.
     * @param threshold IoU threshold in the range [0, 1]
     */
    void setIouThreshold(float threshold);

    /**
     * Gets the intersection-over-union threshold.
     * @return IoU threshold
     */
    float getIouThreshold() const;

    /**
     * Sets the threshold used to binarize instance masks.
     * @param threshold Mask confidence threshold in the range [0, 1]
     */
    void setMaskConfidence(float threshold);

    /**
     * Gets the threshold used to binarize instance masks.
     * @return Mask confidence threshold
     */
    float getMaskConfidence() const;

    /**
     * Sets the prompt mode. Required point or bounding-box data must already be present.
     * @param prompt Prompt mode
     */
    void setPrompt(Prompt prompt);

    /**
     * Gets the prompt mode.
     * @return Prompt mode
     */
    Prompt getPrompt() const;

    /**
     * Sets the prompt point.
     * @param x Point x coordinate
     * @param y Point y coordinate
     */
    void setPoints(std::int32_t x, std::int32_t y);

    /**
     * Gets the prompt point.
     * @return Optional prompt point as (x, y)
     */
    std::optional<std::pair<std::int32_t, std::int32_t>> getPoints() const;

    /**
     * Sets the prompt point label.
     * @param label Point label, 0 for negative or 1 for positive
     */
    void setPointLabel(std::int32_t label);

    /**
     * Gets the prompt point label.
     * @return Optional point label
     */
    std::optional<std::int32_t> getPointLabel() const;

    /**
     * Sets the prompt bounding box.
     * @param boundingBox Bounding box as {x1, y1, x2, y2}, with ordered coordinates and positive x2/y2
     */
    void setBoundingBox(const std::array<std::int32_t, 4>& boundingBox);

    /**
     * Gets the prompt bounding box.
     * @return Optional bounding box as {x1, y1, x2, y2}
     */
    std::optional<std::array<std::int32_t, 4>> getBoundingBox() const;

    /**
     * Validates thresholds, prompt payload, point label, and bounding-box coordinates.
     * @return True if the complete configuration is valid
     */
    bool validate() const;

    /**
     * Serializes this configuration into message metadata.
     * @param metadata Destination metadata buffer
     * @param datatype Datatype identifier written during serialization
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Returns the datatype identifier for this configuration.
     * @return Configuration datatype identifier
     */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::FastSAMParserConfig;
    }

    DEPTHAI_SERIALIZE(FastSAMParserConfig, confidenceThreshold, iouThreshold, maskConfidence, prompt, points, pointLabel, boundingBox);
};

}  // namespace beta
}  // namespace dai
