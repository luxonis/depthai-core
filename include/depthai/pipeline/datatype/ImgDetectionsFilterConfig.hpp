#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/** Configuration shared by all inputs of ImgDetectionsFilter. Runtime messages replace the full configuration. */
class ImgDetectionsFilterConfig : public Buffer {
   public:
    /** Duplicate handling between different inputs with the same label. */
    enum class OverlapMode { OFF, NMS, AVERAGE };

    /** Only these labels pass; an empty list keeps nothing. */
    std::optional<std::vector<std::uint32_t>> labelsToKeep = std::nullopt;
    /** These labels are rejected, including when labelsToKeep is set. */
    std::optional<std::vector<std::uint32_t>> labelsToReject = std::nullopt;
    /** Inclusive confidence range. Default endpoints impose no limit. */
    float minConfidence = 0.0f, maxConfidence = 1.0f;
    /** Inclusive area range in output pixels squared. */
    float minArea = 0.0f, maxArea = std::numeric_limits<float>::max();
    /** Inclusive width range in output pixels, measured in standard box form. */
    float minWidth = 0.0f, maxWidth = std::numeric_limits<float>::max();
    /** Inclusive height range in output pixels, measured in standard box form. */
    float minHeight = 0.0f, maxHeight = std::numeric_limits<float>::max();
    /** All four box corners must lie inside this rectangle in output pixels. */
    std::optional<Rect> regionOfInterest = std::nullopt;
    /** Keep the highest confidence detections without changing their order. Zero keeps none. */
    std::optional<std::uint32_t> maxDetections = std::nullopt;
    /** Sort by descending confidence, preserving ties. */
    bool sortByConfidence = false;
    /** Duplicate handling; has no effect with one input. */
    OverlapMode overlapMode = OverlapMode::NMS;
    /** Rotated rectangle IoU must be strictly greater than this threshold. */
    float overlapIouThreshold = 0.4f;
    /** Latched reference, superseded permanently by the first valid inputReference frame. */
    std::optional<ImgTransformation> reference = std::nullopt;

    ~ImgDetectionsFilterConfig() override;
    /** Serialize this config and its Buffer timestamps. */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    /** Get the transport datatype. */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ImgDetectionsFilterConfig;
    }

    /** Set the inclusive confidence range. Validation occurs at pipeline start or receipt. */
    ImgDetectionsFilterConfig& setConfidenceRange(float min = 0.0f, float max = 1.0f);
    /** Set the inclusive box area range in output pixels squared. */
    ImgDetectionsFilterConfig& setSizeRange(float minArea = 0.0f, float maxArea = std::numeric_limits<float>::max());
    /** Set the inclusive box width range in output pixels. */
    ImgDetectionsFilterConfig& setWidthRange(float min = 0.0f, float max = std::numeric_limits<float>::max());
    /** Set the inclusive box height range in output pixels. */
    ImgDetectionsFilterConfig& setHeightRange(float min = 0.0f, float max = std::numeric_limits<float>::max());
    /** Return whether each range has a minimum strictly less than its maximum. */
    bool validate() const;
    /** Return whether any output pixel geometry limit is active. */
    bool hasGeometryFilters() const;

    DEPTHAI_SERIALIZE(ImgDetectionsFilterConfig,
                      Buffer::sequenceNum,
                      Buffer::ts,
                      Buffer::tsDevice,
                      Buffer::tsSystem,
                      labelsToKeep,
                      labelsToReject,
                      minConfidence,
                      maxConfidence,
                      minArea,
                      maxArea,
                      minWidth,
                      maxWidth,
                      minHeight,
                      maxHeight,
                      regionOfInterest,
                      maxDetections,
                      sortByConfidence,
                      overlapMode,
                      overlapIouThreshold,
                      reference);
};

}  // namespace dai
