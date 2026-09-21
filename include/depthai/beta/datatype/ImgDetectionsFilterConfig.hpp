#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for ImgDetectionsFilter.
 *
 * The default configuration is a no-op: it keeps every detection in its
 * original order.
 */
class ImgDetectionsFilterConfig : public Buffer {
   public:
    ~ImgDetectionsFilterConfig() override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ImgDetectionsFilterConfig;
    }

    /**
     * If set, only detections with one of these labels are kept.
     * Takes precedence over labelsToReject when both are set.
     */
    std::optional<std::vector<std::uint32_t>> labelsToKeep;

    /**
     * If set, detections with one of these labels are removed.
     */
    std::optional<std::vector<std::uint32_t>> labelsToReject;

    /**
     * If set, detections below this confidence are removed.
     */
    std::optional<float> confidenceThreshold;

    /**
     * If set, detections with a normalized bounding-box area below this value
     * are removed.
     */
    std::optional<float> minArea;

    /**
     * Disable non-maximum suppression. Disabled by default to preserve all
     * detections.
     */
    bool nmsDisabled = true;

    /**
     * Confidence threshold applied by non-maximum suppression.
     */
    float nmsConfidenceThreshold = 0.3f;

    /**
     * Intersection-over-union threshold applied by non-maximum suppression.
     */
    float nmsIouThreshold = 0.4f;

    /**
     * Disable confidence sorting. Disabled by default to preserve input order.
     */
    bool sortingDisabled = true;

    /**
     * Sort detections in descending confidence order when sorting is enabled.
     */
    bool sortDescending = true;

    /**
     * If set, retain at most this many detections after filtering and sorting.
     */
    std::optional<std::uint32_t> firstK;

    /**
     * Returns true when this configuration preserves every detection and its
     * order.
     */
    bool isNoOp() const;

    DEPTHAI_SERIALIZE(ImgDetectionsFilterConfig,
                      labelsToKeep,
                      labelsToReject,
                      confidenceThreshold,
                      minArea,
                      nmsDisabled,
                      nmsConfidenceThreshold,
                      nmsIouThreshold,
                      sortingDisabled,
                      sortDescending,
                      firstK);
};

}  // namespace beta
}  // namespace dai
