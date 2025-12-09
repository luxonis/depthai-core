#pragma once

#include <cstddef>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

class SegmentationParserConfig : public Buffer {
   public:
    float confidenceThreshold = 0.5f;
    std::vector<std::string> labels = {};

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT  // used for resizing
    enum class ResizeMode : int { INTER_NEAREST = 0, INTER_LINEAR = 1 };
    size_t outputWidth = 0;
    size_t outputHeight = 0;
    ResizeMode resizeMode = ResizeMode::INTER_NEAREST;

    /**
     * Sets the output size for segmentation parsing
     */
    void setOutputSize(size_t width, size_t height);

    /**
     * Sets the resize mode for segmentation parsing.
     * Fastest is INTER_NEAREST, highest quality is INTER_LINEAR.
     */
    void setResizeMode(ResizeMode mode);
#endif
    /**
     * Construct SegmentationParserConfig message.
     */
    SegmentationParserConfig() = default;
    virtual ~SegmentationParserConfig();

    /**
     * Set confidence threshold for segmentation parsing
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Get confidence threshold for segmentation parsing
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the class labels associated with the segmentation mask.
     * The label at index $i$ in the `labels` vector corresponds to the value $i$ in the segmentation mask data array.
     * @param labels Vector of class labels
     */
    void setLabels(const std::vector<std::string>& labels);

    /**
     * Returns the class labels associated with the segmentation mask.
     */
    std::vector<std::string> getLabels() const;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SegmentationParserConfig;
    }

    DEPTHAI_SERIALIZE(SegmentationParserConfig,
                      confidenceThreshold,
                      labels
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
                      ,
                      outputWidth,
                      outputHeight,
                      resizeMode
#endif
    );
};

}  // namespace dai
