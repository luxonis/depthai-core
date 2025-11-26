#pragma once

#include <optional>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core/mat.hpp>
    #include <opencv2/opencv.hpp>
#endif

// Template for ImgDetections

namespace dai {

template <class DetectionT>
class ImgDetectionsT : public Buffer {
   protected:
    size_t segmentationMaskWidth = 0;
    size_t segmentationMaskHeight = 0;

   public:
    std::vector<DetectionT> detections;
    std::optional<ImgTransformation> transformation;

    ImgDetectionsT() = default;
    virtual ~ImgDetectionsT() = default;

    /*
     * Common API
     */

    /*
     * Returns the width of the segmentation mask.
     */
    std::size_t getSegmentationMaskWidth() const;

    /*
     * Returns the height of the segmentation mask.
     */
    std::size_t getSegmentationMaskHeight() const;

    /*
     * Sets the segmentation mask from a vector of bytes.
     * The size of the vector must be equal to width * height.
     */
    void setSegmentationMask(const std::vector<std::uint8_t>& mask, size_t width, size_t height);

    /*
     * Sets the segmentation mask from an ImgFrame.
     * @param frame Frame must be of type GRAY8
     */
    void setSegmentationMask(dai::ImgFrame& frame);

    /*
     * Returns a copy of the segmentation mask data as a vector of bytes. If mask data is not set, returns std::nullopt.
     */
    std::optional<std::vector<std::uint8_t>> getMaskData() const;

    /*
     * Returns the segmentation mask as an ImgFrame. If mask data is not set, returns std::nullopt.
     */
    std::optional<dai::ImgFrame> getSegmentationMask() const;

// Optional - OpenCV support
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * @note This API only available if OpenCV support is enabled
     */

    /**
     * Copies cv::Mat data to Segmentation Mask buffer
     *
     * @param frame Input cv::Mat frame from which to copy the data
     * @note Throws if mask is not a single channel INT8 type.
     */
    void setCvSegmentationMask(cv::Mat mask);

    /**
     * Retrieves mask data as a cv::Mat copy with specified width and height. If mask data is not set, returns std::nullopt.
     * @param allocator Allows callers to supply a custom cv::MatAllocator for zero-copy/custom memory management; nullptr uses OpenCV’s default.
     */
    std::optional<cv::Mat> getCvSegmentationMask(cv::MatAllocator* allocator = nullptr);

    /**
     * Returns a binary mask where pixels belonging to the instance index are set to 1, others to 0. If mask data is not set, returns std::nullopt.
     * @param index Instance index
     * @param allocator Allows callers to supply a custom cv::MatAllocator for zero-copy/custom memory management; nullptr uses OpenCV’s default.
     */
    std::optional<cv::Mat> getCvSegmentationMaskByIndex(uint8_t index, cv::MatAllocator* allocator = nullptr);

    /**
     * Retrieves data by the semantic class. If no mask data is not set, returns std::nullopt.
     * @param semanticClass Semantic class index
     * @param allocator Allows callers to supply a custom cv::MatAllocator for zero-copy/custom memory management; nullptr uses OpenCV’s default.
     */
    std::optional<cv::Mat> getCvSegmentationMaskByClass(uint8_t semanticClass, cv::MatAllocator* allocator = nullptr);

#endif
};

}  // namespace dai
