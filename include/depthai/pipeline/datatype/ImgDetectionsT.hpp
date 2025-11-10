#pragma once

#include <optional>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#ifdef DEPTHAI_XTENSOR_SUPPORT
    #include <xtensor/containers/xadapt.hpp>
    #include <xtensor/containers/xbuffer_adaptor.hpp>
    #include <xtensor/core/xlayout.hpp>
    #include <xtensor/core/xmath.hpp>
    #include <xtensor/core/xtensor_forward.hpp>

#endif

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

    // Iterator support
    using value_type = DetectionT;
    using iterator = typename std::vector<DetectionT>::iterator;
    using const_iterator = typename std::vector<DetectionT>::const_iterator;

    iterator begin() noexcept {
        return detections.begin();
    }
    iterator end() noexcept {
        return detections.end();
    }
    const_iterator begin() const noexcept {
        return detections.begin();
    }
    const_iterator end() const noexcept {
        return detections.end();
    }
    const_iterator cbegin() const noexcept {
        return detections.cbegin();
    }
    const_iterator cend() const noexcept {
        return detections.cend();
    }
    bool empty() const noexcept {
        return detections.empty();
    }
    size_t size() const noexcept {
        return detections.size();
    }
    value_type& operator[](size_t i) {
        return detections[i];
    }
    const value_type& operator[](size_t i) const {
        return detections[i];
    }

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
     * Sets the segmentation mask from a vector of bytes, along with width and height.
     * The size of the vector must be equal to width * height.
     */
    void setMask(const std::vector<std::uint8_t>& mask, size_t width, size_t height);

    /*
     * Returns a copy of the segmentation mask data as a vector of bytes. If mask data is not set, returns std::nullopt.
     */
    std::optional<std::vector<std::uint8_t>> getMaskData() const;

    std::optional<dai::ImgFrame> getSegmentationMaskAsImgFrame() const;

    // Optional - xtensor support
#ifdef DEPTHAI_XTENSOR_SUPPORT
    /**
     * @note This API only available if xtensor support is enabled
     */
    using XArray2D = xt::xtensor<std::uint8_t, 2, xt::layout_type::row_major>;

    /**
     * Returns a copy of the segmentation mask data as a 2D array. If mask data is not set, returns std::nullopt.
     */
    std::optional<XArray2D> getTensorSegmentationMask() const;

    /**
     * Sets the segmentation mask from a 2D xtensor array.
     */
    ImgDetectionsT& setTensorSegmentationMask(XArray2D mask);

    /*
     * Returns a binary mask where pixels belonging to the instance index are set to 1, others to 0. If mask data is not set, returns std::nullopt.
     */
    std::optional<XArray2D> getTensorSegmentationMaskByIndex(uint8_t index) const;

#endif

// Optional - OpenCV support
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * @note This API only available if OpenCV support is enabled
     */

    /**
     * Copies cv::Mat data to Segmentation Mask buffer
     *
     * @param frame Input cv::Mat frame from which to copy the data
     */
    ImgDetectionsT& setSegmentationMask(cv::Mat mask);

    /**
     * Retrieves data as cv::Mat with specified width, height and type. If mask data is not set, returns std::nullopt.
     *
     * @param copy If false only a reference to data is made, otherwise a copy
     */
    std::optional<cv::Mat> getSegmentationMask(bool copy = false);

    /**
     * Retrieves data as cv::Mat with specified width and height. If mask data is not set, returns std::nullopt.
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
