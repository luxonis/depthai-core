#pragma once

// std
#include <sys/types.h>

#include <cstddef>
#include <cstdint>

// project
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/span.hpp"

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

namespace dai {

class SegmentationMask : public Buffer {
   private:
    size_t width = 0;
    size_t height = 0;

   public:
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;

    SegmentationMask();
    SegmentationMask(size_t width, size_t height);
    SegmentationMask(std::vector<int>, size_t width, size_t height);

    virtual ~SegmentationMask() = default;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SegmentationMask;
    };

    /**
     * Retrieves image timestamp related to dai::Clock::now()
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp() const;

    /**
     * Retrieves image timestamp directly captured from device's monotonic clock,
     * not synchronized to host time. Used when monotonicity is required.
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestampDevice() const;

    /*
     * Returns the width of the segmentation mask.
     */
    std::size_t getWidth() const;

    /*
     * Returns the height of the segmentation mask.
     */
    std::size_t getHeight() const;

    // Optional - xtensor support
#ifdef DEPTHAI_XTENSOR_SUPPORT
    /**
     * @note This API only available if xtensor support is enabled
     */

    /**
     * Constructs SegmentationMask from 2D xtensor array.
     */
    SegmentationMask(xt::xarray<uint8_t>& mask);

    /**
     * Returns a copy of the segmentation mask data as a 2D array.
     */

    xt::xarray<uint8_t> getTensorMask() const;

    /**
     * Sets the segmentation mask from a 2D xtensor array.
     */
    SegmentationMask& setTensorMask(xt::xarray<uint8_t>& mask);

    /*
     * Returns a binary mask (uint8_t per pixel, 0 or 1) for the specified index.
     * Output is a 2D array of row spans, like get2DMask().
     */
    xt::xarray<uint8_t> getTensorMaskByIndex(uint8_t index) const;

#endif

// Optional - OpenCV support
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * @note This API only available if OpenCV support is enabled
     */

    /**
     * Constructs SegmentationMask from cv::Mat
     *
     * @param frame Input cv::Mat frame from which to copy the data
     */
    SegmentationMask(cv::Mat mask);

    /**
     * Copies cv::Mat data to Segmentation Mask buffer
     *
     * @param frame Input cv::Mat frame from which to copy the data
     */
    SegmentationMask& setMask(cv::Mat mask);

    /**
     * Retrieves data as cv::Mat with specified width, height and type
     *
     * @param copy If false only a reference to data is made, otherwise a copy
     */
    cv::Mat getMask(bool copy = false);

    /**
     * Retrieves data as cv::Mat with specified width and height
     *
     * @param copy If false only a reference to data is made, otherwise a copy
     */
    cv::Mat getCvMask(cv::MatAllocator* allocator = nullptr);

    /**
     * Retrieves data of the specified index as cv::Mat.
     *
     * @param copy If false only a reference to data is made, otherwise a copy
     */
    cv::Mat getCvMaskByIndex(uint8_t index, cv::MatAllocator* allocator = nullptr);
#endif

   public:
    DEPTHAI_SERIALIZE(SegmentationMask, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, width, height);
};
}  // namespace dai