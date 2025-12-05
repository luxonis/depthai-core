#pragma once

#include <optional>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

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

/**
 * SegmentationMask message.
 *
 * Segmentation mask of an image is stored as a single-channel INT8 array, where each value represents a class or instance index.
 * The value 255 is treated as a background pixel (no class/instance).
 */
class SegmentationMask : public Buffer, public ProtoSerializable {
   protected:
    size_t width = 0;
    size_t height = 0;

   public:
    using Buffer::getSequenceNum;
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;
    using Buffer::setSequenceNum;
    using Buffer::setTimestamp;
    using Buffer::setTimestampDevice;
    std::optional<ImgTransformation> transformation;

    SegmentationMask();
    SegmentationMask(const std::vector<std::uint8_t>& data, size_t width, size_t height);
    virtual ~SegmentationMask();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SegmentationMask;
    }
    /**
     * Common API
     */

    /**
     * Returns the width of the segmentation mask.
     */
    std::size_t getWidth() const;

    /**
     * Returns the height of the segmentation mask.
     */
    std::size_t getHeight() const;

    /**
     * Sets the segmentation mask from a vector of bytes.
     * The size of the vector must be equal to width * height.
     */
    void setMask(const std::vector<std::uint8_t>& mask, size_t width, size_t height);

    /**
     * Sets the segmentation mask from an ImgFrame.
     * @param frame Frame must be of type GRAY8
     */
    void setMask(dai::ImgFrame& frame);

    /**
     * Returns a copy of the segmentation mask data as a vector of bytes. If mask data is not set, returns std::nullopt.
     */
    std::optional<std::vector<std::uint8_t>> getMaskData() const;

    /*
     * Returns the segmentation mask as an ImgFrame. If mask data is not set, returns std::nullopt.
     */
    std::optional<dai::ImgFrame> getMask() const;

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
    void setCvMask(cv::Mat mask);

    /**
     * Retrieves mask data as a cv::Mat copy with specified width and height. If mask data is not set, returns std::nullopt.
     * @param allocator Allows callers to supply a custom cv::MatAllocator for zero-copy/custom memory management; nullptr uses OpenCV’s default.
     */
    std::optional<cv::Mat> getCvMask(cv::MatAllocator* allocator = nullptr);

    /**
     * Returns a binary mask where pixels belonging to the instance index are set to 1, others to 0. If mask data is not set, returns std::nullopt.
     * @param index Instance index
     * @param allocator Allows callers to supply a custom cv::MatAllocator for zero-copy/custom memory management; nullptr uses OpenCV’s default.
     */
    std::optional<cv::Mat> getCvMaskByIndex(uint8_t index, cv::MatAllocator* allocator = nullptr);

#endif

#ifdef DEPTHAI_ENABLE_PROTOBUF
    /**
     * Serialize message to proto buffer
     *
     * @returns serialized message
     */
    std::vector<std::uint8_t> serializeProto(bool = false) const override;

    /**
     * Serialize schema to proto buffer
     *
     * @returns serialized schema
     */
    ProtoSerializable::SchemaPair serializeSchema() const override;
#endif

    DEPTHAI_SERIALIZE(SegmentationMask, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, transformation, width, height);
};

}  // namespace dai
