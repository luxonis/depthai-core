#pragma once

#include <optional>
#include <string>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"
#include "depthai/utility/span.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core/mat.hpp>
    #include <opencv2/opencv.hpp>
#endif

namespace dai {

/**
 * SegmentationMask message.
 *
 * Segmentation mask of an image is stored as a single-channel UINT8 array, where each value represents a class or instance index.
 * The value 255 is treated as background pixels (no class/instance).
 */
class SegmentationMask : public Buffer, public ProtoSerializable {
    // Optimization option: if network is bottleneck, implement RLE compression for the mask data
   private:
    size_t width = 0;
    size_t height = 0;
    std::vector<std::string> labels;

   public:
    std::optional<ImgTransformation> transformation;

    SegmentationMask();
    SegmentationMask(const std::vector<std::uint8_t>& data, size_t width, size_t height);
    virtual ~SegmentationMask();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SegmentationMask;
    }

    /**
     * Sets the size of the segmentation mask.
     * @note Use with caution as it sets the metadata of the mask without allocating or resizing the underlying data array.
     */
    void setSize(size_t width, size_t height);

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
     * Sets the segmentation mask from a byte span without an extra temporary vector.
     * The span size must be equal to width * height.
     */
    void setMask(span<const std::uint8_t> mask, size_t width, size_t height);

    /**
     * Prepares internal storage for writing and returns a mutable view to it.
     * The caller must fill exactly width * height bytes.
     */
    span<std::uint8_t> prepareMask(size_t width, size_t height);

    /**
     * Sets the class labels associated with the segmentation mask.
     * The label at index `i` in the `labels` vector corresponds to the value `i` in the segmentation mask data array.
     * @param labels Vector of class labels
     */
    void setLabels(const std::vector<std::string>& labels);

    /**
     * Returns a copy of the segmentation mask data as a vector of bytes. If mask data is not set, returns an empty vector.
     */
    std::vector<std::uint8_t> getMaskData() const;
    /**
     * Returns the segmentation mask as an ImgFrame. If mask data is not set, returns an empty frame with only metadata set.
     */
    dai::ImgFrame getFrame() const;

    /**
     * Returns the area (number of pixels) of the specified instance/class index in the segmentation mask.
     * @param index Instance/Class index
     * @note If index is not present in the mask, returns std::nullopt.
     */
    std::optional<uint32_t> getArea(uint8_t index) const;

    /**
     * Returns the normalized centroid (x,y) coordinates of the specified instance/class index in the segmentation mask.
     * @param index Instance/Class index
     * @note If index is not present in the mask, returns std::nullopt.
     */
    std::optional<dai::Point2f> getCentroid(uint8_t index) const;

    /**
     * Returns a list of sorted unique indices present in the segmentation mask.
     */
    std::vector<uint8_t> getUniqueIndices() const;

    /**
     * Returns all class labels associated with the segmentation mask. If no labels are set, returns an empty vector.
     */
    std::vector<std::string> getLabels() const;

    /**
     * Returns a binary mask where pixels belonging to the specified instance/class index are set to 1, others to 0. If mask data is not set, returns an
     * empty vector.
     */
    std::vector<std::uint8_t> getMaskByIndex(uint8_t index) const;

    /**
     * Returns a binary mask where pixels belonging to the specified class label are set to 1, others to 0. If labels are not set or label not found,
     * returns an empty vector.
     */
    std::vector<std::uint8_t> getMaskByLabel(const std::string& label) const;

    /**
     * Returns true if the mask data is not empty and has valid size (width * height).
     */
    bool hasValidMask() const;

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
     * Retrieves mask data as a cv::Mat copy with specified width and height. If mask data is not set, returns an empty matrix.
     * @param allocator Allows callers to supply a custom cv::MatAllocator for zero-copy/custom memory management; nullptr uses OpenCV’s default.
     */
    cv::Mat getCvMask(cv::MatAllocator* allocator = nullptr);

    /**
     * Returns a binary mask where pixels belonging to the instance index are set to 1, others to 0. If mask data is not set, returns an empty matrix.
     * @param index Instance index
     * @param allocator Allows callers to supply a custom cv::MatAllocator for zero-copy/custom memory management; nullptr uses OpenCV’s default.
     */
    cv::Mat getCvMaskByIndex(uint8_t index, cv::MatAllocator* allocator = nullptr);
    /**
     * Calls the opencv findContours function and filters the results based on the provided index. Returns filtered contour as a vector of vectors of
     * non-normalized points. If mask data is not set, returns an empty vector.
     * @param index class index
     */
    std::vector<std::vector<dai::Point2f>> getContour(uint8_t index);

    /**
     * Returns a bounding box for each continous region with the specified index.
     * @param index class index
     * @param calculateRotation If true, returns rotated bounding boxes, otherwise returns the outer, axis-aligned bounding boxes.
     */
    std::vector<dai::RotatedRect> getBoundingBoxes(uint8_t index, bool calculateRotation = true);

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

    DEPTHAI_SERIALIZE(SegmentationMask, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, transformation, width, height, labels);
};

}  // namespace dai
