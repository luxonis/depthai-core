#pragma once

#include <optional>
#include <string>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/KeypointsList.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
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

struct ImgDetection {
    uint32_t label = 0;
    std::string labelName;
    float confidence = 0.f;
    float xmin = 0.f;
    float ymin = 0.f;
    float xmax = 0.f;
    float ymax = 0.f;
    std::optional<RotatedRect> boundingBox;
    std::optional<KeypointsList> keypoints;

    ImgDetection() = default;
    ImgDetection(const RotatedRect& boundingBox, float confidence, uint32_t label);
    ImgDetection(const RotatedRect& boundingBox, std::string labelName, float confidence, uint32_t label);
    ImgDetection(const RotatedRect& boundingBox, const KeypointsList& keypoints, float confidence, uint32_t label);
    ImgDetection(const RotatedRect& boundingBox, const KeypointsList& keypoints, std::string labelName, float confidence, uint32_t label);

    /**
     * Sets the bounding box and the legacy coordinates of the detection.
     */
    void setBoundingBox(RotatedRect boundingBox);

    /**
     * Returns bounding box if it was set, else it constructs a new one from the legacy xmin, ymin, xmax, ymax values.
     */
    RotatedRect getBoundingBox() const;

    /**
     * Sets the bounding box and the legacy coordinates of the detection from the top-left and bottom-right points.
     */
    void setOuterBoundingBox(const float xmin, const float ymin, const float xmax, const float ymax);

    /**
     * Sets the keypoints of the detection.
     * @param keypoints list of Keypoint objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const KeypointsList keypoints);

    /**
     * Sets the keypoints of the detection.
     * @param keypoints list of Keypoint objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Keypoint> keypoints);

    /**
     * Sets the keypoints of the detection.
     * @param keypoints list of Point2f objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Keypoint> keypoints, const std::vector<Edge> edges);

    /**
     * Sets the keypoints of the detection.
     * @param keypoints list of Point3f objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Point3f> keypoints);

    /**
     * Sets the keypoints of the detection.
     * @param keypoints list of Point2f objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Point2f> keypoints);

    /**
     * Returns a list of Keypoint objects, or empty list if no keypoints were set.
     */
    std::vector<Keypoint> getKeypoints() const;

    /**
     * Sets edges for the keypoints, throws if no keypoints were set beforehand.
     */
    void setEdges(const std::vector<Edge> edges);

    /**
     * Returns a list of edges, each edge is a pair of indices, or empty list if no keypoints were set.
     */
    std::vector<Edge> getEdges() const;

    /**
     * Returns the X coordinate of the center of the bounding box.
     */
    float getCenterX() const noexcept;

    /**
     * Returns the Y coordinate of the center of the bounding box.
     */
    float getCenterY() const noexcept;

    /**
     * Returns the width of the (rotated) bounding box.
     */
    float getWidth() const noexcept;

    /**
     * Returns the height of the (rotated) bounding box.
     */
    float getHeight() const noexcept;

    /**
     * Returns the angle of the bounding box.
     */
    float getAngle() const noexcept;

    DEPTHAI_SERIALIZE(ImgDetection, label, labelName, confidence, xmin, ymin, xmax, ymax, boundingBox, keypoints);
};

/**
 * ImgDetections message. Carries normalized detection results
 */
class ImgDetections : public Buffer, public ProtoSerializable {
   private:
    size_t segmentationMaskWidth = 0;
    size_t segmentationMaskHeight = 0;

   public:
    /**
     * Construct ImgDetections message.
     */
    ImgDetections() = default;
    virtual ~ImgDetections();

    /// Detections
    std::vector<ImgDetection> detections;
    std::optional<ImgTransformation> transformation;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

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
    ImgDetections& setTensorSegmentationMask(XArray2D mask);

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
    ImgDetections& setSegmentationMask(cv::Mat mask);

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
    DEPTHAI_SERIALIZE(
        ImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections, transformation, segmentationMaskWidth, segmentationMaskHeight);
};

}  // namespace dai
