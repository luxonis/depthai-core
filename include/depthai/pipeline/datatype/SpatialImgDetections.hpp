#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/SpatialKeypoint.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgDetectionsT.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "depthai/utility/ProtoSerializable.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * SpatialImgDetection structure
 *
 * Contains image detection results together with spatial location data.
 */
struct SpatialImgDetection {
   private:
    void transformKeypointsFallback(const ImgTransformation& source, const ImgTransformation& target, const Point3f& spatialCoordinates);

   public:
    uint32_t label = 0;
    std::string labelName;
    float confidence = 0.f;
    float xmin = 0.f;
    float ymin = 0.f;
    float xmax = 0.f;
    float ymax = 0.f;
    std::optional<RotatedRect> boundingBox;
    std::optional<SpatialKeypointsList> keypoints;
    Point3f spatialCoordinates;
    SpatialLocationCalculatorConfigData boundingBoxMapping;

    SpatialImgDetection() = default;
    SpatialImgDetection(const RotatedRect& boundingBox, Point3f spatialCoordinates, float confidence = 1.f, uint32_t label = 0);
    SpatialImgDetection(const RotatedRect& boundingBox, Point3f spatialCoordinates, std::string labelName, float confidence, uint32_t label);
    SpatialImgDetection(
        const RotatedRect& boundingBox, Point3f spatialCoordinates, const SpatialKeypointsList& keypoints, float confidence = 1.f, uint32_t label = 0);
    SpatialImgDetection(const RotatedRect& boundingBox,
                        Point3f spatialCoordinates,
                        const SpatialKeypointsList& keypoints,
                        std::string labelName,
                        float confidence,
                        uint32_t label);

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
    void setKeypoints(const SpatialKeypointsList keypoints);

    /**
     * Sets the keypoints of the detection.
     * @param keypoints list of Keypoint objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<SpatialKeypoint> keypoints);

    /**
     * Sets the keypoints of the detection.
     * @param keypoints list of Point2f objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<SpatialKeypoint> keypoints, const std::vector<Edge> edges);

    /**
     * Sets the keypoints of the detection.
     * @param keypoints list of Point3f objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Point3f> keypoints);

    /**
     * Sets spatial coordinates for the detection.
     * @param spatialCoordinates list of Point3f objects to set.
     * @note The size of spatialCoordinates.
     */
    void setSpatialCoordinate(const Point3f spatialCoordinates);

    /**
     * Sets edges for the keypoints, throws if no keypoints were set beforehand.
     */
    void setEdges(const std::vector<Edge> edges);

    /**
     * Converts SpatialImgDetection to ImgDetection by dropping spatial data.
     * @return dai::ImgDetection object.
     */
    dai::ImgDetection getImgDetection() const;

    /**
     * Returns a list of Keypoint objects, or empty list if no keypoints were set.
     */
    std::vector<SpatialKeypoint> getKeypoints() const;

    /**
     * Returns a list of spatial coordinates for each keypoint, or empty list if no keypoints were set.
     */
    std::vector<dai::Point3f> getKeypointSpatialCoordinates() const;

    /**
     * Returns a list of edges, each edge is a pair of indices, or empty list if no keypoints were set.
     */
    std::vector<Edge> getEdges() const;

    /**
     * Returns the X coordinate of the center of the bounding box.
     */
    float getCenterX() const;

    /**
     * Returns the Y coordinate of the center of the bounding box.
     */
    float getCenterY() const;

    /**
     * Returns the width of the (rotated) bounding box.
     */
    float getWidth() const;

    /**
     * Returns the height of the (rotated) bounding box.
     */
    float getHeight() const;

    /**
     * Returns the angle of the bounding box.
     */
    float getAngle() const;

    /**
     * Transforms the spatial detection to the target image transformation.
     * @param source Source image transformation.
     * @param target Target image transformation.
     * @param lengthUnit Length unit to use for the transformation. By default, millimeters are assumed.
     */
    void transform(const ImgTransformation& source, const ImgTransformation& target, LengthUnit lengthUnit = LengthUnit::MILLIMETER);

    DEPTHAI_SERIALIZE(
        SpatialImgDetection, label, labelName, confidence, xmin, ymin, xmax, ymax, boundingBox, keypoints, spatialCoordinates, boundingBoxMapping);
};

/**
 * SpatialImgDetections message. Carries detection results together with spatial location data
 */
class SpatialImgDetections : public ImgDetectionsT<SpatialImgDetection>, public ProtoSerializable, public TransformableCRTP<SpatialImgDetections> {
   protected:
    /**
     * Internal transform hook used by transformTo() to apply SpatialImgDetections-specific transformation logic.
     */
    void transformToInternal(const ImgTransformation& target) override;

   public:
    friend class TransformableCRTP<SpatialImgDetections>;
    ~SpatialImgDetections() override;
    using Base = ImgDetectionsT<SpatialImgDetection>;
    using Base::Base;
    using Base::detections;
    using Base::segmentationMaskHeight;
    using Base::segmentationMaskWidth;
    using Base::sequenceNum;
    using Base::ts;
    using Base::tsDevice;
    using Transformable::transformation;

    /**
     * Length unit used by all imgDetections' `spatialCoordinates` in this list.
     */
    LengthUnit unit = LengthUnit::MILLIMETER;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SpatialImgDetections;
    }

    /**
     * Returns a new SpatialImgDetections message with the spatial detections transformed into the target image transformation.
     *
     * For each detection, the bounding box is assumed to lie on a plane parallel to the image plane at depth `detection.spatialCoordinates.z` (that is, all
     * four bounding-box corners are projected using the same depth value). The transformed corners are then fit with the smallest enclosing rotated rectangle
     * to preserve rectangularity.
     *
     * @param target Target image transformation.
     * @return SpatialImgDetections with transformed detections.
     */
    SpatialImgDetections transformTo(const ImgTransformation& target) const;

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

    DEPTHAI_SERIALIZE(SpatialImgDetections, sequenceNum, ts, tsDevice, detections, transformation, segmentationMaskWidth, segmentationMaskHeight, unit);
};

}  // namespace dai
