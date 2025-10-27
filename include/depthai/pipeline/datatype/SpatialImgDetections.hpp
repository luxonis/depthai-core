#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/utility/ProtoSerializable.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * SpatialImgDetection structure
 *
 * Contains image detection results together with spatial location data.
 */
struct SpatialImgDetection : public ImgDetection {
    Point3f spatialCoordinates;
    SpatialLocationCalculatorConfigData boundingBoxMapping;

    float depthAverage = 0.f;
    float depthMode = 0.f;
    float depthMedian = 0.f;
    std::uint16_t depthMin = 0;
    std::uint16_t depthMax = 0;
    std::uint32_t depthAveragePixelCount = 0;

    DEPTHAI_SERIALIZE(SpatialImgDetection,
                      ImgDetection::xmax,
                      ImgDetection::xmin,
                      ImgDetection::ymax,
                      ImgDetection::ymin,
                      ImgDetection::label,
                      ImgDetection::labelName,
                      ImgDetection::confidence,
                      ImgDetection::boundingBox,
                      ImgDetection::keypoints,
                      spatialCoordinates,
                      boundingBoxMapping,
                      depthAverage,
                      depthMode,
                      depthMedian,
                      depthMin,
                      depthMax,
                      depthAveragePixelCount);
};
/**
 * SpatialImgDetections message. Carries detection results together with spatial location data
 */
class SpatialImgDetections : public Buffer, public ProtoSerializable {
   public:
    /**
     * Construct SpatialImgDetections message.
     */
    SpatialImgDetections() = default;
    virtual ~SpatialImgDetections();

    /**
     * Detection results.
     */
    std::vector<SpatialImgDetection> detections;
    std::optional<ImgTransformation> transformation;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

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

    DEPTHAI_SERIALIZE(SpatialImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections, transformation);
};

}  // namespace dai
