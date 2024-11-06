#pragma once

#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {

struct ImgDetection {
    uint32_t label = 0;
    float confidence = 0.f;
    float xmin = 0.f;
    float ymin = 0.f;
    float xmax = 0.f;
    float ymax = 0.f;
};

DEPTHAI_SERIALIZE_EXT(ImgDetection, label, confidence, xmin, ymin, xmax, ymax);

/**
 * ImgDetections message. Carries normalized detection results
 */
class ImgDetections : public Buffer, public utility::ProtoSerializable {
   public:
    /**
     * Construct ImgDetections message.
     */
    ImgDetections() = default;
    virtual ~ImgDetections() = default;

    /// Detections
    std::vector<ImgDetection> detections;
    std::optional<ImgTransformation> transformation;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImgDetections;
    };

    /**
     * Serialize message to proto buffer
     *
     * @returns serialized message
     */
    std::vector<std::uint8_t> serializeProto() const override;

    /**
     * Serialize schema to proto buffer
     *
     * @returns serialized schema
     */
    utility::ProtoSerializable::SchemaPair serializeSchema() const override;

    DEPTHAI_SERIALIZE(ImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections, transformation);
};

}  // namespace dai
