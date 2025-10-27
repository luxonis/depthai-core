#pragma once

#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {

struct ImgDetection {
    uint32_t label = 0;
    std::string labelName;
    float confidence = 0.f;
    float xmin = 0.f;
    float ymin = 0.f;
    float xmax = 0.f;
    float ymax = 0.f;
};

DEPTHAI_SERIALIZE_EXT(ImgDetection, label, labelName, confidence, xmin, ymin, xmax, ymax);

/**
 * ImgDetections message. Carries normalized detection results
 */
class ImgDetections : public Buffer, public ProtoSerializable {
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

    DEPTHAI_SERIALIZE(ImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections, transformation);
};

}  // namespace dai
