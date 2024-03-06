#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
struct Transform {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float qx = 0.f;
    float qy = 0.f;
    float qz = 0.f;
    float qw = 1.f;
};

DEPTHAI_SERIALIZE_EXT(Transform, x, y, z, qx, qy, qz, qw);

/**
 * TransformData message. Carries transform in x,y,z,qx,qy,qz,qw format.
 */
class TransformData : public Buffer {
   public:
    /**
     * Construct TransformData message.
     */
    TransformData() = default;
    virtual ~TransformData() = default;

    /// Transform
    Transform transform;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::TransformData;
    };

    DEPTHAI_SERIALIZE(TransformData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, transform);
};

}  // namespace dai
