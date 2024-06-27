#pragma once
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/common/Point3d.hpp"
#include "depthai/common/Quaterniond.hpp"

#ifdef DEPTHAI_HAVE_RTABMAP_SUPPORT
    #include "rtabmap/core/Transform.h"
#endif
namespace dai {
struct Transform {
    std::array<std::array<double, 4>, 4> matrix;
};

DEPTHAI_SERIALIZE_EXT(Transform, matrix);

/**
 * TransformData message. Carries transform in x,y,z,qx,qy,qz,qw format.
 */
class TransformData : public Buffer {
   public:
    /**
     * Construct TransformData message.
     */
    TransformData();
    TransformData(const Transform& transform);
    TransformData(const std::array<std::array<double, 4>, 4>& data);
    TransformData(double x, double y, double z, double qx, double qy, double qz, double qw);
    TransformData(double x, double y, double z, double roll, double pitch, double yaw);

#ifdef DEPTHAI_HAVE_RTABMAP_SUPPORT
    TransformData(const rtabmap::Transform& transformRTABMap);
    rtabmap::Transform getRTABMapTransform() const;
#endif
    virtual ~TransformData() = default;

    /// Transform
    Transform transform;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::TransformData;
    };

    Point3d getTranslation() const;
    Point3d getRotationEuler() const;
    Quaterniond getQuaternion() const;

    DEPTHAI_SERIALIZE(TransformData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, transform);
};

}  // namespace dai
