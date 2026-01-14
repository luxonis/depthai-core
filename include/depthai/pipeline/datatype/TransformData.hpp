#pragma once
#include "depthai/common/Point3d.hpp"
#include "depthai/common/Quaterniond.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

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
    /**
     * Construct from a Transform struct.
     */
    TransformData(const Transform& transform);
    /**
     * Construct from a 4x4 transform matrix.
     */
    TransformData(const std::array<std::array<double, 4>, 4>& data);
    /**
     * Construct from translation and quaternion components.
     */
    TransformData(double x, double y, double z, double qx, double qy, double qz, double qw);
    /**
     * Construct from translation and roll/pitch/yaw (radians).
     */
    TransformData(double x, double y, double z, double roll, double pitch, double yaw);

#ifdef DEPTHAI_HAVE_RTABMAP_SUPPORT
    /**
     * Construct from an RTAB-Map transform.
     */
    TransformData(const rtabmap::Transform& transformRTABMap);
    /**
     * Convert to an RTAB-Map transform.
     */
    rtabmap::Transform getRTABMapTransform() const;
#endif
    virtual ~TransformData();

    /// Transform
    Transform transform;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::TransformData;
    }

    Point3d getTranslation() const;
    Point3d getRotationEuler() const;
    Quaterniond getQuaternion() const;

    DEPTHAI_SERIALIZE(TransformData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, transform);
};

}  // namespace dai
