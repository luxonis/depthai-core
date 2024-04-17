#pragma once
#include "depthai/pipeline/datatype/Buffer.hpp"
#ifdef DEPTHAI_HAVE_RTABMAP_SUPPORT
    #include "rtabmap/core/Transform.h"
#endif
namespace dai {
struct Transform {
    std::vector<std::vector<double>> data;
};

DEPTHAI_SERIALIZE_EXT(Transform, data);

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
    TransformData(const std::vector<std::vector<double>>& data);
    TransformData(double x, double y, double z, double qx, double qy, double qz, double qw);
    TransformData(double x, double y, double z, double roll, double pitch, double yaw);

#ifdef DEPTHAI_HAVE_RTABMAP_SUPPORT
    TransformData(const rtabmap::Transform& transformRTABMap);
    void getRTABMapTransform(rtabmap::Transform& transformRTABMap) const;
#endif
    virtual ~TransformData() = default;

    /// Transform
    Transform transform;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::TransformData;
    };

    void getTranslation(double& x, double& y, double& z) const;
    void getRotationEuler(double& r, double& p, double& y) const;
    void getQuaternion(double& qx, double& qy, double& qz, double& qw) const;

    DEPTHAI_SERIALIZE(TransformData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, transform);
};

}  // namespace dai
