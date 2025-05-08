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
    TransformData(const Transform& transform, const std::string& frameId = "", const std::string& parentFrameID = "");
    TransformData(const std::array<std::array<double, 4>, 4>& data, const std::string& frameId = "", const std::string& parentFrameID = "");
    TransformData(
        double x, double y, double z, double qx, double qy, double qz, double qw, const std::string& frameID = "", const std::string& parentFrameID = "");
    TransformData(double x, double y, double z, double roll, double pitch, double yaw, const std::string& frameID = "", const std::string& parentFrameID = "");

#ifdef DEPTHAI_HAVE_RTABMAP_SUPPORT
    TransformData(const rtabmap::Transform& transformRTABMap);
    rtabmap::Transform getRTABMapTransform() const;
#endif
    virtual ~TransformData() = default;

    /// Transform
    Transform transform;
    std::string frameID;
    std::string parentFrameID;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::TransformData;
    };

    Point3d getTranslation() const;
    Point3d getRotationEuler() const;
    Quaterniond getQuaternion() const;

    DEPTHAI_SERIALIZE(TransformData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, transform, frameID, parentFrameID);
};

}  // namespace dai
