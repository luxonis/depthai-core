#pragma once
#include "depthai/common/Point3d.hpp"
#include "depthai/common/Quaterniond.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
/**
 * Homogeneous 4x4 transformation acting on column vectors.
 * Translation values are preserved without unit conversion.
 */
struct Transform {
    /// Matrix rows, with rotation in the upper-left 3x3 block and XYZ translation in the last column.
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
     * Copy a homogeneous transformation into a message.
     * @param transform Transformation to copy, without validation or unit conversion.
     */
    TransformData(const Transform& transform);
    /**
     * Copy a homogeneous 4x4 matrix into a message.
     * @param data Matrix rows, with rotation in the upper-left 3x3 block and XYZ
     * translation in the last column. Copied without validation or unit conversion.
     */
    TransformData(const std::array<std::array<double, 4>, 4>& data);
    /**
     * Construct a transformation from XYZ translation and a quaternion in (qx, qy, qz, qw) order.
     * The quaternion must be nonzero and is normalized before forming the rotation matrix.
     * Translation values are preserved without unit conversion.
     */
    TransformData(double x, double y, double z, double qx, double qy, double qz, double qw);
    /**
     * Construct a transformation from XYZ translation and roll, pitch, yaw in radians.
     * Rotation is Rz(yaw) * Ry(pitch) * Rx(roll), acting on column vectors.
     * Translation values are preserved without unit conversion.
     */
    TransformData(double x, double y, double z, double roll, double pitch, double yaw);

    virtual ~TransformData();

    /// Transform
    Transform transform;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::TransformData;
    }

    /// Return XYZ translation in the units supplied when constructing the transformation.
    Point3d getTranslation() const;
    /// Return roll, pitch, yaw in radians as the X, Y, Z components.
    Point3d getRotationEuler() const;
    /// Return the rotation quaternion in (qx, qy, qz, qw) order.
    Quaterniond getQuaternion() const;

    DEPTHAI_SERIALIZE(TransformData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, transform);
};

}  // namespace dai
