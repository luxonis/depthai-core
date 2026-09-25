#pragma once

#include "depthai/common/Point3d.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

namespace dai {

/**
 * Odometry message. Carries a pose and the linear velocity of the tracked body.
 *
 * The pose is inherited from TransformData. Velocity is expressed in the same
 * reference coordinate system as the pose translation and is measured in m/s.
 */
class Odometry : public TransformData {
   public:
    /**
     * Construct an Odometry message with an identity pose and zero velocity.
     */
    Odometry();

    /**
     * Construct an Odometry message.
     * @param transform Pose of the tracked body.
     * @param velocity Linear velocity in the pose reference coordinate system, in m/s.
     */
    Odometry(const Transform& transform, const Point3d& velocity);

    /**
     * Construct an Odometry message from translation, quaternion, and velocity.
     * @param x Translation along the x axis, in meters.
     * @param y Translation along the y axis, in meters.
     * @param z Translation along the z axis, in meters.
     * @param qx Quaternion x component.
     * @param qy Quaternion y component.
     * @param qz Quaternion z component.
     * @param qw Quaternion w component.
     * @param velocity Linear velocity in the pose reference coordinate system, in m/s.
     */
    Odometry(double x, double y, double z, double qx, double qy, double qz, double qw, const Point3d& velocity);

    ~Odometry() override;

    /// Linear velocity in the pose reference coordinate system, in m/s.
    Point3d velocity;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Odometry;
    }

    DEPTHAI_SERIALIZE(Odometry, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, transform, velocity);
};

}  // namespace dai
