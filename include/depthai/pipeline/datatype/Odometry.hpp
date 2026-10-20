#pragma once

#include <optional>

#include "depthai/common/Extrinsics.hpp"
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
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Buffer::tsSystem;
    using TransformData::transform;

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

    /**
     * Transform from the tracked frame whose pose is reported to a named calibration
     * reference, including the tracked frame's axis convention. Translation units and
     * the destination device/camera are carried by Extrinsics. This is independent of
     * the moving pose and does not identify the odometry world or a reset/session.
     * Absent on legacy messages and on producers that do not supply frame metadata.
     */
    std::optional<Extrinsics> trackedFrameExtrinsics = std::nullopt;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Odometry;
    }

    // Keep the legacy six-field binary prefix. Top-level LIBNOP message serialization
    // appends a versioned optional extension, which legacy readers can safely ignore.
    // Direct/nested nop serialization retains only the legacy pose and velocity.
    friend void to_json(nlohmann::json& json, const Odometry& value) {
        json = {{"sequenceNum", value.sequenceNum},
                {"ts", value.ts},
                {"tsDevice", value.tsDevice},
                {"tsSystem", value.tsSystem},
                {"transform", value.transform},
                {"velocity", value.velocity},
                {"trackedFrameExtrinsics", value.trackedFrameExtrinsics}};
    }
    friend void from_json(const nlohmann::json& json, Odometry& value) {
        value.trackedFrameExtrinsics.reset();
        json.at("sequenceNum").get_to(value.sequenceNum);
        json.at("ts").get_to(value.ts);
        json.at("tsDevice").get_to(value.tsDevice);
        json.at("tsSystem").get_to(value.tsSystem);
        json.at("transform").get_to(value.transform);
        json.at("velocity").get_to(value.velocity);
        if(json.contains("trackedFrameExtrinsics")) json.at("trackedFrameExtrinsics").get_to(value.trackedFrameExtrinsics);
    }
    DEPTHAI_DISPLAY(Odometry);
    NOP_STRUCTURE(Odometry, sequenceNum, ts, tsDevice, tsSystem, transform, velocity);
};

namespace utility {
// Specialize bounded message serialization, not nested libnop structures.
template <>
bool serialize<SerializationType::LIBNOP, Odometry>(const Odometry& obj, std::vector<std::uint8_t>& data);
template <>
bool deserialize<SerializationType::LIBNOP, Odometry>(const std::uint8_t* data, std::size_t size, Odometry& obj);
}  // namespace utility

}  // namespace dai
