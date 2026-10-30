#pragma once

#include <array>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/HousingCoordinateSystem.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * PointCloudConfig message. Carries point cloud output settings.
 */
class PointCloudConfig : public Buffer {
    // false = filter to valid (z > 0) points only (default)
    // true  = keep all width*height points (organized)
    bool organized = false;

    std::array<std::array<float, 4>, 4> transformationMatrix = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};

    LengthUnit lengthUnit = LengthUnit::MILLIMETER;

   public:
    enum class CoordinateSystemType : uint8_t {
        DEFAULT,        ///< Default (camera coordinates, no additional transformation)
        CAMERA_SOCKET,  ///< Transform to another camera
        HOUSING         ///< Transform to housing coordinate system
    };

   private:
    CoordinateSystemType coordSystemType = CoordinateSystemType::DEFAULT;
    CameraBoardSocket targetCameraSocket = CameraBoardSocket::AUTO;
    HousingCoordinateSystem targetHousingCS = HousingCoordinateSystem::AUTO;
    bool useSpecTranslation = false;
    // Device owning the target camera socket / housing. Empty: the device owning the reference camera of the depth frame.
    std::string targetDeviceId;

   public:
    PointCloudConfig() = default;
    virtual ~PointCloudConfig();

    /**
     * Retrieve whether the point cloud is organized (all width*height points kept).
     * @returns true if all width*height points are output, false if only valid (z > 0) points are kept
     */
    bool getOrganized() const;

    /**
     * Retrieve transformation matrix applied to every output point.
     * @returns 4x4 row-major transformation matrix (identity by default)
     */
    std::array<std::array<float, 4>, 4> getTransformationMatrix() const;

    /**
     * Retrieve the length unit used for output point coordinates.
     */
    LengthUnit getLengthUnit() const;

    /**
     * Enable or disable organized point cloud output.
     * When true all width*height points are kept; when false only points with z > 0 are emitted.
     */
    PointCloudConfig& setOrganized(bool enable);

    /**
     * Set a 4x4 transformation matrix applied to every output point.
     * Default is the identity matrix.
     */
    PointCloudConfig& setTransformationMatrix(const std::array<std::array<float, 4>, 4>& transformationMatrix);

    /**
     * Convenience overload: set a 3x3 rotation matrix (translation set to zero).
     */
    PointCloudConfig& setTransformationMatrix(const std::array<std::array<float, 3>, 3>& transformationMatrix);

    /**
     * Set the length unit for output point coordinates.
     */
    PointCloudConfig& setLengthUnit(LengthUnit unit);

    /**
     * Set target coordinate system to another camera socket of the device that owns the reference camera of the depth frame
     * (Extrinsics::toDeviceId of the frame extrinsics).
     * @param targetCamera Target camera socket
     */
    PointCloudConfig& setTargetCoordinateSystem(CameraBoardSocket targetCamera);

    /**
     * Set target coordinate system to a housing coordinate system of the device that owns the reference camera of the depth frame
     * (Extrinsics::toDeviceId of the frame extrinsics).
     * @param housingCS Target housing coordinate system
     */
    PointCloudConfig& setTargetCoordinateSystem(HousingCoordinateSystem housingCS);

    /**
     * Set target coordinate system to a camera socket of any device in the pipeline.
     *
     * When the target device differs from the device that owns the reference camera of a depth frame, the transformation between the
     * two devices is taken from the multi-device calibration of the pipeline (Pipeline::setMultiDeviceCalibration), which has to connect
     * both devices.
     * @param targetDeviceId Device ID of the device owning the target camera socket. An empty ID selects the device owning the reference camera.
     * @param targetCamera Target camera socket
     */
    PointCloudConfig& setTargetCoordinateSystem(const std::string& targetDeviceId, CameraBoardSocket targetCamera);

    /**
     * Set target coordinate system to a housing coordinate system of any device in the pipeline.
     *
     * When the target device differs from the device that owns the reference camera of a depth frame, the transformation between the
     * two devices is taken from the multi-device calibration of the pipeline (Pipeline::setMultiDeviceCalibration), which has to connect
     * both devices.
     * @param targetDeviceId Device ID of the device owning the housing coordinate system. An empty ID selects the device owning the reference
     * camera.
     * @param housingCS Target housing coordinate system
     */
    PointCloudConfig& setTargetCoordinateSystem(const std::string& targetDeviceId, HousingCoordinateSystem housingCS);

    /**
     * Deprecated: use setTargetCoordinateSystem(targetCamera) instead.
     */
    PointCloudConfig& setTargetCoordinateSystem(CameraBoardSocket targetCamera, bool useSpecTranslation);

    /**
     * Deprecated: use setTargetCoordinateSystem(housingCS) instead.
     */
    PointCloudConfig& setTargetCoordinateSystem(HousingCoordinateSystem housingCS, bool useSpecTranslation);

    /**
     * Retrieve the coordinate system type.
     */
    CoordinateSystemType getCoordinateSystemType() const;

    /**
     * Retrieve the target camera socket (valid when coordSystemType == CAMERA_SOCKET).
     */
    CameraBoardSocket getTargetCameraSocket() const;

    /**
     * Retrieve the target housing coordinate system (valid when coordSystemType == HOUSING).
     */
    HousingCoordinateSystem getTargetHousingCS() const;

    /**
     * Retrieve whether spec translation is used.
     */
    bool getUseSpecTranslation() const;

    /**
     * Retrieve the device ID of the device owning the target coordinate system.
     * @returns Device ID, or an empty string when the target lives on the device owning the reference camera of the depth frame
     */
    std::string getTargetDeviceId() const;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::PointCloudConfig;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(PointCloudConfig,
                      Buffer::sequenceNum,
                      Buffer::ts,
                      Buffer::tsDevice,
                      Buffer::tsSystem,
                      organized,
                      transformationMatrix,
                      lengthUnit,
                      coordSystemType,
                      targetCameraSocket,
                      targetHousingCS,
                      useSpecTranslation,
                      targetDeviceId);
};

}  // namespace dai
