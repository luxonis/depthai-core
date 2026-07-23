// IWYU pragma: private, include "depthai/depthai.hpp"
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "depthai/common/MultiDeviceCalibrationData.hpp"
#include "depthai/device/CalibrationHandler.hpp"

namespace dai {

/**
 * Storage and transform-query helper for a calibrated multi-device camera rig.
 *
 * All public and internal homogeneous transforms follow:
 *
 *     p_destination = T_destination_from_source * p_source
 *
 * In particular:
 * - `CalibrationHandler::getCameraExtrinsics(src, dst)` returns `T_dst_from_src`
 * - `CalibrationHandler::getHousingCalibration(camera, housing)` returns `T_housing_from_camera`
 * - `rigFromAnchor` stores `T_rig_from_anchor`
 *
 * The handler normalizes internal composition to centimeters and performs one
 * final translation-unit conversion on returned matrices.
 */
class MultiDeviceCalibrationHandler {
   public:
    MultiDeviceCalibrationHandler() = default;

    /**
     * Construct from serialized rig calibration data.
     *
     * @param data Serialized multi-device calibration payload
     * @param validate Enable structural validation when true
     */
    explicit MultiDeviceCalibrationHandler(const MultiDeviceCalibrationData& data, std::optional<bool> validate = std::nullopt);

    /**
     * Construct from a JSON file on disk.
     *
     * @param path JSON file path containing `MultiDeviceCalibrationData`
     * @param validate Enable structural validation when true
     */
    explicit MultiDeviceCalibrationHandler(std::filesystem::path path, std::optional<bool> validate = std::nullopt);

    /**
     * Construct from a JSON object.
     *
     * @param json JSON payload containing `MultiDeviceCalibrationData`
     * @param validate Enable structural validation when true
     */
    static MultiDeviceCalibrationHandler fromJson(const nlohmann::json& json, std::optional<bool> validate = std::nullopt);

    /**
     * Get the stored rig calibration payload.
     */
    MultiDeviceCalibrationData getData() const;

    /**
     * Serialize the stored rig calibration payload to JSON.
     */
    nlohmann::json toJson() const;

    /**
     * Write the stored rig calibration payload to a JSON file.
     *
     * @param path Destination file path
     * @return True on success, false otherwise
     */
    bool toJsonFile(std::filesystem::path path) const;

    /**
     * Returns true if a device with the given MXID exists.
     */
    bool hasDevice(const std::string& mxid) const;

    /**
     * Returns all stored device MXIDs in serialized order.
     */
    std::vector<std::string> getDeviceIds() const;

    /**
     * Returns the stored per-device entry for `mxid`.
     *
     * @throws std::runtime_error if `mxid` is unknown
     */
    MultiDeviceCalibrationDevice getDevice(const std::string& mxid) const;

    /**
     * Returns the embedded single-device calibration handler for `mxid`.
     *
     * @throws std::runtime_error if `mxid` is unknown
     */
    CalibrationHandler getDeviceCalibration(const std::string& mxid) const;

    /**
     * Upsert the complete calibration snapshot for a device.
     *
     * The proposed new state is validated before commit, so a failed update
     * leaves the handler unchanged.
     *
     * @param mxid Device MXID key
     * @param calibration Embedded single-device calibration
     * @param anchorFrame Local anchor frame for this device
     * @param rigFromAnchor Stored transform `T_rig_from_anchor`
     */
    void setDevice(const std::string& mxid, const CalibrationHandler& calibration, const MultiDeviceFrame& anchorFrame, const Extrinsics& rigFromAnchor);

    /**
     * Remove all entries matching `mxid`.
     */
    void removeDevice(const std::string& mxid);

    /**
     * Validate structural correctness and transform-query usability.
     *
     * @param throwOnError Throw `std::runtime_error` on validation failure
     */
    void validate(bool throwOnError = true) const;

    /**
     * Returns `T_dst_from_src` for arbitrary local frames on arbitrary devices.
     *
     * @param srcMxid Source device MXID
     * @param srcFrame Source local frame on the source device
     * @param dstMxid Destination device MXID
     * @param dstFrame Destination local frame on the destination device
     * @param useSpecTranslation Forwarded to embedded `CalibrationHandler` local-frame lookups
     * @param unit Requested translation unit for the returned 4x4 transform
     */
    std::vector<std::vector<float>> getFrameExtrinsics(const std::string& srcMxid,
                                                       const MultiDeviceFrame& srcFrame,
                                                       const std::string& dstMxid,
                                                       const MultiDeviceFrame& dstFrame,
                                                       bool useSpecTranslation = false,
                                                       LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Returns `T_dstAnchor_from_srcAnchor` between two stored device anchors.
     */
    std::vector<std::vector<float>> getDeviceExtrinsics(const std::string& srcMxid, const std::string& dstMxid, LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Returns `T_dstCamera_from_srcCamera`.
     */
    std::vector<std::vector<float>> getCameraExtrinsics(const std::string& srcMxid,
                                                        CameraBoardSocket srcCamera,
                                                        const std::string& dstMxid,
                                                        CameraBoardSocket dstCamera,
                                                        bool useSpecTranslation = false,
                                                        LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Returns `T_rig_from_camera`.
     */
    std::vector<std::vector<float>> getCameraToRigExtrinsics(const std::string& mxid,
                                                             CameraBoardSocket camera,
                                                             bool useSpecTranslation = false,
                                                             LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Returns `T_camera_from_rig`.
     */
    std::vector<std::vector<float>> getRigToCameraExtrinsics(const std::string& mxid,
                                                             CameraBoardSocket camera,
                                                             bool useSpecTranslation = false,
                                                             LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Returns `T_rig_from_housing`.
     */
    std::vector<std::vector<float>> getHousingToRigExtrinsics(const std::string& mxid,
                                                              HousingCoordinateSystem housing,
                                                              bool useSpecTranslation = false,
                                                              LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Returns `T_housing_from_rig`.
     */
    std::vector<std::vector<float>> getRigToHousingExtrinsics(const std::string& mxid,
                                                              HousingCoordinateSystem housing,
                                                              bool useSpecTranslation = false,
                                                              LengthUnit unit = LengthUnit::CENTIMETER) const;

   private:
    MultiDeviceCalibrationData data;

    const MultiDeviceCalibrationDevice& getDeviceRef(const std::string& mxid) const;
    static void validateData(const MultiDeviceCalibrationData& data, bool throwOnError);
    static void validateFrameState(const MultiDeviceFrame& frame, const std::string& context);
    static std::string frameToString(const MultiDeviceFrame& frame);
    static std::vector<std::vector<float>> createIdentityTransform();
    static std::vector<std::vector<float>> rigFromAnchorToCentimeterMatrix(const Extrinsics& transform);
    static void scaleTranslationFromCentimetersInPlace(std::vector<std::vector<float>>& transform, LengthUnit unit);
    std::vector<std::vector<float>> getAnchorFromLocalFrame(const MultiDeviceCalibrationDevice& device,
                                                            const MultiDeviceFrame& frame,
                                                            bool useSpecTranslation) const;
    std::vector<std::vector<float>> getRigFromLocalFrame(const MultiDeviceCalibrationDevice& device,
                                                         const MultiDeviceFrame& frame,
                                                         bool useSpecTranslation) const;
};

}  // namespace dai
