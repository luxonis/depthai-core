#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawCameraControl.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * CameraControl message.
 * Specifies various camera control commands like:
 *
 *  - Still capture
 *
 *  - Auto focus
 *
 *  - Anti banding
 *
 *  - Auto white balance
 *
 *  - Scene
 *
 *  - Effect
 *
 *  - ...
 */
class CameraControl : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawCameraControl& cfg;

   public:
    using AutoFocusMode = RawCameraControl::AutoFocusMode;
    using AntiBandingMode = RawCameraControl::AntiBandingMode;
    using AutoWhiteBalanceMode = RawCameraControl::AutoWhiteBalanceMode;
    using SceneMode = RawCameraControl::SceneMode;
    using EffectMode = RawCameraControl::EffectMode;

    /// Construct CameraControl message
    CameraControl();
    explicit CameraControl(std::shared_ptr<RawCameraControl> ptr);
    virtual ~CameraControl() = default;

    /**
     * Set a command to capture a still image
     */
    CameraControl& setCaptureStill(bool capture);

    /**
     * Set a command to start streaming
     */
    CameraControl& setStartStreaming();

    /**
     * Set a command to stop streaming
     */
    CameraControl& setStopStreaming();

    // Focus
    /**
     * Set a command to specify autofocus mode
     */
    CameraControl& setAutoFocusMode(AutoFocusMode mode);

    /**
     * Set a command to trigger autofocus
     */
    CameraControl& setAutoFocusTrigger();

    /**
     * Set a command to specify focus region in pixels
     * @param startX X coordinate of top left corner of region
     * @param startY Y coordinate of top left corner of region
     * @param width Region width
     * @param height Region height
     */
    CameraControl& setAutoFocusRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height);

    /**
     * Set a command to specify manual focus position
     * @param lensPosition specify lens position 0..255
     */
    CameraControl& setManualFocus(uint8_t lensPosition);

    // Exposure
    /**
     * Set a command to enable auto exposure
     */
    CameraControl& setAutoExposureEnable();

    /**
     * Set a command to specify lock auto exposure
     * @param lock Auto exposure lock mode enabled or disabled
     */
    CameraControl& setAutoExposureLock(bool lock);

    /**
     * Set a command to specify auto exposure region in pixels
     * @param startX X coordinate of top left corner of region
     * @param startY Y coordinate of top left corner of region
     * @param width Region width
     * @param height Region height
     */
    CameraControl& setAutoExposureRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height);

    /**
     * Set a command to specify auto exposure compensation
     * @param compensation Compensation value between -9..9
     */
    CameraControl& setAutoExposureCompensation(int compensation);

    /**
     * Set a command to specify auto banding mode
     * @param mode Auto banding mode to use
     */
    CameraControl& setAntiBandingMode(AntiBandingMode mode);

    /**
     * Set a command to manually specify exposure
     * @param exposureTimeUs Exposure time in microseconds
     * @param sensitivityIso Sensitivity as ISO value, usual range 100..1600
     */
    CameraControl& setManualExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso);

    // White Balance
    /**
     * Set a command to specify auto white balance mode
     * @param mode Auto white balance mode to use
     */
    CameraControl& setAutoWhiteBalanceMode(AutoWhiteBalanceMode mode);

    /**
     * Set a command to specify auto white balance lock
     * @param lock Auto white balance lock mode enabled or disabled
     */
    CameraControl& setAutoWhiteBalanceLock(bool lock);

    /**
     * Set a command to manually specify white-balance color correction
     * @param colorTemperatureK Light source color temperature in kelvins, range 1000..12000
     */
    CameraControl& setManualWhiteBalance(int colorTemperatureK);

    // Other image controls
    /**
     * Set a command to adjust image brightness
     * @param value Brightness, range -10..10
     */
    CameraControl& setBrightness(int value);

    /**
     * Set a command to adjust image contrast
     * @param value Contrast, range -10..10
     */
    CameraControl& setContrast(int value);

    /**
     * Set a command to adjust image saturation
     * @param value Saturation, range -10..10
     */
    CameraControl& setSaturation(int value);

    /**
     * Set a command to adjust image sharpness
     * @param value Sharpness, range 0..4
     */
    CameraControl& setSharpness(int value);

    /**
     * Set a command to adjust luma denoise amount
     * @param value Luma denoise amount, range 0..4
     */
    CameraControl& setLumaDenoise(int value);

    /**
     * Set a command to adjust chroma denoise amount
     * @param value Chroma denoise amount, range 0..4
     */
    CameraControl& setChromaDenoise(int value);

    /**
     * Set a command to specify scene mode
     * @param mode Scene mode
     */
    CameraControl& setSceneMode(SceneMode mode);

    /**
     * Set a command to specify effect mode
     * @param mode Effect mode
     */
    CameraControl& setEffectMode(EffectMode mode);

    // Functions to retrieve properties
    /**
     * Check whether command to capture a still is set
     * @returns True if capture still command is set
     */
    bool getCaptureStill() const;
};

}  // namespace dai
