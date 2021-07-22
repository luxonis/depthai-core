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
    void setCaptureStill(bool capture);

    /**
     * Set a command to start streaming
     */
    void setStartStreaming();

    /**
     * Set a command to stop streaming
     */
    void setStopStreaming();

    // Focus
    /**
     * Set a command to specify autofocus mode
     */
    void setAutoFocusMode(AutoFocusMode mode);

    /**
     * Set a command to trigger autofocus
     */
    void setAutoFocusTrigger();

    /**
     * Set a command to specify focus region in pixels
     * @param startX X coordinate of top left corner of region
     * @param startY Y coordinate of top left corner of region
     * @param width Region width
     * @param height Region height
     */
    void setAutoFocusRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height);

    /**
     * Set a command to specify manual focus position
     * @param lensPosition specify lens position 0..255
     */
    void setManualFocus(uint8_t lensPosition);

    // Exposure
    /**
     * Set a command to enable auto exposure
     */
    void setAutoExposureEnable();

    /**
     * Set a command to specify lock auto exposure
     * @param lock Auto exposure lock mode enabled or disabled
     */
    void setAutoExposureLock(bool lock);

    /**
     * Set a command to specify auto exposure region in pixels
     * @param startX X coordinate of top left corner of region
     * @param startY Y coordinate of top left corner of region
     * @param width Region width
     * @param height Region height
     */
    void setAutoExposureRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height);

    /**
     * Set a command to specify auto exposure compensation
     * @param compensation Compensation value between -9..9
     */
    void setAutoExposureCompensation(int compensation);

    /**
     * Set a command to specify auto banding mode
     * @param mode Auto banding mode to use
     */
    void setAntiBandingMode(AntiBandingMode mode);

    /**
     * Set a command to manually specify exposure
     * @param exposureTimeUs Exposure time in microseconds
     * @param sensitivityIso Sensitivity as ISO value, usual range 100..1600
     */
    void setManualExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso);

    // White Balance
    /**
     * Set a command to specify auto white balance mode
     * @param mode Auto white balance mode to use
     */
    void setAutoWhiteBalanceMode(AutoWhiteBalanceMode mode);

    /**
     * Set a command to specify auto white balance lock
     * @param lock Auto white balance lock mode enabled or disabled
     */
    void setAutoWhiteBalanceLock(bool lock);

    // Other image controls
    /**
     * Set a command to adjust image brightness
     * @param value Brightness, range -10..10
     */
    void setBrightness(int value);

    /**
     * Set a command to adjust image contrast
     * @param value Contrast, range -10..10
     */
    void setContrast(int value);

    /**
     * Set a command to adjust image saturation
     * @param value Saturation, range -10..10
     */
    void setSaturation(int value);

    /**
     * Set a command to adjust image sharpness
     * @param value Sharpness, range 0..4
     */
    void setSharpness(int value);

    /**
     * Set a command to adjust luma denoise amount
     * @param value Luma denoise amount, range 0..4
     */
    void setLumaDenoise(int value);

    /**
     * Set a command to adjust chroma denoise amount
     * @param value Chroma denoise amount, range 0..4
     */
    void setChromaDenoise(int value);

    /**
     * Set a command to specify scene mode
     * @param mode Scene mode
     */
    void setSceneMode(SceneMode mode);

    /**
     * Set a command to specify effect mode
     * @param mode Effect mode
     */
    void setEffectMode(EffectMode mode);

    // Functions to retrieve properties
    /**
     * Check whether command to capture a still is set
     * @returns True if capture still command is set
     */
    bool getCaptureStill() const;
};

}  // namespace dai
