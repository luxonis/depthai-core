#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawCameraControl.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

// protected inheritance, so serialize isn't visible to users
class CameraControl : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawCameraControl& cfg;

   public:
    CameraControl();
    explicit CameraControl(std::shared_ptr<RawCameraControl> ptr);
    virtual ~CameraControl() = default;

    // Functions to set properties
    void setCaptureStill(bool capture);

    void setStartStreaming();
    void setStopStreaming();

    // Focus
    void setAutoFocusMode(RawCameraControl::AutoFocusMode mode);
    void setAutoFocusTrigger();
    void setAutoFocusRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height);
    void setManualFocus(uint8_t lensPosition);

    // Exposure
    void setAutoExposureEnable();
    void setAutoExposureLock(bool lock);
    void setAutoExposureRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height);
    void setAutoExposureCompensation(int8_t compensation);
    void setAntiBandingMode(RawCameraControl::AntiBandingMode mode);
    void setManualExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso);

    // White Balance
    void setAutoWhiteBalanceMode(RawCameraControl::AutoWhiteBalanceMode mode);
    void setAutoWhiteBalanceLock(bool lock);

    // Other image controls
    void setBrightness(uint16_t value);  // TODO move to AE?
    void setContrast(uint16_t value);
    void setSaturation(uint16_t value);
    void setSharpness(uint16_t value);
    void setNoiseReductionStrength(uint16_t value);
    void setLumaDenoise(uint16_t value);
    void setChromaDenoise(uint16_t value);
    void setSceneMode(RawCameraControl::SceneMode mode);
    void setEffectMode(RawCameraControl::EffectMode mode);

    // Functions to retrieve properties
    bool getCaptureStill() const;
};

}  // namespace dai
