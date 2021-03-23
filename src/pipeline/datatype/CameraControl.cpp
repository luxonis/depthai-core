#include "depthai/pipeline/datatype/CameraControl.hpp"

namespace dai {

std::shared_ptr<RawBuffer> CameraControl::serialize() const {
    return raw;
}

CameraControl::CameraControl() : Buffer(std::make_shared<RawCameraControl>()), cfg(*dynamic_cast<RawCameraControl*>(raw.get())) {}
CameraControl::CameraControl(std::shared_ptr<RawCameraControl> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawCameraControl*>(raw.get())) {}

// helpers
// Functions to set properties
void CameraControl::setCaptureStill(bool capture) {
    // Enable capture
    cfg.setCommand(RawCameraControl::Command::STILL_CAPTURE, capture);
}

void CameraControl::setStartStreaming() {
    cfg.setCommand(RawCameraControl::Command::START_STREAM);
}
void CameraControl::setStopStreaming() {
    cfg.setCommand(RawCameraControl::Command::STOP_STREAM);
}

// Focus
void CameraControl::setAutoFocusMode(AutoFocusMode mode) {
    cfg.setCommand(RawCameraControl::Command::AF_MODE);
    cfg.autoFocusMode = mode;
}
void CameraControl::setAutoFocusTrigger() {
    cfg.setCommand(RawCameraControl::Command::AF_TRIGGER);
}
void CameraControl::setAutoFocusRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height) {
    cfg.setCommand(RawCameraControl::Command::AF_REGION);
    cfg.afRegion.x = startX;
    cfg.afRegion.y = startY;
    cfg.afRegion.width = width;
    cfg.afRegion.height = height;
    cfg.afRegion.priority = 1;  // TODO
}
void CameraControl::setManualFocus(uint8_t lensPosition) {
    cfg.setCommand(RawCameraControl::Command::MOVE_LENS);
    cfg.lensPosition = lensPosition;
    setAutoFocusMode(AutoFocusMode::OFF);  // TODO added for initialConfig case
}

// Exposure
void CameraControl::setAutoExposureEnable() {
    cfg.setCommand(RawCameraControl::Command::AE_AUTO);
}
void CameraControl::setAutoExposureLock(bool lock) {
    cfg.setCommand(RawCameraControl::Command::AE_LOCK);
    cfg.aeLockMode = lock;
}
void CameraControl::setAutoExposureRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height) {
    cfg.setCommand(RawCameraControl::Command::AE_REGION);
    cfg.aeRegion.x = startX;
    cfg.aeRegion.y = startY;
    cfg.aeRegion.width = width;
    cfg.aeRegion.height = height;
    cfg.aeRegion.priority = 1;  // TODO
}
void CameraControl::setAutoExposureCompensation(int compensation) {
    cfg.setCommand(RawCameraControl::Command::EXPOSURE_COMPENSATION);
    cfg.expCompensation = compensation;
}
void CameraControl::setAntiBandingMode(AntiBandingMode mode) {
    cfg.setCommand(RawCameraControl::Command::ANTIBANDING_MODE);
    cfg.antiBandingMode = mode;
}
void CameraControl::setManualExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso) {
    cfg.setCommand(RawCameraControl::Command::AE_MANUAL);
    cfg.expManual.exposureTimeUs = exposureTimeUs;
    cfg.expManual.sensitivityIso = sensitivityIso;
    cfg.expManual.frameDurationUs = 0;  // TODO
}

// White Balance
void CameraControl::setAutoWhiteBalanceMode(AutoWhiteBalanceMode mode) {
    cfg.setCommand(RawCameraControl::Command::AWB_MODE);
    cfg.awbMode = mode;
}
void CameraControl::setAutoWhiteBalanceLock(bool lock) {
    cfg.setCommand(RawCameraControl::Command::AWB_LOCK);
    cfg.awbLockMode = lock;
}

// Other image controls
void CameraControl::setBrightness(int value) {
    cfg.setCommand(RawCameraControl::Command::BRIGHTNESS);
    cfg.brightness = value;
}
void CameraControl::setContrast(int value) {
    cfg.setCommand(RawCameraControl::Command::CONTRAST);
    cfg.contrast = value;
}
void CameraControl::setSaturation(int value) {
    cfg.setCommand(RawCameraControl::Command::SATURATION);
    cfg.saturation = value;
}
void CameraControl::setSharpness(int value) {
    cfg.setCommand(RawCameraControl::Command::SHARPNESS);
    cfg.sharpness = value;
}
void CameraControl::setLumaDenoise(int value) {
    cfg.setCommand(RawCameraControl::Command::LUMA_DENOISE);
    cfg.lumaDenoise = value;
}
void CameraControl::setChromaDenoise(int value) {
    cfg.setCommand(RawCameraControl::Command::CHROMA_DENOISE);
    cfg.chromaDenoise = value;
}
void CameraControl::setSceneMode(SceneMode mode) {
    cfg.setCommand(RawCameraControl::Command::SCENE_MODE);
    cfg.sceneMode = mode;
}
void CameraControl::setEffectMode(EffectMode mode) {
    cfg.setCommand(RawCameraControl::Command::EFFECT_MODE);
    cfg.effectMode = mode;
}

bool CameraControl::getCaptureStill() const {
    return cfg.getCommand(RawCameraControl::Command::STILL_CAPTURE);
}

}  // namespace dai
