#include "depthai/pipeline/datatype/CameraControl.hpp"

namespace dai {

// Functions to set properties
CameraControl& CameraControl::setCaptureStill(bool capture) {
    // Enable capture
    setCommand(Command::STILL_CAPTURE, capture);
    return *this;
}

CameraControl& CameraControl::setStartStreaming() {
    setCommand(Command::START_STREAM);
    return *this;
}
CameraControl& CameraControl::setStopStreaming() {
    setCommand(Command::STOP_STREAM);
    return *this;
}
CameraControl& CameraControl::setExternalTrigger(int numFramesBurst, int numFramesDiscard) {
    setCommand(Command::EXTERNAL_TRIGGER);
    lowPowerNumFramesBurst = numFramesBurst;
    lowPowerNumFramesDiscard = numFramesDiscard;
    return *this;
}

CameraControl& CameraControl::setFrameSyncMode(FrameSyncMode mode) {
    setCommand(Command::FRAME_SYNC);
    frameSyncMode = mode;
    return *this;
}

CameraControl& CameraControl::setStrobeSensor(int activeLevel) {
    setCommand(Command::STROBE_CONFIG);
    strobeConfig.enable = true;
    strobeConfig.activeLevel = activeLevel;
    strobeConfig.gpioNumber = -1;
    return *this;
}

CameraControl& CameraControl::setStrobeExternal(int gpioNumber, int activeLevel) {
    setCommand(Command::STROBE_CONFIG);
    strobeConfig.enable = true;
    strobeConfig.activeLevel = activeLevel;
    strobeConfig.gpioNumber = gpioNumber;
    return *this;
}

CameraControl& CameraControl::setStrobeDisable() {
    setCommand(Command::STROBE_CONFIG);
    strobeConfig.enable = false;
    return *this;
}

// Focus
CameraControl& CameraControl::setAutoFocusMode(AutoFocusMode mode) {
    setCommand(Command::AF_MODE);
    autoFocusMode = mode;
    return *this;
}
CameraControl& CameraControl::setAutoFocusTrigger() {
    setCommand(Command::AF_TRIGGER);
    return *this;
}
CameraControl& CameraControl::setAutoFocusLensRange(int infinityPosition, int macroPosition) {
    setCommand(Command::AF_LENS_RANGE);
    lensPosAutoInfinity = infinityPosition;
    lensPosAutoMacro = macroPosition;
    return *this;
}
CameraControl& CameraControl::setAutoFocusRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height) {
    setCommand(Command::AF_REGION);
    afRegion.x = startX;
    afRegion.y = startY;
    afRegion.width = width;
    afRegion.height = height;
    afRegion.priority = 1;  // TODO
    return *this;
}
CameraControl& CameraControl::setManualFocus(uint8_t lensPosition) {
    setCommand(Command::MOVE_LENS);
    lensPosition = lensPosition;
    return *this;
}

// Exposure
CameraControl& CameraControl::setAutoExposureEnable() {
    setCommand(Command::AE_AUTO);
    return *this;
}
CameraControl& CameraControl::setAutoExposureLock(bool lock) {
    setCommand(Command::AE_LOCK);
    aeLockMode = lock;
    return *this;
}
CameraControl& CameraControl::setAutoExposureRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height) {
    setCommand(Command::AE_REGION);
    aeRegion.x = startX;
    aeRegion.y = startY;
    aeRegion.width = width;
    aeRegion.height = height;
    aeRegion.priority = 1;  // TODO
    return *this;
}
CameraControl& CameraControl::setAutoExposureCompensation(int compensation) {
    setCommand(Command::EXPOSURE_COMPENSATION);
    expCompensation = compensation;
    return *this;
}
CameraControl& CameraControl::setAntiBandingMode(AntiBandingMode mode) {
    setCommand(Command::ANTIBANDING_MODE);
    antiBandingMode = mode;
    return *this;
}
CameraControl& CameraControl::setManualExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso) {
    setCommand(Command::AE_MANUAL);
    expManual.exposureTimeUs = exposureTimeUs;
    expManual.sensitivityIso = sensitivityIso;
    expManual.frameDurationUs = 0;  // TODO
    return *this;
}

void CameraControl::setManualExposure(std::chrono::microseconds exposureTime, uint32_t sensitivityIso) {
    setManualExposure(exposureTime.count(), sensitivityIso);
}

// White Balance
CameraControl& CameraControl::setAutoWhiteBalanceMode(AutoWhiteBalanceMode mode) {
    setCommand(Command::AWB_MODE);
    awbMode = mode;
    return *this;
}
CameraControl& CameraControl::setAutoWhiteBalanceLock(bool lock) {
    setCommand(Command::AWB_LOCK);
    awbLockMode = lock;
    return *this;
}
CameraControl& CameraControl::setManualWhiteBalance(int colorTemperatureK) {
    setCommand(Command::WB_COLOR_TEMP);
    wbColorTemp = colorTemperatureK;
    return *this;
}

// Other image controls
CameraControl& CameraControl::setBrightness(int value) {
    setCommand(Command::BRIGHTNESS);
    brightness = value;
    return *this;
}
CameraControl& CameraControl::setContrast(int value) {
    setCommand(Command::CONTRAST);
    contrast = value;
    return *this;
}
CameraControl& CameraControl::setSaturation(int value) {
    setCommand(Command::SATURATION);
    saturation = value;
    return *this;
}
CameraControl& CameraControl::setSharpness(int value) {
    setCommand(Command::SHARPNESS);
    sharpness = value;
    return *this;
}
CameraControl& CameraControl::setLumaDenoise(int value) {
    setCommand(Command::LUMA_DENOISE);
    lumaDenoise = value;
    return *this;
}
CameraControl& CameraControl::setChromaDenoise(int value) {
    setCommand(Command::CHROMA_DENOISE);
    chromaDenoise = value;
    return *this;
}
CameraControl& CameraControl::setSceneMode(SceneMode mode) {
    setCommand(Command::SCENE_MODE);
    sceneMode = mode;
    return *this;
}
CameraControl& CameraControl::setEffectMode(EffectMode mode) {
    setCommand(Command::EFFECT_MODE);
    effectMode = mode;
    return *this;
}

CameraControl& CameraControl::setMisc(std::string control, std::string value) {
    miscControls.push_back(std::make_pair(control, value));
    return *this;
}
CameraControl& CameraControl::setMisc(std::string control, int value) {
    return setMisc(control, std::to_string(value));
}
CameraControl& CameraControl::setMisc(std::string control, float value) {
    return setMisc(control, std::to_string(value));
}
void CameraControl::clearMiscControls() {
    miscControls.clear();
}
std::vector<std::pair<std::string, std::string>> CameraControl::getMiscControls() {
    return miscControls;
}

bool CameraControl::getCaptureStill() const {
    return getCommand(Command::STILL_CAPTURE);
}

std::chrono::microseconds CameraControl::getExposureTime() const {
    return std::chrono::microseconds(expManual.exposureTimeUs);
}

int CameraControl::getSensitivity() const {
    return expManual.sensitivityIso;
}

int CameraControl::getLensPosition() const {
    return lensPosition;
}

}  // namespace dai
