#pragma once
#include <string>
#include <tuple>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai-shared/common/Point2f.hpp"

namespace dai {

class CalibrationHandler {
   public:
    CalibrationHandler() = default;
    explicit CalibrationHandler(std::string eepromDataPath);
    CalibrationHandler(std::string calibrationDataPath, std::string boardConfigPath);
    explicit CalibrationHandler(EepromData eepromData);  // should I allow this? Yes needed to load it from device

    dai::EepromData getEepromData() const;
    std::vector<std::vector<float>> getCameraIntrinsics(
        CameraBoardSocket cameraId, int height = -1, int width = -1, Point2f topLeftPixelId = Point2f(), Point2f bottomRightPixelId = Point2f());

    std::tuple<std::vector<std::vector<float>>, int, int> getDefaultIntrinsics(CameraBoardSocket cameraId);
    std::vector<float> getDistortionCoefficients(CameraBoardSocket cameraId);

    std::vector<std::vector<float>> getCameraExtrinsics(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useMeasuredTranslation = false);

    std::vector<std::vector<float>> getCameraToImuExtrinsics(CameraBoardSocket cameraId, bool useMeasuredTranslation = false);
    // How to check if IMU exists ? set everthing in IMU extrinsics to ZERO ???

    std::vector<std::vector<float>> getImuToCameraExtrinsics(CameraBoardSocket cameraId, bool useMeasuredTranslation = false);
    // getStereoRightRectificationRotation();
    // getStereoLeftRectificationRotation();
    // getStereoLeftCameraId();
    // getStereoRightCameraId();

    // TODO(sachin): Add Q matrix. Since Q matrix is specific to the stereo. may be better to have this over there!!
    bool eepromToJsonFile(std::string destPath);

    void setBoardInfo(uint32_t version, bool swapLeftRightCam, std::string boardName, std::string boardRev);
    void setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, int width, int height);
    void setdistortionCoefficients(CameraBoardSocket cameraId, std::vector<float> distortionCoefficients);
    void setFov(CameraBoardSocket cameraId, double hfov);
    // should I allow setting of measured translation
    void setCameraExtrinsics(CameraBoardSocket srcCameraId,
                             CameraBoardSocket destCameraId,
                             std::vector<std::vector<float>> rotationMatrix,
                             std::vector<float> translation);
    void setImuExtrinsics(CameraBoardSocket destCameraId, std::vector<std::vector<float>> rotationMatrix, std::vector<float> translation);

    void setStereoLeft(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation);
    void setStereoRight(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation);

   private:
    /** when the user is writing extrinsics do we validate if
     * the connection between all the cameras exists ?
     * Some users might not need that connection so they might ignore adding
     * that in that case if the user calls the extrinsics betwwn those cameras it
     * fails We can provide an appropriate error that connection doesn't exist between the requested camera id's..
     * And other option is making sure the connection exists all the time by validating the links.
     */
    // bool isCameraArrayConnected;
    dai::EepromData eepromData;
    std::vector<std::vector<float>> computeExtrinsicMatrix(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useMeasuredTranslation = false);
    bool checkExtrinsicsLink(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera);

    // bool validateCameraArray();
    // void createSrcLinks();
};

}  // namespace dai