#include "depthai-shared/common/EepromData.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"



namespace dai {

class CalibrationHandler{

    public:
    explicit CalibrationHandler(std::string eepromDataPath);
    CalibrationHandler(std::string calibrationDataPath, std::string boardConfigPath);
    explicit CalibrationHandler(EepromData eepromData); // should I allow this? Yes needed to load it from device 

    dai::EepromData getEepromData() const;
    // std::vector<std::vector<float>> getCameraIntrinsics(CameraBoardSocket cameraID, int width, int height);
    // std::vector<std::vector<float>> getCameraIntrinsics(CameraBoardSocket cameraID);
    // std::vector<float> getDistortionCoefficients(CameraBoardSocket cameraID);

    // std::vector<std::vector<float>> getCameraExtrinsics(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera);    

    // std::vector<std::vector<float>> getCameraToImuExtrinsics(CameraBoardSocket cameraId, enableMeasuredTranslation = true);
    // How to check if IMU exists ? set everthing in IMU extrinsics to ZERO ???
    // std::vector<std::vector<float>> getImuToCameraExtrinsics(CameraBoardSocket cameraId, enableMeasuredTranslation = true);

    // void eepromToJsonFile(std::string destPath);

    // should I allow setting of measured translation 
    // void setCameraExtrinsics(CameraBoardSocket srcCameraID, CameraBoardSocket destCameraID, std::vector<std::vector<float>> rotationMatrix, std::vector<float> translation);
    // void setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float> intrinsics);
    // void setdistortionCoefficients(CameraBoardSocket cameraId, std::vector<float> distortionCoefficients);
    // void setFov(CameraBoardSocket cameraId, std::vector<float> distortionCoefficients);
    


    private:
    /** when the user is writing extrinsics do we validate if 
     * the connection between all the cameras exists ? 
     * Some users might not need that connection so they might ignore adding 
     * that in that case if the user calls the extrinsics betwwn those cameras it 
     * fails We can provide an appropriate error that connection doesn't exist between the requested camera id's..
     * And other option is making sure the connection exists all the time by validating the links. 
     */
    bool isCameraArrayConnected; 
    EepromData eepromData;
    // bool validateCameraArray();
    // void createSrcLinks();
};

}  // dai namespace