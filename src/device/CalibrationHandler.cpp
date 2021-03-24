#include "depthai/device/CalibrationHandler.hpp"
#include "depthai-shared/common/CameraInfo.hpp"
#include "depthai-shared/common/Extrinsics.hpp"

#include "nlohmann/json.hpp"
#include <fstream>
#include <string>

namespace dai {

CalibrationHandler::CalibrationHandler(std::string eepromDataPath) {
    std::ifstream jsonStream(eepromDataPath);
    // TODO(sachin): Check if the file exists first.
    nlohmann::json json_data = nlohmann::json::parse(jsonStream);
    eepromData = json_data;
    // validateCameraArray();
}

CalibrationHandler::CalibrationHandler(std::string calibrationDataPath, std::string boardConfigPath) {
    std::ifstream jsonStream(boardConfigPath);
    nlohmann::json json_data = nlohmann::json::parse(jsonStream);
}

CalibrationHandler::CalibrationHandler(EepromData eepromData) {
    this->eepromData = eepromData;
}

dai::EepromData CalibrationHandler::getEepromData() const{
    return eepromData;
}

void CalibrationHandler::setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float> intrinsics, int width, int height){
    if (intrinsics.size() != 3 || intrinsics[0].size() != 3){
        std::runtime_error("Intrinsic Matrix size should always be 3x3 ");
    }

    if(intrinsics[0][1] != 0 || intrinsics[1][0] != 0 || intrinsics[2][0] != 0 || intrinsics[2][1] != 0){
        std::runtime_error("Invalid Intrinsic Matrix entered!!");
    }

    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()){
        dai::CameraInfo camera_info;
        camera_info.intrinsicMatrix = intrinsics;
        eepromData.cameraData.emplace(cameraId, camera_info);
    }
    else{
        eepromData.cameraData[cameraId].intrinsicMatrix = intrinsics;
    }
    return;
}

void CalibrationHandler::setCameraExtrinsics(CameraBoardSocket srcCameraID, CameraBoardSocket destCameraID, std::vector<std::vector<float>> rotationMatrix, std::vector<float> translation){

    if (rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3){
        std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    //  TODO(sachin): Add measuredTranslation also as optional argumnet ?
    if (translation.size() != 3){
        std::runtime_error("Translation vector size should always be 3x1");
    }

    dai::Extrinsics extrinsics;
    extrinsics.rotationMatrix = rotationMatrix;
    extrinsics.translation = dai::Point3f(translation[0], translation[1], translation[2]);
    extrinsics.toCameraSocket = destCameraID;

    if(eepromData.cameraData.find(srcCameraID) == eepromData.cameraData.end()){
        dai::CameraInfo camera_info;
        camera_info.extrinsics = extrinsics;
        eepromData.cameraData.emplace(cameraId, camera_info);
    }
    else{
        eepromData.cameraData[cameraId].extrinsics = extrinsics;
    }
    return;
}
    
void CalibrationHandler::setdistortionCoefficients(CameraBoardSocket cameraId, std::vector<float> distortionCoefficients){
    if (distortionCoefficients.size() != 14){
        std::runtime_error("distortionCoefficients size should always be 14"); // should it be ??
    }
   
    if(eepromData.cameraData.find(srcCameraID) == eepromData.cameraData.end()){
        dai::CameraInfo camera_info;
        camera_info.distortionCoeff = distortionCoefficients;
        eepromData.cameraData.emplace(cameraId, camera_info);
    }
    else{
        eepromData.cameraData[cameraId].distortionCoeff = distortionCoefficients;
    }
    return;
}

void setBoardInfo(uint32_t version, bool swapLeftRightCam, std::string boardName, std::string boardRev){
    eepromData.version = version;
    eepromData.swapLeftRightCam = swapLeftRightCam;
    eepromData.boardName = boardName;
    eepromData.boardRev = boardRev;
    return;
}

void setStereoLeft(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation){

    // if(eepromData.cameraData.find(srcCameraID) == eepromData.cameraData.end()){
    //     std::runtime_error("Initialize left cameraID along with it's intrinsics and extrinsics before adding stereo rectification");
    // }
    if (rectifiedRotation.size() != 3 || rectifiedRotation[0].size() != 3){){
        std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    eepromData.stereoRectificationData.rectifiedRotationLeft = rectifiedRotationLeft;
    eepromData.stereoRectificationData.leftCameraSocket = cameraId;
    return;
}

void setStereoRight(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation){

    // if(eepromData.cameraData.find(srcCameraID) == eepromData.cameraData.end()){
    //     std::runtime_error("Initialize right cameraID along with it's intrinsics and extrinsics before adding stereo rectification");
    // }
    if (rectifiedRotation.size() != 3 || rectifiedRotation[0].size() != 3){){
        std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    eepromData.stereoRectificationData.rectifiedRotationRight = rectifiedRotationLeft;
    eepromData.stereoRectificationData.rightCameraSocket = cameraId;
    return;
}

void CalibrationHandler::setImuExtrinsics(CameraBoardSocket destCameraID, std::vector<std::vector<float>> rotationMatrix, std::vector<float> translation){

    if (rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3){
        std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    //  TODO(sachin): Add measuredTranslation also as optional argumnet ?
    if (translation.size() != 3){
        std::runtime_error("Translation vector size should always be 3x1");
    }

    dai::Extrinsics extrinsics;
    extrinsics.rotationMatrix = rotationMatrix;
    extrinsics.translation = dai::Point3f(translation[0], translation[1], translation[2]);
    extrinsics.toCameraSocket = destCameraID;
    eepromData.imuExtrinsics = extrinsics;
    return;
}

void CalibrationHandler::setFov(CameraBoardSocket cameraId, double hfov){
    if(eepromData.cameraData.find(srcCameraID) == eepromData.cameraData.end()){
        dai::CameraInfo camera_info;
        camera_info.measuredFovDeg = hfov;
        eepromData.cameraData.emplace(cameraId, camera_info);
    }
    else{
        eepromData.cameraData[cameraId].measuredFovDeg = hfov;
    }
    return;
}
}  // namespace dai