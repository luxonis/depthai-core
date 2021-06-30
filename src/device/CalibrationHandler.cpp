#include "device/CalibrationHandler.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <tuple>
#include <unordered_set>

#include "depthai-shared/common/CameraInfo.hpp"
#include "depthai-shared/common/Extrinsics.hpp"
#include "depthai-shared/common/Point3f.hpp"
#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"
#include "utility/matrixOps.hpp"

namespace dai {

using namespace matrix;

CalibrationHandler::CalibrationHandler(std::string eepromDataPath) {
    std::ifstream jsonStream(eepromDataPath);
    // TODO(sachin): Check if the file exists first.
    if(!jsonStream.is_open()) {
        throw std::runtime_error("Calibration data file doesn't exist at the provided path. Please provide a absolute path.");
    }
    if(!jsonStream.good() || jsonStream.bad()) {
        throw std::runtime_error("Calibration data file not found or corrupted");
    }
    nlohmann::json jsonData = nlohmann::json::parse(jsonStream);
    eepromData = jsonData;
}

CalibrationHandler::CalibrationHandler(std::string calibrationDataPath, std::string boardConfigPath) {
    auto matrixConv = [](std::vector<float>& src, int startIdx) {
        std::vector<std::vector<float>> dest;
        int currIdx = startIdx;
        for(int j = 0; j < 3; j++) {
            std::vector<float> temp;
            for(int k = 0; k < 3; k++) {
                temp.push_back(src[currIdx]);
                currIdx++;
            }
            dest.push_back(temp);
        }
        return dest;
    };

    unsigned versionSize = sizeof(float) * (9 * 7 + 3 * 2 + 14 * 3); /*R1,R2,M1,M2,R,T,M3,R_rgb,T_rgb,d1,d2,d3*/
    std::ifstream file(calibrationDataPath, std::ios::binary);
    if(!file.is_open() || !file.good() || file.bad()) {
        throw std::runtime_error("Calibration data file not found or corrupted");
    }

    std::ifstream boardConfigStream(boardConfigPath);
    if(!boardConfigStream.is_open() || !boardConfigStream.good() || boardConfigStream.bad()) {
        throw std::runtime_error("BoardConfig file not found or corrupted");
    }

    nlohmann::json boardConfigData = nlohmann::json::parse(boardConfigStream);
    CameraBoardSocket left = CameraBoardSocket::LEFT;
    CameraBoardSocket right = CameraBoardSocket::RIGHT;

    if(boardConfigData.contains("board_config")) {
        eepromData.boardName = boardConfigData.at("board_config").at("name").get<std::string>();
        eepromData.boardRev = boardConfigData.at("board_config").at("revision").get<std::string>();
        bool swapLeftRightCam = boardConfigData.at("board_config").at("swap_left_and_right_cameras").get<bool>();
        eepromData.version = 6;

        if(!swapLeftRightCam) {
            right = CameraBoardSocket::LEFT;
            left = CameraBoardSocket::RIGHT;
        }

        eepromData.cameraData[right].specHfovDeg = boardConfigData.at("board_config").at("left_fov_deg").get<float>();
        eepromData.cameraData[left].specHfovDeg = boardConfigData.at("board_config").at("left_fov_deg").get<float>();
        eepromData.cameraData[CameraBoardSocket::RGB].specHfovDeg = boardConfigData.at("board_config").at("rgb_fov_deg").get<float>();

        eepromData.cameraData[left].extrinsics.specTranslation.x = -boardConfigData.at("board_config").at("left_to_right_distance_cm").get<float>();
        eepromData.cameraData[left].extrinsics.specTranslation.y = 0;
        eepromData.cameraData[left].extrinsics.specTranslation.z = 0;

        eepromData.cameraData[right].extrinsics.specTranslation.x = boardConfigData.at("board_config").at("left_to_right_distance_cm").get<float>()
                                                                    - boardConfigData.at("board_config").at("left_to_rgb_distance_cm").get<float>();
        eepromData.cameraData[right].extrinsics.specTranslation.y = 0;
        eepromData.cameraData[right].extrinsics.specTranslation.z = 0;
    } else {
        throw std::runtime_error("board_config key not found");
    }

    file.seekg(0, file.end);
    unsigned fSize = file.tellg();
    file.seekg(0, file.beg);

    if(fSize != versionSize) {
        throw std::runtime_error("The calib file version is less than version 5. which has been deprecated. Please Recalibrate with the new version.");
    }

    std::vector<float> calibrationBuff(versionSize / sizeof(float));
    file.read(reinterpret_cast<char*>(calibrationBuff.data()), fSize);
    // std::vector<float> calibrationBuff(std::istreambuf_iterator<char>(file), {});

    eepromData.stereoRectificationData.rectifiedRotationLeft = matrixConv(calibrationBuff, 0);
    eepromData.stereoRectificationData.rectifiedRotationRight = matrixConv(calibrationBuff, 9);
    eepromData.stereoRectificationData.leftCameraSocket = left;
    eepromData.stereoRectificationData.rightCameraSocket = right;

    eepromData.cameraData[left].intrinsicMatrix = matrixConv(calibrationBuff, 18);
    eepromData.cameraData[right].intrinsicMatrix = matrixConv(calibrationBuff, 27);
    eepromData.cameraData[CameraBoardSocket::RGB].intrinsicMatrix = matrixConv(calibrationBuff, 48);  // 9*5 + 3

    eepromData.cameraData[left].cameraType = CameraModel::Perspective;
    eepromData.cameraData[right].cameraType = CameraModel::Perspective;
    eepromData.cameraData[CameraBoardSocket::RGB].cameraType = CameraModel::Perspective;  // 9*5 + 3

    eepromData.cameraData[left].width = 1280;
    eepromData.cameraData[left].height = 800;

    eepromData.cameraData[right].width = 1280;
    eepromData.cameraData[right].height = 800;

    eepromData.cameraData[CameraBoardSocket::RGB].width = 1920;
    eepromData.cameraData[CameraBoardSocket::RGB].height = 1080;

    eepromData.cameraData[left].distortionCoeff = std::vector<float>(calibrationBuff.begin() + 69, calibrationBuff.begin() + 83);  // 69 + 14
    eepromData.cameraData[right].distortionCoeff = std::vector<float>(calibrationBuff.begin() + 83, calibrationBuff.begin() + 69 + (2 * 14));
    eepromData.cameraData[CameraBoardSocket::RGB].distortionCoeff =
        std::vector<float>(calibrationBuff.begin() + 69 + (2 * 14), calibrationBuff.begin() + 69 + (3 * 14));

    eepromData.cameraData[left].extrinsics.rotationMatrix = matrixConv(calibrationBuff, 36);
    eepromData.cameraData[left].extrinsics.toCameraSocket = right;

    eepromData.cameraData[left].extrinsics.translation.x = calibrationBuff[45];
    eepromData.cameraData[left].extrinsics.translation.y = calibrationBuff[46];
    eepromData.cameraData[left].extrinsics.translation.z = calibrationBuff[47];

    eepromData.cameraData[right].extrinsics.rotationMatrix = matrixConv(calibrationBuff, 57);
    eepromData.cameraData[right].extrinsics.toCameraSocket = CameraBoardSocket::RGB;

    eepromData.cameraData[right].extrinsics.translation.x = -calibrationBuff[66];
    eepromData.cameraData[right].extrinsics.translation.y = -calibrationBuff[67];
    eepromData.cameraData[right].extrinsics.translation.z = -calibrationBuff[68];

    CameraInfo& camera = eepromData.cameraData[right];

    float temp = camera.extrinsics.rotationMatrix[0][1];
    camera.extrinsics.rotationMatrix[0][1] = camera.extrinsics.rotationMatrix[1][0];
    camera.extrinsics.rotationMatrix[1][0] = temp;

    temp = camera.extrinsics.rotationMatrix[0][2];
    camera.extrinsics.rotationMatrix[0][2] = camera.extrinsics.rotationMatrix[2][0];
    camera.extrinsics.rotationMatrix[2][0] = temp;

    temp = camera.extrinsics.rotationMatrix[1][2];
    camera.extrinsics.rotationMatrix[1][2] = camera.extrinsics.rotationMatrix[2][1];
    camera.extrinsics.rotationMatrix[2][1] = temp;
}

CalibrationHandler::CalibrationHandler(EepromData eepromData) {
    this->eepromData = eepromData;
}

dai::EepromData CalibrationHandler::getEepromData() const {
    return eepromData;
}

std::vector<std::vector<float>> CalibrationHandler::getCameraIntrinsics(
    CameraBoardSocket cameraId, int resizeWidth, int resizeHeight, Point2f topLeftPixelId, Point2f bottomRightPixelId) {
    if(eepromData.version < 4) {
        throw std::runtime_error("Your device contains old calibration which doesn't include Intrinsic data. Please recalibrate your device");
    }
    if(eepromData.cameraData[cameraId].intrinsicMatrix.size() == 0 || eepromData.cameraData[cameraId].intrinsicMatrix[0][0] == 0) {
        throw std::runtime_error("There is no Intrinsic matrix available for the the requested cameraID");
    }
    std::vector<std::vector<float>> intrinsicMatrix = eepromData.cameraData[cameraId].intrinsicMatrix;
    if(resizeWidth != -1 || resizeHeight != -1) {
        if(resizeWidth == -1) {
            resizeWidth = eepromData.cameraData[cameraId].width * resizeHeight / static_cast<float>(eepromData.cameraData[cameraId].height);
        }
        if(resizeHeight == -1) {
            resizeHeight = eepromData.cameraData[cameraId].height * resizeWidth / static_cast<float>(eepromData.cameraData[cameraId].width);
        }
        float scale = resizeHeight / static_cast<float>(eepromData.cameraData[cameraId].height);
        if(scale * eepromData.cameraData[cameraId].width < resizeWidth) {
            scale = resizeWidth / static_cast<float>(eepromData.cameraData[cameraId].width);
        }
        std::vector<std::vector<float>> scaleMat = {{scale, 0, 0}, {0, scale, 0}, {0, 0, 1}};
        intrinsicMatrix = matMul(scaleMat, intrinsicMatrix);

        if(scale * eepromData.cameraData[cameraId].height > resizeHeight) {
            intrinsicMatrix[1][2] -= (eepromData.cameraData[cameraId].height * scale - resizeHeight) / 2;
        } else if(scale * eepromData.cameraData[cameraId].width > resizeWidth) {
            intrinsicMatrix[0][2] -= (eepromData.cameraData[cameraId].width * scale - resizeWidth) / 2;
        }
    }
    if(resizeWidth != -1 || resizeHeight != -1) {
        if(topLeftPixelId.y > resizeHeight || bottomRightPixelId.x > resizeWidth) {
            throw std::runtime_error("Invalid Crop size. Crop width or height is more than the original resized height and width");
        }
    } else {
        if(topLeftPixelId.y > eepromData.cameraData[cameraId].height || bottomRightPixelId.x > eepromData.cameraData[cameraId].width) {
            throw std::runtime_error("Invalid Crop size. Crop width or height is more than the original resized height and width");
        }
    }

    if(topLeftPixelId.x > bottomRightPixelId.x || topLeftPixelId.y < bottomRightPixelId.y) {
        throw std::runtime_error("Invalid Crop ratio.");
    }

    intrinsicMatrix[0][2] -= topLeftPixelId.x;
    intrinsicMatrix[1][2] -= bottomRightPixelId.y;
    return intrinsicMatrix;
}

std::vector<std::vector<float>> CalibrationHandler::getCameraIntrinsics(CameraBoardSocket cameraId,
                                                                        Size2f destShape,
                                                                        Point2f topLeftPixelId,
                                                                        Point2f bottomRightPixelId) {
    return getCameraIntrinsics(cameraId, destShape.width, destShape.height, topLeftPixelId, bottomRightPixelId);
}

std::vector<std::vector<float>> CalibrationHandler::getCameraIntrinsics(CameraBoardSocket cameraId,
                                                                        std::tuple<int, int> destShape,
                                                                        Point2f topLeftPixelId,
                                                                        Point2f bottomRightPixelId) {
    return getCameraIntrinsics(cameraId, std::get<0>(destShape), std::get<1>(destShape), topLeftPixelId, bottomRightPixelId);
}

std::tuple<std::vector<std::vector<float>>, int, int> CalibrationHandler::getDefaultIntrinsics(CameraBoardSocket cameraId) {
    if(eepromData.version < 4)
        throw std::runtime_error("Your device contains old calibration which doesn't include Intrinsic data. Please recalibrate your device");

    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end())
        throw std::runtime_error("There is no Camera data available corresponding to the the requested cameraId");

    if(eepromData.cameraData[cameraId].intrinsicMatrix.size() == 0 || eepromData.cameraData[cameraId].intrinsicMatrix[0][0] == 0)
        throw std::runtime_error("There is no Intrinsic matrix available for the the requested cameraID");

    return {eepromData.cameraData[cameraId].intrinsicMatrix, eepromData.cameraData[cameraId].width, eepromData.cameraData[cameraId].height};
}

std::vector<float> CalibrationHandler::getDistortionCoefficients(CameraBoardSocket cameraId) {
    if(eepromData.version < 4)
        throw std::runtime_error("Your device contains old calibration which doesn't include Intrinsic data. Please recalibrate your device");

    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end())
        throw std::runtime_error("There is no Camera data available corresponding to the the requested cameraID");

    if(eepromData.cameraData[cameraId].intrinsicMatrix.size() == 0 || eepromData.cameraData[cameraId].intrinsicMatrix[0][0] == 0)
        throw std::runtime_error("There is no Intrinsic matrix available for the the requested cameraID");

    return eepromData.cameraData[cameraId].distortionCoeff;
}

float CalibrationHandler::getFov(CameraBoardSocket cameraId) {
    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end())
        throw std::runtime_error("There is no Camera data available corresponding to the the requested cameraID");

    return eepromData.cameraData[cameraId].specHfovDeg;
}

uint8_t CalibrationHandler::getLensPosition(CameraBoardSocket cameraId) {
    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end())
        throw std::runtime_error("There is no Camera data available corresponding to the the requested cameraID");

    return eepromData.cameraData[cameraId].lensPosition;
}

std::vector<std::vector<float>> CalibrationHandler::getCameraExtrinsics(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useSpecTranslation) {
    /**
     * 1. Check if both camera ID exists.
     * 2. Check if the forward link exists from source to dest camera. if No go to step 5
     * 3. Call computeExtrinsicMatrix to get the projection matrix from source -> destination camera.
     * 4. Jump to end and return the projection matrix
     * 5. if no check if there is forward link from dest to source. if No return an error that link doesn't exist.
     * 6. Call computeExtrinsicMatrix to get the projection matrix from destination -> source camera.
     * 7. Carry Transpose on the rotation matrix and get neg of Final translation
     * 8. Return the Final TransformationMatrix containing both rotation matrix and Translation
     */
    if(eepromData.cameraData.find(srcCamera) == eepromData.cameraData.end()) {
        throw std::runtime_error("There is no Camera data available corresponding to the the requested source cameraId");
    }
    if(eepromData.cameraData.find(dstCamera) == eepromData.cameraData.end()) {
        throw std::runtime_error("There is no Camera data available corresponding to the the requested destination cameraId");
    }

    std::vector<std::vector<float>> extrinsics;
    if(checkExtrinsicsLink(srcCamera, dstCamera)) {
        return computeExtrinsicMatrix(srcCamera, dstCamera, useSpecTranslation);
    } else if(checkExtrinsicsLink(dstCamera, srcCamera)) {
        extrinsics = computeExtrinsicMatrix(dstCamera, srcCamera, useSpecTranslation);

        float temp = extrinsics[0][1];
        extrinsics[0][1] = extrinsics[1][0];
        extrinsics[1][0] = temp;

        temp = extrinsics[0][2];
        extrinsics[0][2] = extrinsics[2][0];
        extrinsics[2][0] = temp;

        temp = extrinsics[1][2];
        extrinsics[1][2] = extrinsics[2][1];
        extrinsics[2][1] = temp;

        extrinsics[0][3] = -extrinsics[0][3];
        extrinsics[1][3] = -extrinsics[1][3];
        extrinsics[2][3] = -extrinsics[2][3];

        return extrinsics;
    } else {
        throw std::runtime_error("Extrinsic connection between the requested cameraId's doesn't exist. Please recalibrate or modify your calibration data");
    }
    return extrinsics;
}

std::vector<std::vector<float>> CalibrationHandler::getCameraToImuExtrinsics(CameraBoardSocket cameraId, bool useSpecTranslation) {
    std::vector<std::vector<float>> transformationMatrix = getImuToCameraExtrinsics(cameraId, useSpecTranslation);
    float temp = transformationMatrix[0][1];
    transformationMatrix[0][1] = transformationMatrix[1][0];
    transformationMatrix[1][0] = temp;

    temp = transformationMatrix[0][2];
    transformationMatrix[0][2] = transformationMatrix[2][0];
    transformationMatrix[2][0] = temp;

    temp = transformationMatrix[1][2];
    transformationMatrix[1][2] = transformationMatrix[2][1];
    transformationMatrix[2][1] = temp;

    transformationMatrix[0][3] = -transformationMatrix[0][3];
    transformationMatrix[1][3] = -transformationMatrix[1][3];
    transformationMatrix[2][3] = -transformationMatrix[2][3];

    return transformationMatrix;
}

std::vector<std::vector<float>> CalibrationHandler::getImuToCameraExtrinsics(CameraBoardSocket cameraId, bool useSpecTranslation) {
    if(eepromData.imuExtrinsics.rotationMatrix.size() == 0 || eepromData.imuExtrinsics.toCameraSocket == CameraBoardSocket::AUTO) {
        throw std::runtime_error("IMU calibration data is not available on device yet.");
    } else if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        throw std::runtime_error("There is no Camera data available corresponding to the requested source cameraId");
    }

    std::vector<std::vector<float>> transformationMatrix = eepromData.imuExtrinsics.rotationMatrix;
    if(useSpecTranslation) {
        transformationMatrix[0].push_back(eepromData.cameraData[cameraId].extrinsics.specTranslation.x);
        transformationMatrix[1].push_back(eepromData.cameraData[cameraId].extrinsics.specTranslation.y);
        transformationMatrix[2].push_back(eepromData.cameraData[cameraId].extrinsics.specTranslation.z);
    } else {
        transformationMatrix[0].push_back(eepromData.cameraData[cameraId].extrinsics.translation.x);
        transformationMatrix[1].push_back(eepromData.cameraData[cameraId].extrinsics.translation.y);
        transformationMatrix[2].push_back(eepromData.cameraData[cameraId].extrinsics.translation.z);
    }
    std::vector<float> homogeneous_vector = {0, 0, 0, 1};
    transformationMatrix.push_back(homogeneous_vector);

    if(eepromData.imuExtrinsics.toCameraSocket == cameraId) {
        return transformationMatrix;
    } else {
        std::vector<std::vector<float>> localTransformationMatrix = getCameraExtrinsics(eepromData.imuExtrinsics.toCameraSocket, cameraId, useSpecTranslation);
        return matMul(transformationMatrix, localTransformationMatrix);
    }
}

std::vector<std::vector<float>> CalibrationHandler::getStereoRightRectificationRotation() {
    std::vector<std::vector<float>> rotationMatrix = eepromData.stereoRectificationData.rectifiedRotationRight;
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3) {
        throw std::runtime_error("Rectified Rotation Matrix Doesn't exist ");
    }
    return rotationMatrix;
}

std::vector<std::vector<float>> CalibrationHandler::getStereoLeftRectificationRotation() {
    ;
    std::vector<std::vector<float>> rotationMatrix = eepromData.stereoRectificationData.rectifiedRotationLeft;
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3) {
        throw std::runtime_error("Rectified Rotation Matrix Doesn't exist ");
    }
    return rotationMatrix;
}

dai::CameraBoardSocket CalibrationHandler::getStereoLeftCameraId() {
    return eepromData.stereoRectificationData.leftCameraSocket;
}

dai::CameraBoardSocket CalibrationHandler::getStereoRightCameraId() {
    return eepromData.stereoRectificationData.rightCameraSocket;
}

bool CalibrationHandler::eepromToJsonFile(std::string destPath) const {
    nlohmann::json j = eepromData;
    std::ofstream ob(destPath);
    ob << std::setw(4) << j << std::endl;
    return true;
}

std::vector<std::vector<float>> CalibrationHandler::computeExtrinsicMatrix(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useSpecTranslation) {
    if(srcCamera == CameraBoardSocket::AUTO || dstCamera == CameraBoardSocket::AUTO) {
        throw std::runtime_error("Invalid cameraId input..");
    }
    if(eepromData.cameraData[srcCamera].extrinsics.toCameraSocket == dstCamera) {
        if(eepromData.cameraData[srcCamera].extrinsics.rotationMatrix.size() == 0
           || eepromData.cameraData[srcCamera].extrinsics.toCameraSocket == CameraBoardSocket::AUTO) {
            throw std::runtime_error(
                "Defined Extrinsic conenction but rotation matrix is not available. Please cross check your calibration data configuration.");
        }
        std::vector<std::vector<float>> transformationMatrix = eepromData.cameraData[srcCamera].extrinsics.rotationMatrix;
        if(useSpecTranslation) {
            dai::Point3f& mTrans = eepromData.cameraData[srcCamera].extrinsics.specTranslation;
            if(mTrans.x == 0 && mTrans.y == 0 && mTrans.z == 0) {
                throw std::runtime_error("Cannot use useSpecTranslation argument since specTranslation has {0, 0, 0}");
            }
            transformationMatrix[0].push_back(eepromData.cameraData[srcCamera].extrinsics.specTranslation.x);
            transformationMatrix[1].push_back(eepromData.cameraData[srcCamera].extrinsics.specTranslation.y);
            transformationMatrix[2].push_back(eepromData.cameraData[srcCamera].extrinsics.specTranslation.z);
        } else {
            transformationMatrix[0].push_back(eepromData.cameraData[srcCamera].extrinsics.translation.x);
            transformationMatrix[1].push_back(eepromData.cameraData[srcCamera].extrinsics.translation.y);
            transformationMatrix[2].push_back(eepromData.cameraData[srcCamera].extrinsics.translation.z);
        }
        std::vector<float> homogeneous_vector = {0, 0, 0, 1};
        transformationMatrix.push_back(homogeneous_vector);
        return transformationMatrix;
    } else {
        std::vector<std::vector<float>> futureTransformationMatrix =
            computeExtrinsicMatrix(eepromData.cameraData[srcCamera].extrinsics.toCameraSocket, dstCamera, useSpecTranslation);
        std::vector<std::vector<float>> currTransformationMatrix = eepromData.cameraData[srcCamera].extrinsics.rotationMatrix;
        if(useSpecTranslation) {
            dai::Point3f& mTrans = eepromData.cameraData[srcCamera].extrinsics.specTranslation;
            if(mTrans.x == 0 && mTrans.y == 0 && mTrans.z == 0) {
                throw std::runtime_error("Cannot use useSpecTranslation argument since specTranslation has {0, 0, 0}");
            }
            currTransformationMatrix[0].push_back(eepromData.cameraData[srcCamera].extrinsics.specTranslation.x);
            currTransformationMatrix[1].push_back(eepromData.cameraData[srcCamera].extrinsics.specTranslation.y);
            currTransformationMatrix[2].push_back(eepromData.cameraData[srcCamera].extrinsics.specTranslation.z);
        } else {
            currTransformationMatrix[0].push_back(eepromData.cameraData[srcCamera].extrinsics.translation.x);
            currTransformationMatrix[1].push_back(eepromData.cameraData[srcCamera].extrinsics.translation.y);
            currTransformationMatrix[2].push_back(eepromData.cameraData[srcCamera].extrinsics.translation.z);
        }
        std::vector<float> homogeneous_vector = {0, 0, 0, 1};
        currTransformationMatrix.push_back(homogeneous_vector);
        return matMul(currTransformationMatrix, futureTransformationMatrix);
    }
}

bool CalibrationHandler::checkExtrinsicsLink(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera) {
    bool isConnectionFound = false;
    CameraBoardSocket currentCameraId = srcCamera;
    while(currentCameraId != CameraBoardSocket::AUTO) {
        currentCameraId = eepromData.cameraData[currentCameraId].extrinsics.toCameraSocket;
        if(currentCameraId == dstCamera) {
            isConnectionFound = true;
            break;
        }
    }
    return isConnectionFound;
}

void CalibrationHandler::setBoardInfo(std::string boardName, std::string boardRev) {
    eepromData.boardName = boardName;
    eepromData.boardRev = boardRev;
    return;
}

void CalibrationHandler::setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, int width, int height) {
    if(intrinsics.size() != 3 || intrinsics[0].size() != 3) {
        throw std::runtime_error("Intrinsic Matrix size should always be 3x3 ");
    }

    if(intrinsics[0][1] != 0 || intrinsics[1][0] != 0 || intrinsics[2][0] != 0 || intrinsics[2][1] != 0) {
        throw std::runtime_error("Invalid Intrinsic Matrix entered!!");
    }

    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        dai::CameraInfo camera_info;
        camera_info.height = height;
        camera_info.width = width;
        camera_info.intrinsicMatrix = intrinsics;
        eepromData.cameraData.emplace(cameraId, camera_info);
    } else {
        eepromData.cameraData[cameraId].height = height;
        eepromData.cameraData[cameraId].width = width;
        eepromData.cameraData[cameraId].intrinsicMatrix = intrinsics;
    }
    return;
}

void CalibrationHandler::setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, Size2f frameSize) {
    setCameraIntrinsics(cameraId, intrinsics, frameSize.width, frameSize.height);
    return;
}

void CalibrationHandler::setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, std::tuple<int, int> frameSize) {
    setCameraIntrinsics(cameraId, intrinsics, std::get<0>(frameSize), std::get<1>(frameSize));
    return;
}

void CalibrationHandler::setDistortionCoefficients(CameraBoardSocket cameraId, std::vector<float> distortionCoefficients) {
    if(distortionCoefficients.size() != 14) {
        throw std::runtime_error("distortionCoefficients size should always be 14");  // should it be ??
    }

    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        dai::CameraInfo camera_info;
        camera_info.distortionCoeff = distortionCoefficients;
        eepromData.cameraData.emplace(cameraId, camera_info);
    } else {
        eepromData.cameraData[cameraId].distortionCoeff = distortionCoefficients;
    }
    return;
}

void CalibrationHandler::setFov(CameraBoardSocket cameraId, float hfov) {
    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        dai::CameraInfo camera_info;
        camera_info.specHfovDeg = hfov;
        eepromData.cameraData.emplace(cameraId, camera_info);
    } else {
        eepromData.cameraData[cameraId].specHfovDeg = hfov;
    }
    return;
}

void CalibrationHandler::setLensPosition(CameraBoardSocket cameraId, uint8_t lensPosition) {
    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        dai::CameraInfo camera_info;
        camera_info.lensPosition = lensPosition;
        eepromData.cameraData.emplace(cameraId, camera_info);
    } else {
        eepromData.cameraData[cameraId].lensPosition = lensPosition;
    }
    return;
}

void CalibrationHandler::setCameraType(CameraBoardSocket cameraId, CameraModel cameraModel) {
    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        dai::CameraInfo camera_info;
        camera_info.cameraType = cameraModel;
        eepromData.cameraData.emplace(cameraId, camera_info);
    } else {
        eepromData.cameraData[cameraId].cameraType = cameraModel;
    }
    return;
}

void CalibrationHandler::setCameraExtrinsics(CameraBoardSocket srcCameraId,
                                             CameraBoardSocket destCameraId,
                                             std::vector<std::vector<float>> rotationMatrix,
                                             std::vector<float> translation,
                                             std::vector<float> specTranslation) {
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3) {
        throw std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    if(translation.size() != 3) {
        throw std::runtime_error("Translation vector size should always be 3x1");
    }
    if(specTranslation.size() != 3) {
        throw std::runtime_error("specTranslation vector size should always be 3x1");
    }

    dai::Extrinsics extrinsics;
    extrinsics.rotationMatrix = rotationMatrix;
    extrinsics.translation = dai::Point3f(translation[0], translation[1], translation[2]);
    extrinsics.specTranslation = dai::Point3f(specTranslation[0], specTranslation[1], specTranslation[2]);
    extrinsics.toCameraSocket = destCameraId;

    if(eepromData.cameraData.find(srcCameraId) == eepromData.cameraData.end()) {
        dai::CameraInfo camera_info;
        camera_info.extrinsics = extrinsics;
        eepromData.cameraData.emplace(srcCameraId, camera_info);
    } else {
        eepromData.cameraData[srcCameraId].extrinsics = extrinsics;
    }
    return;
}

void CalibrationHandler::setImuExtrinsics(CameraBoardSocket destCameraId,
                                          std::vector<std::vector<float>> rotationMatrix,
                                          std::vector<float> translation,
                                          std::vector<float> specTranslation) {
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3) {
        throw std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    if(translation.size() != 3) {
        throw std::runtime_error("Translation vector size should always be 3x1");
    }
    if(specTranslation.size() != 3) {
        throw std::runtime_error("specTranslation vector size should always be 3x1");
    }

    dai::Extrinsics extrinsics;
    extrinsics.rotationMatrix = rotationMatrix;
    extrinsics.translation = dai::Point3f(translation[0], translation[1], translation[2]);
    extrinsics.specTranslation = dai::Point3f(specTranslation[0], specTranslation[1], specTranslation[2]);
    extrinsics.toCameraSocket = destCameraId;
    eepromData.imuExtrinsics = extrinsics;
    return;
}

void CalibrationHandler::setStereoLeft(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation) {
    if(rectifiedRotation.size() != 3 || rectifiedRotation[0].size() != 3) {
        throw std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    eepromData.stereoRectificationData.rectifiedRotationLeft = rectifiedRotation;
    eepromData.stereoRectificationData.leftCameraSocket = cameraId;
    return;
}

void CalibrationHandler::setStereoRight(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation) {
    if(rectifiedRotation.size() != 3 || rectifiedRotation[0].size() != 3) {
        throw std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    eepromData.stereoRectificationData.rectifiedRotationRight = rectifiedRotation;
    eepromData.stereoRectificationData.rightCameraSocket = cameraId;
    return;
}

bool CalibrationHandler::validateCameraArray() {
    if(eepromData.cameraData.size() > 1) {
        if(eepromData.cameraData.find(dai::CameraBoardSocket::LEFT) != eepromData.cameraData.end()) {
            return checkSrcLinks(dai::CameraBoardSocket::LEFT) || checkSrcLinks(dai::CameraBoardSocket::RIGHT);
        } else {
            spdlog::debug(
                "make sure the head of the Extrinsics is your left camera. Please cross check the data by creating a json file using "
                "eepromToJsonFile(). ");
            return false;
        }
    } else {
        return true;  // Considering this would be bw1093 device
    }
}

bool CalibrationHandler::checkSrcLinks(CameraBoardSocket headSocket) {
    bool isConnectionValidated = true;
    std::unordered_set<dai::CameraBoardSocket> marked;

    while(headSocket != CameraBoardSocket::AUTO) {
        if(eepromData.cameraData.find(headSocket) == eepromData.cameraData.end()) {
            spdlog::debug(
                "Found link to a CameraID whose camera calibration is not loaded. Please cross check the connection by creating a json file using "
                "eepromToJsonFile(). ");
            isConnectionValidated = false;
            break;
        }
        if(marked.find(headSocket) != marked.end()) {
            spdlog::debug(
                "Loop found in extrinsics connection. Please cross check that the extrinsics are connected in an array in single direction by creating "
                "a json file using eepromToJsonFile(). ");
            isConnectionValidated = false;
            break;
        }
        marked.insert(headSocket);
        headSocket = eepromData.cameraData.at(headSocket).extrinsics.toCameraSocket;
    }

    if(isConnectionValidated && eepromData.cameraData.size() != marked.size()) {
        isConnectionValidated = false;
        spdlog::debug("Extrinsics between all the cameras is not found with single head and a tail");
    }
    return isConnectionValidated;
}

}  // namespace dai