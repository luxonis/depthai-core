#include "depthai/device/CalibrationHandler.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <tuple>

#include "depthai-shared/common/CameraInfo.hpp"
#include "depthai-shared/common/Extrinsics.hpp"
#include "depthai-shared/common/Point3f.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "nlohmann/json.hpp"

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
    // validateCameraArray();
}

CalibrationHandler::CalibrationHandler(std::string calibrationDataPath, std::string boardConfigPath) {
    // std::ifstream jsonStream(boardConfigPath);
    // nlohmann::json json_data = nlohmann::json::parse(jsonStream);
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
    if(!file.good() || file.bad()) {
        throw std::runtime_error("Calibration data file not found or corrupted");
    }

    if(file.is_open()) {
        std::ifstream boardConfigStream(boardConfigPath);
        if(!boardConfigStream.good() || boardConfigStream.bad()) {
            throw std::runtime_error("BoardConfig file not found or corrupted");
        }

        if(boardConfigStream.is_open()) {
            nlohmann::json boardConfigData = nlohmann::json::parse(boardConfigStream);

            if(boardConfigData.contains("board_config")) {
                eepromData.boardName = boardConfigData.at("board_config").at("name").get<std::string>();
                eepromData.boardRev = boardConfigData.at("board_config").at("revision").get<std::string>();
                eepromData.swapLeftRightCam = boardConfigData.at("board_config").at("swap_left_and_right_cameras").get<bool>();
                eepromData.version = 5;

                eepromData.cameraData[CameraBoardSocket::RIGHT].measuredFovDeg = boardConfigData.at("board_config").at("left_fov_deg").get<float>();
                eepromData.cameraData[CameraBoardSocket::LEFT].measuredFovDeg = boardConfigData.at("board_config").at("left_fov_deg").get<float>();
                eepromData.cameraData[CameraBoardSocket::RGB].measuredFovDeg = boardConfigData.at("board_config").at("rgb_fov_deg").get<float>();

                eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.measuredTranslation.x =
                    boardConfigData.at("board_config").at("left_to_right_distance_cm").get<float>();
                eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.measuredTranslation.y = 0;
                eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.measuredTranslation.z = 0;
            }
        } else {
            throw std::runtime_error("Failed to open the board config file");
        }

        file.seekg(0, file.end);
        unsigned fSize = file.tellg();
        file.seekg(0, file.beg);

        if(fSize != versionSize) {
            throw std::runtime_error("The calib file version is less than version 5. which has been deprecated. Please Recalibrate with the new version.");
        }

        std::vector<float> calibration_buff(versionSize / sizeof(float));
        file.read(reinterpret_cast<char*>(calibration_buff.data()), fSize);

        eepromData.stereoRectificationData.rectifiedRotationLeft = matrixConv(calibration_buff, 0);
        eepromData.stereoRectificationData.rectifiedRotationRight = matrixConv(calibration_buff, 9);
        // FIXME(sachin) : when swap is enabled should I swap rectification of left and right ?
        eepromData.stereoRectificationData.leftCameraSocket = CameraBoardSocket::LEFT;
        eepromData.stereoRectificationData.rightCameraSocket = CameraBoardSocket::RIGHT;

        eepromData.cameraData[CameraBoardSocket::LEFT].intrinsicMatrix = matrixConv(calibration_buff, 18);
        eepromData.cameraData[CameraBoardSocket::RIGHT].intrinsicMatrix = matrixConv(calibration_buff, 27);
        eepromData.cameraData[CameraBoardSocket::RGB].intrinsicMatrix = matrixConv(calibration_buff, 48);  // 9*5 + 3

        eepromData.cameraData[CameraBoardSocket::LEFT].cameraType = CameraModel::Perspective;
        eepromData.cameraData[CameraBoardSocket::RIGHT].cameraType = CameraModel::Perspective;
        eepromData.cameraData[CameraBoardSocket::RGB].cameraType = CameraModel::Perspective;  // 9*5 + 3

        eepromData.cameraData[CameraBoardSocket::LEFT].width = 1280;
        eepromData.cameraData[CameraBoardSocket::LEFT].height = 800;

        eepromData.cameraData[CameraBoardSocket::RIGHT].width = 1280;
        eepromData.cameraData[CameraBoardSocket::RIGHT].height = 800;

        eepromData.cameraData[CameraBoardSocket::RGB].width = 1920;
        eepromData.cameraData[CameraBoardSocket::RGB].height = 1080;

        eepromData.cameraData[CameraBoardSocket::LEFT].distortionCoeff =
            std::vector<float>(calibration_buff.begin() + 69, calibration_buff.begin() + 83);  // 69 + 14
        eepromData.cameraData[CameraBoardSocket::RIGHT].distortionCoeff =
            std::vector<float>(calibration_buff.begin() + 83, calibration_buff.begin() + 69 + (2 * 14));
        eepromData.cameraData[CameraBoardSocket::RGB].distortionCoeff =
            std::vector<float>(calibration_buff.begin() + 69 + (2 * 14), calibration_buff.begin() + 69 + (3 * 14));

        eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.fromCameraSocket = CameraBoardSocket::AUTO;
        eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.rotationMatrix = matrixConv(calibration_buff, 36);
        eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.toCameraSocket = CameraBoardSocket::RIGHT;

        eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.translation.x = -calibration_buff[45];
        eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.translation.y = -calibration_buff[46];
        eepromData.cameraData[CameraBoardSocket::LEFT].extrinsics.translation.z = -calibration_buff[47];

        eepromData.cameraData[CameraBoardSocket::RIGHT].extrinsics.fromCameraSocket = CameraBoardSocket::AUTO;
        eepromData.cameraData[CameraBoardSocket::RIGHT].extrinsics.rotationMatrix = matrixConv(calibration_buff, 57);
        eepromData.cameraData[CameraBoardSocket::RIGHT].extrinsics.toCameraSocket = CameraBoardSocket::RGB;

        eepromData.cameraData[CameraBoardSocket::RIGHT].extrinsics.translation.x = calibration_buff[66];
        eepromData.cameraData[CameraBoardSocket::RIGHT].extrinsics.translation.y = calibration_buff[67];
        eepromData.cameraData[CameraBoardSocket::RIGHT].extrinsics.translation.z = calibration_buff[68];

        CameraInfo& camera = eepromData.cameraData[CameraBoardSocket::LEFT];

        float temp = camera.extrinsics.rotationMatrix[0][1];
        camera.extrinsics.rotationMatrix[0][1] = camera.extrinsics.rotationMatrix[1][0];
        camera.extrinsics.rotationMatrix[1][0] = temp;

        temp = camera.extrinsics.rotationMatrix[0][2];
        camera.extrinsics.rotationMatrix[0][2] = camera.extrinsics.rotationMatrix[2][0];
        camera.extrinsics.rotationMatrix[2][0] = temp;

        temp = camera.extrinsics.rotationMatrix[1][2];
        camera.extrinsics.rotationMatrix[1][2] = camera.extrinsics.rotationMatrix[2][1];
        camera.extrinsics.rotationMatrix[2][1] = temp;

    } else {
        throw std::runtime_error("Failed to open the .calib file");
    }
}

CalibrationHandler::CalibrationHandler(EepromData eepromData) {
    this->eepromData = eepromData;
}

dai::EepromData CalibrationHandler::getEepromData() const {
    return eepromData;
}

// FIXME(sachin): Returns wrong value when resizing from 1080 to 720 with only height passed
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
        intrinsicMatrix = matMul(intrinsicMatrix, scaleMat);

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

    std::cout << topLeftPixelId.x << " - " << bottomRightPixelId.y << std::endl;
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

std::vector<std::vector<float>> CalibrationHandler::getCameraToImuExtrinsics(CameraBoardSocket cameraId, bool useMeasuredTranslation) {
    std::vector<std::vector<float>> transformationMatrix = getImuToCameraExtrinsics(cameraId, useMeasuredTranslation);
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

std::vector<std::vector<float>> CalibrationHandler::getImuToCameraExtrinsics(CameraBoardSocket cameraId, bool useMeasuredTranslation) {
    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        throw std::runtime_error("There is no Camera data available corresponding to the the requested source cameraId");
    }
    std::vector<std::vector<float>> transformationMatrix = eepromData.imuExtrinsics.rotationMatrix;
    if(useMeasuredTranslation) {
        // TODO(sachin): What if measured translation is (0,0,0) ??? Should I throw an error ?
        transformationMatrix[0].push_back(eepromData.cameraData[cameraId].extrinsics.measuredTranslation.x);
        transformationMatrix[1].push_back(eepromData.cameraData[cameraId].extrinsics.measuredTranslation.y);
        transformationMatrix[2].push_back(eepromData.cameraData[cameraId].extrinsics.measuredTranslation.z);
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
        std::vector<std::vector<float>> localTransformationMatrix =
            getCameraExtrinsics(eepromData.imuExtrinsics.toCameraSocket, cameraId, useMeasuredTranslation);
        return matMul(transformationMatrix, localTransformationMatrix);
    }
}

// FIXME(sachin): Does inverse works on the 4x4 projection matrix since it is in homogeneous  coordinate system? I think yes but corss check once
// TODO(sachin) : Add a loop checker to make sure lin found doesnt go into infinite loop
std::vector<std::vector<float>> CalibrationHandler::getCameraExtrinsics(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useMeasuredTranslation) {
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
        return computeExtrinsicMatrix(srcCamera, dstCamera, useMeasuredTranslation);
    } else if(checkExtrinsicsLink(dstCamera, srcCamera)) {
        extrinsics = computeExtrinsicMatrix(dstCamera, srcCamera, useMeasuredTranslation);

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

std::vector<std::vector<float>> CalibrationHandler::computeExtrinsicMatrix(CameraBoardSocket srcCamera,
                                                                           CameraBoardSocket dstCamera,
                                                                           bool useMeasuredTranslation) {
    if(srcCamera == CameraBoardSocket::AUTO || dstCamera == CameraBoardSocket::AUTO) {
        throw std::runtime_error("Invalid cameraId input..");
    }
    if(eepromData.cameraData[srcCamera].extrinsics.toCameraSocket == dstCamera) {
        std::vector<std::vector<float>> transformationMatrix = eepromData.cameraData[srcCamera].extrinsics.rotationMatrix;
        if(useMeasuredTranslation) {
            // TODO(sachin): What if measured translation is (0,0,0) ??? Should I throw an error ?
            transformationMatrix[0].push_back(eepromData.cameraData[srcCamera].extrinsics.measuredTranslation.x);
            transformationMatrix[1].push_back(eepromData.cameraData[srcCamera].extrinsics.measuredTranslation.y);
            transformationMatrix[2].push_back(eepromData.cameraData[srcCamera].extrinsics.measuredTranslation.z);
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
            computeExtrinsicMatrix(eepromData.cameraData[srcCamera].extrinsics.toCameraSocket, dstCamera, useMeasuredTranslation);
        std::vector<std::vector<float>> currTransformationMatrix = eepromData.cameraData[srcCamera].extrinsics.rotationMatrix;
        if(useMeasuredTranslation) {
            // TODO(sachin): What if measured translation is (0,0,0) ??? Should I throw an error ?
            currTransformationMatrix[0].push_back(eepromData.cameraData[srcCamera].extrinsics.measuredTranslation.x);
            currTransformationMatrix[1].push_back(eepromData.cameraData[srcCamera].extrinsics.measuredTranslation.y);
            currTransformationMatrix[2].push_back(eepromData.cameraData[srcCamera].extrinsics.measuredTranslation.z);
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

std::vector<std::vector<float>> CalibrationHandler::getStereoLeftRectificationRotation() {
    ;
    std::vector<std::vector<float>> rotationMatrix = eepromData.stereoRectificationData.rectifiedRotationLeft;
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3) {
        throw std::runtime_error("Rectified Rotation Matrix Doesn't exist ");
    }
    return rotationMatrix;
}

std::vector<std::vector<float>> CalibrationHandler::getStereoRightRectificationRotation() {
    std::vector<std::vector<float>> rotationMatrix = eepromData.stereoRectificationData.rectifiedRotationRight;
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3) {
        throw std::runtime_error("Rectified Rotation Matrix Doesn't exist ");
    }
    return rotationMatrix;
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

void CalibrationHandler::setCameraExtrinsics(CameraBoardSocket srcCameraId,
                                             CameraBoardSocket destCameraId,
                                             std::vector<std::vector<float>> rotationMatrix,
                                             std::vector<float> translation,
                                             std::vector<float> measuredTranslation) {
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3) {
        throw std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    //  TODO(sachin): Add measuredTranslation also as optional argumnet ?
    if(translation.size() != 3) {
        throw std::runtime_error("Translation vector size should always be 3x1");
    }
    if(measuredTranslation.size() != 3) {
        throw std::runtime_error("measuredTranslation vector size should always be 3x1");
    }

    dai::Extrinsics extrinsics;
    extrinsics.rotationMatrix = rotationMatrix;
    extrinsics.translation = dai::Point3f(translation[0], translation[1], translation[2]);
    extrinsics.measuredTranslation = dai::Point3f(measuredTranslation[0], measuredTranslation[1], measuredTranslation[2]);
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
                                          std::vector<float> measuredTranslation) {
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3) {
        throw std::runtime_error("Rotation Matrix size should always be 3x3 ");
    }
    //  TODO(sachin): Add measuredTranslation also as optional argumnet ?
    if(translation.size() != 3) {
        throw std::runtime_error("Translation vector size should always be 3x1");
    }
    if(measuredTranslation.size() != 3) {
        throw std::runtime_error("measuredTranslation vector size should always be 3x1");
    }

    dai::Extrinsics extrinsics;
    extrinsics.rotationMatrix = rotationMatrix;
    extrinsics.translation = dai::Point3f(translation[0], translation[1], translation[2]);
    extrinsics.measuredTranslation = dai::Point3f(measuredTranslation[0], measuredTranslation[1], measuredTranslation[2]);
    extrinsics.toCameraSocket = destCameraId;
    eepromData.imuExtrinsics = extrinsics;
    return;
}

void CalibrationHandler::setdistortionCoefficients(CameraBoardSocket cameraId, std::vector<float> distortionCoefficients) {
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

void CalibrationHandler::setBoardInfo(bool swapLeftRightCam, std::string boardName, std::string boardRev) {
    eepromData.swapLeftRightCam = swapLeftRightCam;
    eepromData.boardName = boardName;
    eepromData.boardRev = boardRev;
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

void CalibrationHandler::setFov(CameraBoardSocket cameraId, double hfov) {
    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        dai::CameraInfo camera_info;
        camera_info.measuredFovDeg = hfov;
        eepromData.cameraData.emplace(cameraId, camera_info);
    } else {
        eepromData.cameraData[cameraId].measuredFovDeg = hfov;
    }
    return;
}

void CalibrationHandler::setlensPosition(CameraBoardSocket cameraId, uint8_t lensPosition) {
    if(eepromData.cameraData.find(cameraId) == eepromData.cameraData.end()) {
        dai::CameraInfo camera_info;
        camera_info.lensPosition = lensPosition;
        eepromData.cameraData.emplace(cameraId, camera_info);
    } else {
        eepromData.cameraData[cameraId].lensPosition = lensPosition;
    }
    return;
}

bool CalibrationHandler::eepromToJsonFile(std::string destPath) {
    nlohmann::json j = eepromData;
    std::ofstream ob(destPath);
    ob << std::setw(4) << j << std::endl;
    return true;
}

}  // namespace dai