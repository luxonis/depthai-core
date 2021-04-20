#pragma once
#include <string>
#include <tuple>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/common/Size2f.hpp"

namespace dai {
/**
 * CalibrationHandler is an interface to read/load/write structured calibration and device data.
 */
class CalibrationHandler {
   public:
    CalibrationHandler() = default;

    /**
     * @brief Construct a new Calibration Handler object using the
     * eeprom json file created from calibration procedure.
     *
     * @param eepromDataPath - takes the full path to the json file containing the calibration and device info.
     */
    explicit CalibrationHandler(std::string eepromDataPath);

    /**
     * @brief Construct a new Calibration Handler object using the board
     * config json file and .calib binary files created using gen1 calibration.
     *
     * @param calibrationDataPath - Full Path to the .calib binary file from the gen1 calibration. (Supports only Version 5)
     * @param boardConfigPath - Full Path to the board config json file containing device information.
     */
    CalibrationHandler(std::string calibrationDataPath, std::string boardConfigPath);

    /**
     * @brief Construct a new Calibration Handler object from EepromData object.
     *
     * @param eepromData - EepromData data structure containing the calibration data.
     */
    explicit CalibrationHandler(EepromData eepromData);

    /**
     * @brief Get the Eeprom Data object
     *
     * @return dai::EepromData - return the EepromData object which contains the raw calibration data
     */
    dai::EepromData getEepromData() const;

    /**
     * @brief Get the Camera Intrinsics object
     *
     * @param cameraId - Uses the cameraId to identify which camera intrinsics to return
     * @param resizewidth - resized width of the image for which intrinsics is requested.  resizewidth = -1 represents width is same as default intrinsics
     * @param resizeHeight - resized height of the image for which intrinsics is requested.  resizeHeight = -1 represents height is same as default intrinsics
     * @param topLeftPixelId - (x, y) point represents the top left corner coordinates of the cropped image which is used to modify the intrinsics for the
     * respective cropped image
     * @param bottomRightPixelId - (x, y) point represents the bottom right corner coordinates of the cropped image which is used to modify the intrinsics for
     * the respective cropped image
     * @return std::vector<std::vector<float>> - repesents the 3x3 intrinsics matrix of the respective camera at the requested size and crop dimensions.
     */
    std::vector<std::vector<float>> getCameraIntrinsics(
        CameraBoardSocket cameraId, int resizeWidth = -1, int resizeHeight = -1, Point2f topLeftPixelId = Point2f(), Point2f bottomRightPixelId = Point2f());

    /**
     * @brief Get the Camera Intrinsics object
     *
     * @param cameraId - Uses the cameraId to identify which camera intrinsics to return
     * @param destShape - resized width and height of the image for which intrinsics is requested.
     * @param topLeftPixelId - (x, y) point represents the top left corner coordinates of the cropped image which is used to modify the intrinsics for the
     * respective cropped image
     * @param bottomRightPixelId - (x, y) point represents the bottom right corner coordinates of the cropped image which is used to modify the intrinsics for
     * the respective cropped image
     * @return std::vector<std::vector<float>> - repesents the 3x3 intrinsics matrix of the respective camera at the requested size and crop dimensions.
     */
    std::vector<std::vector<float>> getCameraIntrinsics(CameraBoardSocket cameraId,
                                                        Size2f destShape,
                                                        Point2f topLeftPixelId = Point2f(),
                                                        Point2f bottomRightPixelId = Point2f());

    /**
     * @brief Get the Default Intrinsics object
     *
     * @param cameraId
     * @return std::tuple<std::vector<std::vector<float>>, int, int>
     */
    std::tuple<std::vector<std::vector<float>>, int, int> getDefaultIntrinsics(CameraBoardSocket cameraId);

    /**
     * @brief Get the Distortion Coefficients object
     *
     * @param cameraId - Uses the cameraId to identify which distortion Coefficients to return.
     * @return std::vector<float> - repesents the distortion coefficients of the requested camera.
     */
    std::vector<float> getDistortionCoefficients(CameraBoardSocket cameraId);

    /**
     * @brief Get the Camera Extrinsics object between two cameras
     * From the data loaded if there is a linked connection between any two cameras then there relative rotation and translation is returned by this function.
     * @param srcCamera - Camera Id of the camera which will be considerd as origin.
     * @param dstCamera -  Camera Id of the destination camera to which we are fetching the rotation and translation from the SrcCamera
     * @param useMeasuredTranslation - Enabling this bool uses the translation information from the board design data
     * @return std::vector<std::vector<float>> - returns a transformationMatrix which is 4x4 in homogenious coordinate system
     *
     * Matrix representation of transformation matrix
     * transformation matrix = \begin{bmatrix}
     *                              r_00 & r_01 & r_02 & T_x \\
     *                              r_10 & r_11 & r_12 & T_y \\
     *                              r_20 & r_21 & r_22 & T_z
     *                          \end{bmatrix}
     */
    std::vector<std::vector<float>> getCameraExtrinsics(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useMeasuredTranslation = false);

    /**
     * @brief Get the Camera To Imu Extrinsics object
     * From the data loaded if there is a linked connection between IMU and the given camera then there relative rotation and translation from the camera to IMU
     * is returned.
     * @param cameraId - Camera Id of the camera which will be considerd as origin. from which Transformation matrix to the IMU will be found
     * @param useMeasuredTranslation - Enabling this bool uses the translation information from the board design data
     * @return std::vector<std::vector<float>> - returns a transformationMatrix which is 4x4 in homogenious coordinate system
     * Matrix representation of transformation matrix
     * transformation matrix = \begin{bmatrix}
     *                              r_00 & r_01 & r_02 & T_x \\
     *                              r_10 & r_11 & r_12 & T_y \\
     *                              r_20 & r_21 & r_22 & T_z
     *                          \end{bmatrix}
     */
    std::vector<std::vector<float>> getCameraToImuExtrinsics(CameraBoardSocket cameraId, bool useMeasuredTranslation = false);
    // TODO(Sachin): How to check if IMU exists ? set everthing in IMU extrinsics to ZERO ???

    /**
     * @brief Get the Imu To Camera Extrinsics object
     * From the data loaded if there is a linked connection between IMU and the given camera then there relative rotation and translation from the IMU to Camera
     * is returned.
     * @param cameraId - Camera Id of the camera which will be considerd as destination. To which Transformation matrix from the IMU will be found.
     * @param useMeasuredTranslation - Enabling this bool uses the translation information from the board design data
     * @return std::vector<std::vector<float>> - returns a transformationMatrix which is 4x4 in homogenious coordinate system
     * Matrix representation of transformation matrix
     * transformation matrix = \begin{bmatrix}
     *                              r_00 & r_01 & r_02 & T_x \\
     *                              r_10 & r_11 & r_12 & T_y \\
     *                              r_20 & r_21 & r_22 & T_z \\
     *                               0   &  0   &  0   & 1
     *                          \end{bmatrix}
     */
    std::vector<std::vector<float>> getImuToCameraExtrinsics(CameraBoardSocket cameraId, bool useMeasuredTranslation = false);

    // TODO (Sachin) : Fill therse docstrings
    /**
     *
     * @brief Get the Stereo Right Rectification Rotation object
     *
     * @return std::vector<std::vector<float>>
     */
    std::vector<std::vector<float>> getStereoRightRectificationRotation();

    /**
     * @brief Get the Stereo Left Rectification Rotation object
     *
     * @return std::vector<std::vector<float>>
     */
    std::vector<std::vector<float>> getStereoLeftRectificationRotation();
    // getStereoLeftCameraId();
    // getStereoRightCameraId();
    // setCameraExtrinsicsRotation(...)
    // setCameraExtrinsicstranslation(..., setMeasuredTranslation)`
    // setIMUExtrinsicsRotation(...)
    // setIMUExtrinsicstranslation(..., setMeasuredTranslation)`
    // TODO(sachin): Add Q matrix. Since Q matrix is specific to the stereo. may be better to have this over there!!

    /**
     * @brief Write raw calibration/board data to json file.
     *
     * @param destPath -  Full path to the json file in which raw calibration data will be stored
     * @return bool - True on success, false otherwise
     */
    bool eepromToJsonFile(std::string destPath);

    /**
     * @brief Set the Board Info object
     *
     * @param version - Sets the version of the Calibration data(Current version is 6)
     * @param swapLeftRightCam - This bool swaps the connection of left and right camera
     * @param boardName - Sets your board name.
     * @param boardRev - set your board revision id.
     */
    void setBoardInfo(bool swapLeftRightCam, std::string boardName, std::string boardRev);

    /**
     * @brief Set the Camera Intrinsics object
     *
     * @param cameraId - CameraId of the camera for which Camera intrinsics are being loaded
     * @param intrinsics - 3x3 intrinsics matrix in the form of std::vector<std::vector<float>>
     * @param frameSize - repesents the width and height of the image at which intrinsics are calculated.
     *
     * intrinsic matrix = \begin{bmatrix}
     *                              f_x &  0  & c_x \\
     *                              0   & f_y & c_y \\
     *                              0   &  0  &  1
     *                          \end{bmatrix}
     */
    void setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, Size2f frameSize);

    /**
     * @brief Set the Camera Intrinsics object
     *
     * @param cameraId - CameraId of the camera for which Camera intrinsics are being loaded
     * @param intrinsics - 3x3 intrinsics matrix in the form of std::vector<std::vector<float>>
     * @param width - repesents the width of the image at which intrinsics are calculated.
     * @param height - repesents the height of the image at which intrinsics are calculated.
     *
     * intrinsic matrix = \begin{bmatrix}
     *                              f_x &  0  & c_x \\
     *                              0   & f_y & c_y \\
     *                              0   &  0  &  1
     *                          \end{bmatrix}
     */
    void setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, int width, int height);

    /**
     * @brief Sets the distortion Coefficients obtained from camera calibration
     *
     * @param cameraId - Camera Id of the camera for which distoriton coefficients are computed
     * @param distortionCoefficients - Distortion Coefficients of the respective Camera.
     */
    void setdistortionCoefficients(CameraBoardSocket cameraId, std::vector<float> distortionCoefficients);

    /**
     * @brief Set the Fov of the Camera
     *
     * @param cameraId - Camera Id of the camera
     * @param hfov - Horizontal fov of the camera from Camera Datasheet
     */
    void setFov(CameraBoardSocket cameraId, double hfov);

    /**
     * @brief Sets the distortion Coefficients obtained from camera calibration
     *
     * @param cameraId - Camera Id of the camera
     * @param lensPosition - lens posiotion value of the camera at the time of calibration
     */
    void setlensPosition(CameraBoardSocket cameraId, uint8_t lensPosition);

    // /**
    //  * @brief Set the Camera Type object
    //  *
    //  * @param cameraId - CameraId of the camera for which cameraModel Type is being updated.
    //  * @param cameraModel - Type of the model the camera represents
    //  */
    // void setCameraType(CameraBoardSocket cameraId, CameraModel cameraModel);

    /**
     * @brief Set the Camera Extrinsics object
     *
     * @param srcCameraId - Camera Id of the camera which will be considerd as relative origin.
     * @param destCameraId - Camera Id of the camera which will be considerd as destination from srcCameraId.
     * @param rotationMatrix - Rotation between srcCameraId and destCameraId origins.
     * @param translation - Translation between srcCameraId and destCameraId origins.
     * @param measuredTranslation - Translation between srcCameraId and destCameraId origins from the design.
     */
    void setCameraExtrinsics(CameraBoardSocket srcCameraId,
                             CameraBoardSocket destCameraId,
                             std::vector<std::vector<float>> rotationMatrix,
                             std::vector<float> translation,
                             std::vector<float> measuredTranslation = {0, 0, 0});

    /**
     * @brief Set the Imu to Camera Extrinsics object
     *
     * @param destCameraId - Camera Id of the camera which will be considerd as destination from IMU.
     * @param rotationMatrix - Rotation between srcCameraId and destCameraId origins.
     * @param translation - Translation between IMU and destCameraId origins.
     * @param measuredTranslation - Translation between IMU and destCameraId origins from the design.
     */
    void setImuExtrinsics(CameraBoardSocket destCameraId,
                          std::vector<std::vector<float>> rotationMatrix,
                          std::vector<float> translation,
                          std::vector<float> measuredTranslation = {0, 0, 0});

    /**
     * @brief Set the Stereo Left Rectification object
     *
     * @param cameraId - CameraId of the camera which will be used as left Camera of stereo Setup
     * @param rectifiedRotation - Rectification rotation of the left camera required for feature matching
     *
     * Homography of the Left Rectification = Intrinsics_right * rectifiedRotation * inv(Intrinsics_left)
     */
    void setStereoLeft(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation);

    /**
     * @brief Set the Stereo Right Rectification object
     *
     * @param cameraId - CameraId of the camera which will be used as left Camera of stereo Setup
     * @param rectifiedRotation - Rectification rotation of the left camera required for feature matching
     *
     * Homography of the Right Rectification = Intrinsics_right * rectifiedRotation * inv(Intrinsics_right)
     */
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