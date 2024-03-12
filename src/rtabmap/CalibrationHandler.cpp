#include "depthai/device/CalibrationHandler.hpp"
#include "rtabmap/core/StereoCameraModel.h"

namespace dai {
void CalibrationHandler::getRTABMapCameraModel(rtabmap::StereoCameraModel& model, CameraBoardSocket cameraId, int width, int height, float alphaScaling) {
    cv::Mat cameraMatrix;
    std::vector<std::vector<float> > matrix = getCameraIntrinsics(cameraId, width, height);
    cameraMatrix = (cv::Mat_<double>(3, 3) << matrix[0][0],
                    matrix[0][1],
                    matrix[0][2],
                    matrix[1][0],
                    matrix[1][1],
                    matrix[1][2],
                    matrix[2][0],
                    matrix[2][1],
                    matrix[2][2]);
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);
    std::cout << "width " << width << " height " << height << std::endl;
    std::cout << "fx " << fx << " fy " << fy << " cx " << cx << " cy " << cy << std::endl;
    double baseline = getBaselineDistance(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B) / 100.0;
    if(cameraId == dai::CameraBoardSocket::CAM_A)
        model = rtabmap::StereoCameraModel(eepromData.boardName, fx, fy, cx, cy, baseline, rtabmap::Transform::getIdentity(), cv::Size(width, height));
    else
        model = rtabmap::StereoCameraModel(
            eepromData.boardName,
            fx,
            fy,
            cx,
            cy,
            baseline,
            rtabmap::Transform::getIdentity() * rtabmap::Transform(-getBaselineDistance(dai::CameraBoardSocket::CAM_A) / 100.0, 0, 0),
            cv::Size(width, height));

}
}  // namespace dai