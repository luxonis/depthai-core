#include "depthai/rtabmap/RTABMapConversions.hpp"

#include "common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "pipeline/datatype/PointCloudData.hpp"
#include "rtabmap/core/StereoCameraModel.h"
namespace dai {

std::shared_ptr<TransformData> rtabmapToTransformData(rtabmap::Transform transformRTABMap) {
    auto transform = std::make_shared<TransformData>();

    transform->transform.matrix = {{{transformRTABMap.r11(), transformRTABMap.r12(), transformRTABMap.r13(), transformRTABMap.o14()},
                                    {transformRTABMap.r21(), transformRTABMap.r22(), transformRTABMap.r23(), transformRTABMap.o24()},
                                    {transformRTABMap.r31(), transformRTABMap.r32(), transformRTABMap.r33(), transformRTABMap.o34()}}};
    return transform;
}
rtabmap::Transform getRTABMapTransform(const Transform& transform) {
    return rtabmap::Transform(transform.matrix[0][0],
                              transform.matrix[0][1],
                              transform.matrix[0][2],
                              transform.matrix[0][3],
                              transform.matrix[1][0],
                              transform.matrix[1][1],
                              transform.matrix[1][2],
                              transform.matrix[1][3],
                              transform.matrix[2][0],
                              transform.matrix[2][1],
                              transform.matrix[2][2],
                              transform.matrix[2][3]);
}
rtabmap::StereoCameraModel getRTABMapCameraModel(CameraBoardSocket cameraId,
                                                 int width,
                                                 int height,
                                                 const rtabmap::Transform& localTransform,
                                                 float alphaScaling,
                                                 CalibrationHandler calHandler,
                                                 CameraBoardSocket left,
                                                 CameraBoardSocket right) {
    cv::Mat cameraMatrix, distCoeffs, newCameraMatrix;
    std::vector<std::vector<float> > matrix = calHandler.getCameraIntrinsics(cameraId, width, height);
    cameraMatrix = (cv::Mat_<double>(3, 3) << matrix[0][0],
                    matrix[0][1],
                    matrix[0][2],
                    matrix[1][0],
                    matrix[1][1],
                    matrix[1][2],
                    matrix[2][0],
                    matrix[2][1],
                    matrix[2][2]);

    std::vector<float> coeffs = calHandler.getDistortionCoefficients(cameraId);
    if(calHandler.getDistortionModel(cameraId) == dai::CameraModel::Perspective)
        distCoeffs = (cv::Mat_<double>(1, 8) << coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7]);

    if(alphaScaling > -1.0f)
        newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(width, height), alphaScaling);
    else
        newCameraMatrix = cameraMatrix;
    double fx = newCameraMatrix.at<double>(0, 0);
    double fy = newCameraMatrix.at<double>(1, 1);
    double cx = newCameraMatrix.at<double>(0, 2);
    double cy = newCameraMatrix.at<double>(1, 2);
    double baseline = double(calHandler.getBaselineDistance(right, left)) / 100.0;

    return rtabmap::StereoCameraModel(calHandler.getEepromData().boardName, fx, fy, cx, cy, baseline, localTransform, cv::Size(width, height));
}

}  // namespace dai
