#include "RTABMapConversions.hpp"

#include <cmath>
#include <stdexcept>

#include "common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/Odometry.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/Transform.h"
namespace dai {

namespace {
void validateRectifiedFrame(const ImgFrame& frame) {
    const auto& transformation = frame.transformation;
    const auto k = transformation.getIntrinsicMatrix();
    if(frame.getWidth() == 0 || frame.getHeight() == 0 || transformation.getSize() != std::make_pair(size_t(frame.getWidth()), size_t(frame.getHeight()))) {
        throw std::invalid_argument("RTABMapSLAM: frame dimensions must match image transformation metadata.");
    }
    for(const auto& row : k) {
        for(const auto value : row) {
            if(!std::isfinite(value)) throw std::invalid_argument("RTABMapSLAM: non-finite frame intrinsics.");
        }
    }
    if(k[0][0] <= 0 || k[1][1] <= 0 || std::abs(k[0][1]) > 1e-6f || std::abs(k[1][0]) > 1e-6f || std::abs(k[2][0]) > 1e-6f || std::abs(k[2][1]) > 1e-6f
       || std::abs(k[2][2] - 1) > 1e-6f || (k[0][0] == 1 && k[1][1] == 1 && k[0][2] == 0 && k[1][2] == 0)) {
        throw std::invalid_argument("RTABMapSLAM: expected calibrated, positive-focal pinhole intrinsics.");
    }
    if(transformation.getDistortionModel() != CameraModel::Perspective) {
        throw std::invalid_argument("RTABMapSLAM: expected rectified perspective images.");
    }
    for(const auto value : transformation.getDistortionCoefficients()) {
        if(!std::isfinite(value) || std::abs(value) > 1e-6f) {
            throw std::invalid_argument("RTABMapSLAM: input images must already be rectified.");
        }
    }
}
}  // namespace

rtabmap::CameraModel getRTABMapFrameCameraModel(const ImgFrame& image, const ImgFrame& depth, const rtabmap::Transform& referenceToBase) {
    validateRectifiedFrame(image);
    validateRectifiedFrame(depth);
    const auto k = image.transformation.getIntrinsicMatrix();
    const auto depthK = depth.transformation.getIntrinsicMatrix();
    // RTAB-Map supports a lower-resolution depth image covering the same view.
    const double sx = double(depth.getWidth()) / image.getWidth();
    const double sy = double(depth.getHeight()) / image.getHeight();
    if(image.getWidth() % depth.getWidth() != 0 || image.getHeight() % depth.getHeight() != 0 || std::abs(sx - sy) > 1e-6) {
        throw std::invalid_argument("RTABMapSLAM: depth must have the image's resolution or a uniform integer decimation.");
    }
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            const double scale = row == 0 ? sx : (row == 1 ? sy : 1.0);
            if(std::abs(depthK[row][col] - scale * k[row][col]) > 0.01) {
                throw std::invalid_argument("RTABMapSLAM: depth and rectified image projections do not match. Align depth to the rectified image.");
            }
        }
    }
    const auto imageExtrinsics = image.transformation.getExtrinsics();
    const auto depthExtrinsics = depth.transformation.getExtrinsics();
    for(const auto& extrinsics : {imageExtrinsics, depthExtrinsics}) {
        if(!isConvertibleLengthUnit(extrinsics.lengthUnit)) {
            throw std::invalid_argument("RTABMapSLAM: frame extrinsics require a known metric length unit.");
        }
        if(!extrinsics.hasValidRotationMatrix() || extrinsics.toCameraSocket == CameraBoardSocket::AUTO) {
            throw std::invalid_argument("RTABMapSLAM: valid frame extrinsics with a named reference camera are required.");
        }
        for(const auto value : extrinsics.getTranslationVector(false, LengthUnit::METER)) {
            if(!std::isfinite(value)) throw std::invalid_argument("RTABMapSLAM: non-finite frame translation.");
        }
    }
    if(!imageExtrinsics.isEqualExtrinsics(depthExtrinsics, 1e-4f)) {
        throw std::invalid_argument("RTABMapSLAM: depth and rectified image must have the same optical frame.");
    }
    // The image already carries its optical-frame-to-reference transform.
    // The configured local transform maps that named reference into the odometry body.
    Transform imageToReference;
    const auto relative = imageExtrinsics.getTransformationMatrix(false, LengthUnit::METER);
    for(size_t row = 0; row < 4; ++row) {
        for(size_t col = 0; col < 4; ++col) {
            imageToReference.matrix[row][col] = relative[row][col];
        }
    }
    return rtabmap::CameraModel(
        k[0][0], k[1][1], k[0][2], k[1][2], referenceToBase * getRTABMapTransform(imageToReference), 0.0, cv::Size(image.getWidth(), image.getHeight()));
}

std::shared_ptr<TransformData> rtabmapToTransformData(rtabmap::Transform transformRTABMap) {
    auto transform = std::make_shared<TransformData>();

    transform->transform.matrix = {{{transformRTABMap.r11(), transformRTABMap.r12(), transformRTABMap.r13(), transformRTABMap.o14()},
                                    {transformRTABMap.r21(), transformRTABMap.r22(), transformRTABMap.r23(), transformRTABMap.o24()},
                                    {transformRTABMap.r31(), transformRTABMap.r32(), transformRTABMap.r33(), transformRTABMap.o34()},
                                    {0.0, 0.0, 0.0, 1.0}}};
    return transform;
}

std::shared_ptr<Odometry> rtabmapToOdometry(rtabmap::Transform transformRTABMap, const Point3d& velocity) {
    auto odometry = std::make_shared<Odometry>();

    odometry->transform.matrix = {{{transformRTABMap.r11(), transformRTABMap.r12(), transformRTABMap.r13(), transformRTABMap.o14()},
                                   {transformRTABMap.r21(), transformRTABMap.r22(), transformRTABMap.r23(), transformRTABMap.o24()},
                                   {transformRTABMap.r31(), transformRTABMap.r32(), transformRTABMap.r33(), transformRTABMap.o34()},
                                   {0.0, 0.0, 0.0, 1.0}}};
    odometry->velocity = velocity;
    return odometry;
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
