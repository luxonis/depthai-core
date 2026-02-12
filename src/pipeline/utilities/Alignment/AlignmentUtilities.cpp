#include "AlignmentUtilities.hpp"

#include <spdlog/async_logger.h>

#include <opencv2/core/types.hpp>

/////////////////////////////////////////// Distortions ///////////////////////////////////////////

std::array<float, 3> applyTilt(float x, float y, float tauX, float tauY) {
    if(tauX == 0.0f && tauY == 0.0f) return {x, y, 1.0f};
    const float cosTx = std::cos(tauX);
    const float sinTx = std::sin(tauX);
    const float cosTy = std::cos(tauY);
    const float sinTy = std::sin(tauY);

    // Rotate around X, then around Y
    const float rx0 = x;
    const float rx1 = cosTx * y - sinTx;
    const float rx2 = sinTx * y + cosTx;

    const float ry0 = cosTy * rx0 + sinTy * rx2;
    const float ry1 = rx1;
    const float ry2 = -sinTy * rx0 + cosTy * rx2;

    if(ry2 == 0.0f) return std::array<float, 3>{ry0, ry1, 1.0f};
    return std::array<float, 3>{ry0 / ry2, ry1 / ry2, 1.0f};
}

std::array<float, 3> distortPerspective(std::array<float, 3> point, const std::vector<float>& coeffs) {
    float x = point[0];
    float y = point[1];

    const float k1 = coeffAt(coeffs, 0);
    const float k2 = coeffAt(coeffs, 1);
    const float p1 = coeffAt(coeffs, 2);
    const float p2 = coeffAt(coeffs, 3);
    const float k3 = coeffAt(coeffs, 4);
    const float k4 = coeffAt(coeffs, 5);
    const float k5 = coeffAt(coeffs, 6);
    const float k6 = coeffAt(coeffs, 7);
    const float s1 = coeffAt(coeffs, 8);
    const float s2 = coeffAt(coeffs, 9);
    const float s3 = coeffAt(coeffs, 10);
    const float s4 = coeffAt(coeffs, 11);
    const float tauX = coeffAt(coeffs, 12);
    const float tauY = coeffAt(coeffs, 13);

    const float r2 = x * x + y * y;
    const float r4 = r2 * r2;
    const float r6 = r4 * r2;
    const float radialNum = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
    const float radialDen = 1.0f + k4 * r2 + k5 * r4 + k6 * r6;
    const float radial = (radialDen != 0.0f) ? (radialNum / radialDen) : radialNum;

    float xDist = x * radial;
    float yDist = y * radial;

    const float xTan = 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
    const float yTan = p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;

    const float xPrism = s1 * r2 + s2 * r4;
    const float yPrism = s3 * r2 + s4 * r4;

    xDist += xTan + xPrism;
    yDist += yTan + yPrism;

    return applyTilt(xDist, yDist, tauX, tauY);
}

inline std::array<float, 3> distortFisheye(std::array<float, 3> point, const std::vector<float>& coeffs) {
    const float x = point[0];
    const float y = point[1];
    const float r = std::sqrt(x * x + y * y);
    if(r == 0.0f) return {x, y, 1.0f};
    const float k1 = coeffAt(coeffs, 0);
    const float k2 = coeffAt(coeffs, 1);
    const float k3 = coeffAt(coeffs, 2);
    const float k4 = coeffAt(coeffs, 3);

    const float theta = std::atan(r);
    const float theta2 = theta * theta;
    const float theta4 = theta2 * theta2;
    const float theta6 = theta4 * theta2;
    const float theta8 = theta4 * theta4;
    const float thetaD = theta * (1.0f + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    const float scale = thetaD / r;
    return {x * scale, y * scale, 1.0f};
}

inline std::array<float, 3> distortRadialDivision(std::array<float, 3> point, const std::vector<float>& coeffs) {
    const float x = point[0];
    const float y = point[1];
    const float k1 = coeffAt(coeffs, 0);
    const float k2 = coeffAt(coeffs, 1);
    const float k3 = coeffAt(coeffs, 2);
    const float r2 = x * x + y * y;
    const float r4 = r2 * r2;
    const float r6 = r4 * r2;
    const float denom = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
    return {x / denom, y / denom, 1.0f};
}

std::array<float, 3> distortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs) {
    if(coeffs.empty() || !hasNonZeroDistortion(coeffs)) return point;
    switch(model) {
        case dai::CameraModel::Perspective:
            return distortPerspective(point, coeffs);
        case dai::CameraModel::Fisheye:
            return distortFisheye(point, coeffs);
        case dai::CameraModel::RadialDivision:
            return distortRadialDivision(point, coeffs);
        case dai::CameraModel::Equirectangular:
        default:
            return point;
    }
}

/////////////////////////////////////////// Undistortions ///////////////////////////////////////////

std::array<float, 3> undistortPerspective(std::array<float, 3> point, const std::vector<float>& coeffs) {
    std::array<float, 3> undistorted = point;
    for(int i = 0; i < 20; ++i) {
        const auto distorted = distortPerspective(undistorted, coeffs);
        const float dx = point[0] - distorted[0];
        const float dy = point[1] - distorted[1];
        undistorted[0] += dx;
        undistorted[1] += dy;
        if(dx * dx + dy * dy < 1e-12f) break;
    }
    return undistorted;
}

std::array<float, 3> undistortFisheye(std::array<float, 3> point, const std::vector<float>& coeffs) {
    std::array<float, 3> undistorted = point;
    for(int i = 0; i < 20; ++i) {
        const auto distorted = distortFisheye(undistorted, coeffs);
        const float dx = point[0] - distorted[0];
        const float dy = point[1] - distorted[1];
        undistorted[0] += dx;
        undistorted[1] += dy;
        if(dx * dx + dy * dy < 1e-12f) break;
    }
    return undistorted;
}

std::array<float, 3> undistortRadialDivision(std::array<float, 3> point, const std::vector<float>& coeffs) {
    std::array<float, 3> undistorted = point;
    const float k1 = coeffAt(coeffs, 0);
    const float k2 = coeffAt(coeffs, 1);
    const float k3 = coeffAt(coeffs, 2);
    for(int i = 0; i < 20; ++i) {
        const float r2 = undistorted[0] * undistorted[0] + undistorted[1] * undistorted[1];
        const float r4 = r2 * r2;
        const float r6 = r4 * r2;
        const float scale = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
        const float x = point[0] * scale;
        const float y = point[1] * scale;
        const float dx = x - undistorted[0];
        const float dy = y - undistorted[1];
        undistorted[0] += dx;
        undistorted[1] += dy;
        if(dx * dx + dy * dy < 1e-12f) break;
    }
    return undistorted;
}

std::array<float, 3> undistortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs) {
    if(coeffs.empty() || !hasNonZeroDistortion(coeffs)) return point;
    switch(model) {
        case dai::CameraModel::Perspective:
            return undistortPerspective(point, coeffs);
        case dai::CameraModel::Fisheye:
            return undistortFisheye(point, coeffs);
        case dai::CameraModel::RadialDivision: {
            const auto undistorted = undistortRadialDivision(point, coeffs);
            // const auto redistorted = distortRadialDivision(undistorted, coeffs);
            // const float dx = redistorted[0] - point[0];
            // const float dy = redistorted[1] - point[1];
            // const float err2 = dx * dx + dy * dy;
            // const float tol2 = 1e-6f * (1.0f + point[0] * point[0] + point[1] * point[1]);
            // if(!std::isfinite(err2) || err2 > tol2) {
            //     return point;
            // }
            return undistorted;
        }
        case dai::CameraModel::Equirectangular:
        default:
            return point;
    }
}

// general

inline float coeffAt(const std::vector<float>& coeffs, size_t idx) {
    return idx < coeffs.size() ? coeffs[idx] : 0.0f;
}

inline bool hasNonZeroDistortion(const std::vector<float>& coeffs) {
    return std::any_of(coeffs.begin(), coeffs.end(), [](float value) { return std::abs(value) > 0.0f; });
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
dai::Point2f opencvPointTransformation(dai::Point2f sourcePt, const dai::ImgTransformation& from, const dai::ImgTransformation& to) {
    const std::array<std::array<float, 3>, 3> fromIntrinsic = from.getSourceIntrinsicMatrix();
    const cv::Matx33f fromIntrinsicsCv = cv::Matx33f(fromIntrinsic[0][0],
                                                     fromIntrinsic[0][1],
                                                     fromIntrinsic[0][2],
                                                     fromIntrinsic[1][0],
                                                     fromIntrinsic[1][1],
                                                     fromIntrinsic[1][2],
                                                     fromIntrinsic[2][0],
                                                     fromIntrinsic[2][1],
                                                     fromIntrinsic[2][2]);
    const std::array<std::array<float, 3>, 3> toIntrinsic = to.getSourceIntrinsicMatrix();
    const cv::Matx33f toIntrinsicsCv = cv::Matx33f(toIntrinsic[0][0],
                                                   toIntrinsic[0][1],
                                                   toIntrinsic[0][2],
                                                   toIntrinsic[1][0],
                                                   toIntrinsic[1][1],
                                                   toIntrinsic[1][2],
                                                   toIntrinsic[2][0],
                                                   toIntrinsic[2][1],
                                                   toIntrinsic[2][2]);

    const std::vector<float> fromDistortionCoeffs = from.getDistortionCoefficients();
    std::vector<cv::Point2f> src = {cv::Point2f(sourcePt.x, sourcePt.y)};
    std::vector<cv::Point2f> transformed;
    switch(from.getDistortionModel()) {
        case dai::CameraModel::Perspective: {
            cv::Mat dist(1, static_cast<int>(fromDistortionCoeffs.size()), CV_32F);
            std::memcpy(dist.ptr<float>(), fromDistortionCoeffs.data(), fromDistortionCoeffs.size() * sizeof(float));
            cv::undistortPoints(src, transformed, fromIntrinsicsCv, dist, cv::noArray(), toIntrinsicsCv, {cv::TermCriteria::MAX_ITER, 50, 0.01});
            break;
        }
        case dai::CameraModel::Fisheye: {
            cv::Mat dist(1, 4, CV_32F);
            dist.at<float>(0, 0) = coeffAt(fromDistortionCoeffs, 0);
            dist.at<float>(0, 1) = coeffAt(fromDistortionCoeffs, 1);
            dist.at<float>(0, 2) = coeffAt(fromDistortionCoeffs, 2);
            dist.at<float>(0, 3) = coeffAt(fromDistortionCoeffs, 3);
            cv::fisheye::undistortPoints(src, transformed, fromIntrinsicsCv, dist, fromIntrinsicsCv, toIntrinsicsCv);
            break;
        }
        case dai::CameraModel::RadialDivision:
        case dai::CameraModel::Equirectangular:
        default:
            transformed = src;
    }

    return transformed.empty() ? sourcePt : dai::Point2f(transformed[0].x, transformed[0].y);
}

#endif
