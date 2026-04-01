#include "AlignmentUtilities.hpp"

#include <spdlog/async_logger.h>

#include <depthai/utility/matrixOps.hpp>

namespace {

constexpr float kTiny = 1e-12f;

std::array<std::array<float, 3>, 3> makeRotXY(float tauX, float tauY) {
    const float cTx = std::cos(tauX);
    const float sTx = std::sin(tauX);
    const float cTy = std::cos(tauY);
    const float sTy = std::sin(tauY);

    return {{{cTy, sTy * sTx, -sTy * cTx}, {0.0f, cTx, sTx}, {sTy, -cTy * sTx, cTy * cTx}}};
}

std::array<std::array<float, 3>, 3> makeTiltMatrix(float tauX, float tauY) {
    const auto rotXY = makeRotXY(tauX, tauY);
    const float r22 = rotXY[2][2];
    const float r02 = rotXY[0][2];
    const float r12 = rotXY[1][2];

    const std::array<std::array<float, 3>, 3> projZ = {{{r22, 0.0f, -r02}, {0.0f, r22, -r12}, {0.0f, 0.0f, 1.0f}}};
    return dai::matrix::matMul(projZ, rotXY);
}

std::array<std::array<float, 3>, 3> makeInvTiltMatrix(float tauX, float tauY) {
    const auto rotXY = makeRotXY(tauX, tauY);
    const float r22 = rotXY[2][2];
    if(std::abs(r22) < kTiny) {
        return {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    }

    const float invR22 = 1.0f / r22;
    const std::array<std::array<float, 3>, 3> invProjZ = {{{invR22, 0.0f, invR22 * rotXY[0][2]}, {0.0f, invR22, invR22 * rotXY[1][2]}, {0.0f, 0.0f, 1.0f}}};

    const std::array<std::array<float, 3>, 3> rotXYT = {
        {{rotXY[0][0], rotXY[1][0], rotXY[2][0]}, {rotXY[0][1], rotXY[1][1], rotXY[2][1]}, {rotXY[0][2], rotXY[1][2], rotXY[2][2]}}};

    return dai::matrix::matMul(rotXYT, invProjZ);
}

}  // namespace

std::array<float, 3> pixelToRay(dai::Point2f px, const dai::ImgTransformation& transformation) {
    std::array<float, 3> pxHomogeneous = {px.x, px.y, 1.0f};
    auto intrinsicMatrixInv = transformation.getSourceIntrinsicMatrixInv();
    auto distortionModel = transformation.getDistortionModel();
    auto distortionCoeffs = transformation.getDistortionCoefficients();

    std::array<float, 3> pxSensor = dai::matrix::matVecMul(intrinsicMatrixInv, pxHomogeneous);
    std::array<float, 3> undistortedRay = undistortPoint(pxSensor, distortionModel, distortionCoeffs);
    std::array<float, 3> ray = {undistortedRay[0] / undistortedRay[2], undistortedRay[1] / undistortedRay[2], 1.0f};
    return ray;
}

dai::Point2f rayToPixel(const std::array<float, 3>& ray, const dai::ImgTransformation& transformation) {
    auto distortionModel = transformation.getDistortionModel();
    auto distortionCoeffs = transformation.getDistortionCoefficients();
    auto intrinsicMatrix = transformation.getSourceIntrinsicMatrix();
    std::array<float, 3> distortedRay = distortPoint(ray, distortionModel, distortionCoeffs);

    std::array<float, 3> rayHomogeneous = {distortedRay[0] / distortedRay[2], distortedRay[1] / distortedRay[2], 1.0f};
    std::array<float, 3> pxHomogeneous = dai::matrix::matVecMul(intrinsicMatrix, rayHomogeneous);

    return {pxHomogeneous[0] / pxHomogeneous[2], pxHomogeneous[1] / pxHomogeneous[2]};
}

std::array<float, 3> applyTilt(float x, float y, float tauX, float tauY) {
    if(tauX == 0.0f && tauY == 0.0f) return {x, y, 1.0f};
    const auto matTilt = makeTiltMatrix(tauX, tauY);
    const auto tilted = dai::matrix::matVecMul(matTilt, {x, y, 1.0f});
    if(std::abs(tilted[2]) < kTiny) return {tilted[0], tilted[1], 1.0f};
    return {tilted[0] / tilted[2], tilted[1] / tilted[2], 1.0f};
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
    float safeRadialDen = radialDen;
    if(std::abs(safeRadialDen) < kTiny) {
        safeRadialDen = std::copysign(kTiny, safeRadialDen == 0.0f ? 1.0f : safeRadialDen);
    }
    const float radial = radialNum / safeRadialDen;

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

std::array<float, 3> distortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs) {
    if(coeffs.empty() || !hasNonZeroDistortion(coeffs)) return point;
    auto homogeneousPoint = dai::matrix::dehomogenizePoint3(point);
    switch(model) {
        case dai::CameraModel::Perspective:
            return distortPerspective(homogeneousPoint, coeffs);
        case dai::CameraModel::Fisheye:
            return distortFisheye(homogeneousPoint, coeffs);
        case dai::CameraModel::RadialDivision:
            throw std::invalid_argument("Unsupported distortion model: RadialDivision");
        case dai::CameraModel::Equirectangular:
            throw std::invalid_argument("Unsupported distortion model: Equirectangular");
        default:
            throw std::invalid_argument("Unsupported distortion model.");
    }
}

/////////////////////////////////////////// Undistortions ///////////////////////////////////////////

std::array<float, 3> undistortPerspective(std::array<float, 3> point, const std::vector<float>& coeffs) {
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

    float x = point[0];
    float y = point[1];

    if(tauX != 0.0f || tauY != 0.0f) {
        const auto invTilt = makeInvTiltMatrix(tauX, tauY);
        const auto untilted = dai::matrix::matVecMul(invTilt, {x, y, 1.0f});
        if(std::abs(untilted[2]) < kTiny) return point;
        x = untilted[0] / untilted[2];
        y = untilted[1] / untilted[2];
    }

    const float x0 = x;
    const float y0 = y;

    for(int i = 0; i < 50; ++i) {
        const float r2 = x * x + y * y;
        const float r4 = r2 * r2;
        const float r6 = r4 * r2;

        const float num = 1.0f + k4 * r2 + k5 * r4 + k6 * r6;
        const float den = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
        if(std::abs(den) < kTiny) break;
        const float icdist = num / den;
        if(!std::isfinite(icdist) || icdist <= 0.0f) break;

        const float deltaX = 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x) + s1 * r2 + s2 * r4;
        const float deltaY = p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y + s3 * r2 + s4 * r4;

        const float xNew = (x0 - deltaX) * icdist;
        const float yNew = (y0 - deltaY) * icdist;

        const float dx = xNew - x;
        const float dy = yNew - y;
        x = xNew;
        y = yNew;
        if(dx * dx + dy * dy < 1e-14f) break;
    }

    return {x, y, 1.0f};
}

std::array<float, 3> undistortFisheye(std::array<float, 3> point, const std::vector<float>& coeffs) {
    const float x = point[0];
    const float y = point[1];
    const float rd = std::sqrt(x * x + y * y);
    if(rd < kTiny) return {x, y, 1.0f};

    const float k1 = coeffAt(coeffs, 0);
    const float k2 = coeffAt(coeffs, 1);
    const float k3 = coeffAt(coeffs, 2);
    const float k4 = coeffAt(coeffs, 3);

    // Solve theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)
    // with Newton iterations, then map back via r = tan(theta).
    float theta = rd;
    for(int i = 0; i < 40; ++i) {
        const float th2 = theta * theta;
        const float th4 = th2 * th2;
        const float th6 = th4 * th2;
        const float th8 = th4 * th4;

        const float model = theta * (1.0f + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8) - rd;
        const float deriv = 1.0f + 3.0f * k1 * th2 + 5.0f * k2 * th4 + 7.0f * k3 * th6 + 9.0f * k4 * th8;
        if(std::abs(deriv) < kTiny) break;

        const float step = model / deriv;
        theta -= step;
        if(std::abs(step) < 1e-8f) break;
    }

    const float tanTheta = std::tan(theta);
    const float scale = tanTheta / rd;
    return {x * scale, y * scale, 1.0f};
}
std::array<float, 3> undistortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs) {
    if(coeffs.empty() || !hasNonZeroDistortion(coeffs)) return point;
    switch(model) {
        case dai::CameraModel::Perspective:
            return undistortPerspective(point, coeffs);
        case dai::CameraModel::Fisheye:
            return undistortFisheye(point, coeffs);
        case dai::CameraModel::RadialDivision: {
            throw std::invalid_argument("Unsupported distortion model: RadialDivision");
        }
        case dai::CameraModel::Equirectangular: {
            throw std::invalid_argument("Unsupported distortion model: Equirectangular");
        }
        default:
            throw std::invalid_argument("Unsupported distortion model");
    }
}

/////////////////////////////////////////// general ////////////////////////////////////////////

inline float coeffAt(const std::vector<float>& coeffs, size_t idx) {
    return idx < coeffs.size() ? coeffs[idx] : 0.0f;
}

inline bool hasNonZeroDistortion(const std::vector<float>& coeffs) {
    return std::any_of(coeffs.begin(), coeffs.end(), [](float value) { return std::abs(value) > 0.0f; });
}
