#include "depthai/common/BsplineCalibration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dai {
namespace {

void require(bool condition, const char* message) {
    if(!condition) throw std::invalid_argument(message);
}

bool allFinite(const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

// Element (i, j) of a row-major 3x3 matrix
double at(const std::vector<double>& m, int i, int j) {
    return m[3 * i + j];
}

void requireRotation(const std::vector<double>& r) {
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            double dot = 0;
            for(int k = 0; k < 3; ++k) dot += at(r, i, k) * at(r, j, k);
            require(std::abs(dot - (i == j ? 1.0 : 0.0)) <= 1e-5, "Nonorthonormal spline rotation");
        }
    }
    const double det = at(r, 0, 0) * (at(r, 1, 1) * at(r, 2, 2) - at(r, 1, 2) * at(r, 2, 1))
                       - at(r, 0, 1) * (at(r, 1, 0) * at(r, 2, 2) - at(r, 1, 2) * at(r, 2, 0))
                       + at(r, 0, 2) * (at(r, 1, 0) * at(r, 2, 1) - at(r, 1, 1) * at(r, 2, 0));
    require(std::abs(det - 1) <= 1e-5, "Improper spline rotation");
}

std::array<double, 4> cubicBasis(double t) {
    const double t2 = t * t, t3 = t2 * t;
    return {{(1 - 3 * t + 3 * t2 - t3) / 6, (4 - 6 * t2 + 3 * t3) / 6, (1 + 3 * t + 3 * t2 - 3 * t3) / 6, t3 / 6}};
}

}  // namespace

void BsplineCameraCalibration::validate() const {
    require(nx >= 4 && ny >= 4 && nx <= 128 && ny <= 128 && std::isfinite(fovXDegrees) && fovXDegrees > 0 && fovXDegrees < 180
                && intrinsics.size() == 4 + 2 * static_cast<size_t>(nx) * ny,
            "Invalid cubic splined-stereographic dimensions/FOV/parameter count");
    require(allFinite(intrinsics), "Nonfinite spline intrinsics");
    require(intrinsics[0] > 0 && intrinsics[1] > 0, "Nonpositive spline focal length");
}

std::array<double, 2> BsplineCameraCalibration::project(const std::array<double, 3>& ray) const {
    const double denominator = std::hypot(std::hypot(ray[0], ray[1]), ray[2]) + ray[2];
    require(std::isfinite(denominator) && denominator > 0, "Invalid spline projection ray");
    const double ux = 2 * ray[0] / denominator, uy = 2 * ray[1] / denominator;
    constexpr double pi = 3.14159265358979323846;  // M_PI is not standard (MSVC)
    const double density = (nx - 3) / (4 * std::tan(fovXDegrees * pi / 720));
    const double ix = ux * density + (nx - 1) / 2.0, iy = uy * density + (ny - 1) / 2.0;
    require(std::isfinite(ix) && std::isfinite(iy), "Nonfinite spline grid coordinate");
    // Clamp as double first, so that very large rays cannot overflow int
    const int x0 = static_cast<int>(std::clamp(ix, 1.0, double(nx - 3)));
    const int y0 = static_cast<int>(std::clamp(iy, 1.0, double(ny - 3)));
    const auto bx = cubicBasis(ix - x0), by = cubicBasis(iy - y0);
    std::array<double, 2> delta{{0, 0}};
    for(int y = 0; y < 4; ++y) {
        for(int x = 0; x < 4; ++x) {
            for(int axis = 0; axis < 2; ++axis) delta[axis] += by[y] * bx[x] * intrinsics[4 + 2 * ((y0 - 1 + y) * nx + x0 - 1 + x) + axis];
        }
    }
    return {{intrinsics[0] * (ux + delta[0]) + intrinsics[2], intrinsics[1] * (uy + delta[1]) + intrinsics[3]}};
}

std::array<std::array<double, 4>, 4> BsplineStereoCalibration::getCameraToReference(CameraBoardSocket socket) const {
    validate();
    require(referenceSocket != CameraBoardSocket::AUTO && (socket == leftSocket || socket == rightSocket),
            "Spline camera has no matching reference registration");
    // Left: leftToReference. Right: leftToReference * inverse(leftToRight).
    std::array<std::array<double, 4>, 4> result{};
    for(int i = 0; i < 3; ++i) {
        result[i][3] = leftToReferenceTranslationCm[i];
        for(int j = 0; j < 3; ++j) {
            if(socket == leftSocket) {
                result[i][j] = at(leftToReferenceRotation, i, j);
            } else {
                for(int k = 0; k < 3; ++k) result[i][j] += at(leftToReferenceRotation, i, k) * at(leftToRightRotation, j, k);
                result[i][3] -= result[i][j] * leftToRightTranslationCm[j];
            }
        }
    }
    result[3][3] = 1;
    return result;
}

void BsplineStereoCalibration::validate() const {
    left.validate();
    right.validate();
    require(version == 2 && !deviceId.empty() && !calibrationId.empty() && width > 0 && height > 0 && width <= 8192 && height <= 8192
                && leftSocket != CameraBoardSocket::AUTO && rightSocket != CameraBoardSocket::AUTO && leftSocket != rightSocket && rotationLeft.size() == 9
                && rotationRight.size() == 9 && virtualIntrinsics.size() == 4 && leftToRightRotation.size() == 9 && leftToRightTranslationCm.size() == 3
                && std::isfinite(baselineCm) && baselineCm > 0,
            "Invalid B-spline stereo calibration");
    const bool reference = referenceSocket != CameraBoardSocket::AUTO;
    const bool housing = !housingToLeftRotation.empty() || !housingToLeftTranslationCm.empty();
    require((reference ? leftToReferenceRotation.size() == 9 && leftToReferenceTranslationCm.size() == 3
                       : leftToReferenceRotation.empty() && leftToReferenceTranslationCm.empty())
                && (!housing || (housingToLeftRotation.size() == 9 && housingToLeftTranslationCm.size() == 3)),
            "Incomplete spline reference/housing registration");
    for(const auto* values : {&rotationLeft,
                              &rotationRight,
                              &virtualIntrinsics,
                              &leftToRightRotation,
                              &leftToRightTranslationCm,
                              &leftToReferenceRotation,
                              &leftToReferenceTranslationCm,
                              &housingToLeftRotation,
                              &housingToLeftTranslationCm}) {
        require(allFinite(*values), "Nonfinite B-spline stereo geometry");
    }
    require(virtualIntrinsics[0] > 0 && virtualIntrinsics[1] > 0, "Nonpositive virtual focal length");
    for(const auto* rotation : {&rotationLeft, &rotationRight, &leftToRightRotation, &leftToReferenceRotation, &housingToLeftRotation}) {
        if(!rotation->empty()) requireRotation(*rotation);
    }
    // The rectification must agree with the stereo geometry: Rright * RleftToRight == Rleft, Rright * T == (-baseline, 0, 0)
    for(int i = 0; i < 3; ++i) {
        double t = 0;
        for(int k = 0; k < 3; ++k) t += at(rotationRight, i, k) * leftToRightTranslationCm[k];
        require(std::abs(t - (i == 0 ? -baselineCm : 0.0)) <= 1e-4, "Spline rectification baseline mismatch");
        for(int j = 0; j < 3; ++j) {
            double v = 0;
            for(int k = 0; k < 3; ++k) v += at(rotationRight, i, k) * at(leftToRightRotation, k, j);
            require(std::abs(v - at(rotationLeft, i, j)) <= 1e-5, "Spline rectification rotation mismatch");
        }
    }
}

}  // namespace dai
