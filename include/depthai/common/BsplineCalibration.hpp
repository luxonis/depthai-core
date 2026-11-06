#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/** Cubic mrcal splined-stereographic lens. Intrinsics: fx,fy,cx,cy, then
 * interleaved (delta-u-x,delta-u-y), x varying fastest. No Python runtime needed.
 */
struct BsplineCameraCalibration {
    int nx = 0, ny = 0;
    double fovXDegrees = 0;
    std::vector<double> intrinsics;

    void validate() const {
        if(nx < 4 || ny < 4 || nx > 128 || ny > 128 || !std::isfinite(fovXDegrees) || fovXDegrees <= 0 || fovXDegrees >= 180
           || intrinsics.size() != 4 + 2 * static_cast<size_t>(nx) * ny) {
            throw std::invalid_argument("Invalid cubic splined-stereographic dimensions/FOV/parameter count");
        }
        for(double v : intrinsics) if(!std::isfinite(v)) throw std::invalid_argument("Nonfinite spline intrinsics");
        if(intrinsics[0] <= 0 || intrinsics[1] <= 0) throw std::invalid_argument("Nonpositive spline focal length");
    }

    // Call validate() once before projecting a grid. Segment extrapolation matches
    // mrcal 2.5.2, including its boundary convention; coordinates are not clamped.
    std::array<double, 2> project(const std::array<double, 3>& ray) const {
        const double denominator = std::hypot(std::hypot(ray[0], ray[1]), ray[2]) + ray[2];
        if(!std::isfinite(denominator) || denominator <= 0) throw std::invalid_argument("Invalid spline projection ray");
        const double ux = 2 * ray[0] / denominator, uy = 2 * ray[1] / denominator;
        constexpr double pi = 3.14159265358979323846;
        const double density = (nx - 3) / (4 * std::tan(fovXDegrees * pi / 720));
        const double ix = ux * density + (nx - 1) / 2.0, iy = uy * density + (ny - 1) / 2.0;
        if(!std::isfinite(ix) || !std::isfinite(iy)) throw std::invalid_argument("Nonfinite spline grid coordinate");
        // Clamp as double before converting: very large rays must not overflow int.
        const int x0 = static_cast<int>(std::clamp(ix, 1.0, double(nx - 3)));
        const int y0 = static_cast<int>(std::clamp(iy, 1.0, double(ny - 3)));
        auto basis = [](double t) {
            const double t2 = t*t, t3 = t2*t;
            return std::array<double, 4>{{(1-3*t+3*t2-t3)/6, (4-6*t2+3*t3)/6,
                                         (1+3*t+3*t2-3*t3)/6, t3/6}};
        };
        const auto bx = basis(ix-x0), by = basis(iy-y0);
        std::array<double, 2> delta{{0,0}};
        for(int y=0; y<4; ++y) for(int x=0; x<4; ++x) for(int axis=0; axis<2; ++axis) {
            delta[axis] += by[y] * bx[x] * intrinsics[4 + 2*((y0-1+y)*nx + x0-1+x) + axis];
        }
        return {{intrinsics[0]*(ux+delta[0])+intrinsics[2], intrinsics[1]*(uy+delta[1])+intrinsics[3]}};
    }

    DEPTHAI_SERIALIZE(BsplineCameraCalibration, nx, ny, fovXDegrees, intrinsics);
};

/** Versioned native-resolution stereo calibration. Stored persistently in an
 * RVC4 /data extension, not in the limited legacy EEPROM/CBA area. R maps raw
 * camera rays to the rectified camera; virtualIntrinsics is fx,fy,cx,cy.
 * This first version supports horizontal native-resolution stereo only.
 */
struct BsplineStereoCalibration {
    uint32_t version = 2;
    std::string deviceId, calibrationId;
    CameraBoardSocket leftSocket = CameraBoardSocket::AUTO, rightSocket = CameraBoardSocket::AUTO;
    int width = 0, height = 0;
    BsplineCameraCalibration left, right;
    std::vector<double> rotationLeft, rotationRight, virtualIntrinsics;
    double baselineCm = 0;
    // Stereo geometry belongs to this lens model, independently of legacy R/T.
    std::vector<double> leftToRightRotation, leftToRightTranslationCm;
    // Optional registration of the fitted left optical frame to the existing
    // device reference. Reference axes and optical center remain physically fixed.
    CameraBoardSocket referenceSocket = CameraBoardSocket::AUTO;
    std::vector<double> leftToReferenceRotation, leftToReferenceTranslationCm;
    // Optional housing -> fitted left transform (cm), independent of legacy housing.
    std::vector<double> housingToLeftRotation, housingToLeftTranslationCm;

    /** Transform a fitted raw camera frame to the registered device reference.
     * Validates the model and throws if registration is absent
     * or the socket is outside this stereo pair. Translation is in centimeters.
     */
    std::array<std::array<double, 4>, 4> getCameraToReference(CameraBoardSocket socket) const {
        validate();
        if(referenceSocket == CameraBoardSocket::AUTO || (socket != leftSocket && socket != rightSocket)) {
            throw std::invalid_argument("Spline camera has no matching reference registration");
        }
        std::array<std::array<double, 4>, 4> result{};
        for(int i=0;i<3;++i) {
            result[i][3] = leftToReferenceTranslationCm[i];
            for(int j=0;j<3;++j) {
                if(socket == leftSocket) {
                    result[i][j] = leftToReferenceRotation[3*i+j];
                } else {
                    for(int k=0;k<3;++k) result[i][j] += leftToReferenceRotation[3*i+k]*leftToRightRotation[3*j+k];
                    result[i][3] -= result[i][j]*leftToRightTranslationCm[j];
                }
            }
        }
        result[3][3] = 1;
        return result;
    }

    void validate() const {
        left.validate(); right.validate();
        if(version != 2 || deviceId.empty() || calibrationId.empty() || width <= 0 || height <= 0 || width > 8192 || height > 8192
           || leftSocket == CameraBoardSocket::AUTO || rightSocket == CameraBoardSocket::AUTO || leftSocket == rightSocket
           || rotationLeft.size()!=9 || rotationRight.size()!=9 || virtualIntrinsics.size()!=4
           || leftToRightRotation.size()!=9 || leftToRightTranslationCm.size()!=3
           || !std::isfinite(baselineCm) || baselineCm <= 0) throw std::invalid_argument("Invalid B-spline stereo calibration");
        const bool reference = referenceSocket != CameraBoardSocket::AUTO;
        const bool housing = !housingToLeftRotation.empty() || !housingToLeftTranslationCm.empty();
        if((reference && (leftToReferenceRotation.size()!=9 || leftToReferenceTranslationCm.size()!=3))
           || (!reference && (!leftToReferenceRotation.empty() || !leftToReferenceTranslationCm.empty()))
           || (housing && (housingToLeftRotation.size()!=9 || housingToLeftTranslationCm.size()!=3))) {
            throw std::invalid_argument("Incomplete spline reference/housing registration");
        }
        for(const auto* a : {&rotationLeft, &rotationRight, &virtualIntrinsics, &leftToRightRotation, &leftToRightTranslationCm,
                            &leftToReferenceRotation, &leftToReferenceTranslationCm, &housingToLeftRotation, &housingToLeftTranslationCm}) {
            for(double v : *a) if(!std::isfinite(v)) throw std::invalid_argument("Nonfinite B-spline stereo geometry");
        }
        if(virtualIntrinsics[0]<=0 || virtualIntrinsics[1]<=0) throw std::invalid_argument("Nonpositive virtual focal length");
        for(const auto* r : {&rotationLeft, &rotationRight, &leftToRightRotation, &leftToReferenceRotation, &housingToLeftRotation}) {
            if(r->empty()) continue;
            for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
                double dot=0; for(int k=0;k<3;++k) dot+=(*r)[3*i+k]*(*r)[3*j+k];
                if(std::abs(dot-(i==j ? 1.0:0.0))>1e-5) throw std::invalid_argument("Nonorthonormal spline rotation");
            }
            const double det=(*r)[0]*((*r)[4]*(*r)[8]-(*r)[5]*(*r)[7])-(*r)[1]*((*r)[3]*(*r)[8]-(*r)[5]*(*r)[6])+(*r)[2]*((*r)[3]*(*r)[7]-(*r)[4]*(*r)[6]);
            if(std::abs(det-1)>1e-5) throw std::invalid_argument("Improper spline rotation");
        }
        // Rright * RleftToRight == Rleft; Rright*T == (-baseline,0,0).
        for(int i=0;i<3;++i) {
            double t=0; for(int k=0;k<3;++k) t+=rotationRight[3*i+k]*leftToRightTranslationCm[k];
            if(std::abs(t-(i==0 ? -baselineCm:0.0))>1e-4) throw std::invalid_argument("Spline rectification baseline mismatch");
            for(int j=0;j<3;++j) {
                double v=0; for(int k=0;k<3;++k) v+=rotationRight[3*i+k]*leftToRightRotation[3*k+j];
                if(std::abs(v-rotationLeft[3*i+j])>1e-5) throw std::invalid_argument("Spline rectification rotation mismatch");
            }
        }
    }
    DEPTHAI_SERIALIZE(BsplineStereoCalibration, version, deviceId, calibrationId, leftSocket, rightSocket, width, height, left, right,
                      rotationLeft, rotationRight, virtualIntrinsics, baselineCm, leftToRightRotation, leftToRightTranslationCm,
                      referenceSocket, leftToReferenceRotation, leftToReferenceTranslationCm, housingToLeftRotation, housingToLeftTranslationCm);
};
} // namespace dai
