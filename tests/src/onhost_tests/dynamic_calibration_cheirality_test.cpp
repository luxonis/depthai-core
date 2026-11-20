#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <cmath>
#include <cstdint>
#include <depthai/depthai.hpp>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// After the depthai headers: the DCL headers define macros that collide with depthai enumerators.
#include <DynamicCalibration.hpp>

#include "../../../src/pipeline/node/DynamicCalibrationUtils.hpp"

// The calibration library has to refuse a stereo pose that reconstructs the scene behind the cameras (the
// cheirality check). The tests render synthetic image pairs of a known scene for a known stereo pair, run them
// through the same DCL entry point the DynamicCalibration node uses and compare the answer with the truth.
//
// Camera A sits at the origin and looks along +z; camera B is 7.5 cm to its right with the same orientation.
// The pose of a camera is the transform from camera A's frame to its own: p_cam = R * p_A + t.

namespace {

constexpr int kWidth = 640;
constexpr int kHeight = 400;
constexpr double kFocal = 400.0;
constexpr double kCx = 320.0;
constexpr double kCy = 200.0;
constexpr double kBaseline = 0.075;
constexpr double kWallDistance = 0.5;
constexpr double kPi = 3.14159265358979323846;

using Vec3 = std::array<double, 3>;

struct Pose {
    Vec3 rvec{0.0, 0.0, 0.0};
    Vec3 tvec{0.0, 0.0, 0.0};
};

const Pose kTruePoseB{{0.0, 0.0, 0.0}, {-kBaseline, 0.0, 0.0}};

double toDegrees(double radians) {
    return radians * 180.0 / kPi;
}

double latticeValue(int64_t x, int64_t y, uint32_t seed) {
    uint32_t h = static_cast<uint32_t>(x) * 374761393U + static_cast<uint32_t>(y) * 668265263U + seed * 2246822519U;
    h = (h ^ (h >> 13U)) * 1274126177U;
    h ^= h >> 16U;
    return static_cast<double>(h & 0xFFFFU) / 65535.0;
}

double valueNoise(double u, double v, uint32_t seed) {
    const double floorU = std::floor(u);
    const double floorV = std::floor(v);
    double fu = u - floorU;
    double fv = v - floorV;
    fu = fu * fu * (3.0 - 2.0 * fu);
    fv = fv * fv * (3.0 - 2.0 * fv);
    const auto iu = static_cast<int64_t>(floorU);
    const auto iv = static_cast<int64_t>(floorV);
    const double top = latticeValue(iu, iv, seed) * (1.0 - fu) + latticeValue(iu + 1, iv, seed) * fu;
    const double bottom = latticeValue(iu, iv + 1, seed) * (1.0 - fu) + latticeValue(iu + 1, iv + 1, seed) * fu;
    return top * (1.0 - fv) + bottom * fv;
}

/// A random texture with detail from 8 cm down to 5 mm, as a function of the position on a surface in meters.
double texture(double x, double y, uint32_t seed) {
    double value = 0.0;
    double amplitude = 1.0;
    double total = 0.0;
    double cell = 0.08;
    for(uint32_t octave = 0; octave < 5; ++octave) {
        value += amplitude * valueNoise(x / cell + 100.0, y / cell + 100.0, seed + octave);
        total += amplitude;
        amplitude *= 0.6;
        cell *= 0.5;
    }
    return value / total;
}

enum class Scene {
    /// One textured wall, 0.5 m away and parallel to the image planes.
    Wall,
    /// A textured wall 2 m away with square panels floating 1.2 m and 0.6 m in front of the cameras.
    Panels,
};

/// Intensity seen along the ray origin + s * direction, both in camera A's frame.
double shade(Scene scene, const Vec3& origin, const Vec3& direction) {
    const auto hit = [&](double z) {
        const double s = (z - origin[2]) / direction[2];
        return std::array<double, 2>{origin[0] + s * direction[0], origin[1] + s * direction[1]};
    };
    if(scene == Scene::Wall) {
        const auto point = hit(kWallDistance);
        return texture(point[0], point[1], 1);
    }
    // Nearest surface first.
    struct Layer {
        double z;
        uint32_t seed;
        /// Panels on every other tile; otherwise on every third column of every other row.
        bool checker;
    };
    for(const Layer layer : {Layer{0.6, 13, false}, Layer{1.2, 7, true}}) {
        const auto point = hit(layer.z);
        const double tile = 0.25 * layer.z;
        const double tileX = std::floor(point[0] / tile);
        const double tileY = std::floor(point[1] / tile);
        const auto column = static_cast<int64_t>(tileX);
        const auto row = static_cast<int64_t>(tileY);
        const bool occupied = layer.checker ? ((column + row) % 2 == 0) : (column % 3 == 0 && row % 2 == 0);
        const double inTileX = point[0] / tile - tileX;
        const double inTileY = point[1] / tile - tileY;
        if(occupied && inTileX > 0.15 && inTileX < 0.85 && inTileY > 0.15 && inTileY < 0.85) {
            return texture(point[0] * 0.5 / layer.z, point[1] * 0.5 / layer.z, layer.seed);
        }
    }
    const auto point = hit(2.0);
    return texture(point[0] * 0.4, point[1] * 0.4, 1);
}

/// The scene as seen by a camera at `center` (camera A's frame) that looks along +z like camera A.
dcl::ImageData render(Scene scene, const Vec3& center) {
    std::vector<double> intensity(static_cast<size_t>(kWidth) * kHeight);
    double lowest = std::numeric_limits<double>::max();
    double highest = std::numeric_limits<double>::lowest();
    for(int v = 0; v < kHeight; ++v) {
        for(int u = 0; u < kWidth; ++u) {
            const double value = shade(scene, center, {(u - kCx) / kFocal, (v - kCy) / kFocal, 1.0});
            intensity[static_cast<size_t>(v) * kWidth + u] = value;
            lowest = std::min(lowest, value);
            highest = std::max(highest, value);
        }
    }
    dcl::ImageData image;
    image.width = kWidth;
    image.height = kHeight;
    image.format = dcl::DCL_8UC1;
    image.data.resize(intensity.size());
    for(size_t idx = 0; idx < intensity.size(); ++idx) {
        image.data[idx] = static_cast<uint8_t>(std::lround(255.0 * (intensity[idx] - lowest) / (highest - lowest)));
    }
    return image;
}

std::shared_ptr<dcl::CameraCalibrationHandle> calibrationHandle(const Pose& pose) {
    const dcl::scalar_t rotation[3] = {pose.rvec[0], pose.rvec[1], pose.rvec[2]};
    const dcl::scalar_t translation[3] = {pose.tvec[0], pose.tvec[1], pose.tvec[2]};
    const dcl::scalar_t cameraMatrix[9] = {kFocal, 0.0, kCx, 0.0, kFocal, kCy, 0.0, 0.0, 1.0};
    dcl::scalar_t distortion[14] = {0.0};
    return std::make_shared<dcl::CameraCalibrationHandle>(rotation, translation, cameraMatrix, dcl::PerspectiveDistortion::fromArray(distortion));
}

/// The second pose that explains a wall at kWallDistance as well as the true one: its planar twin.
///
/// The two images of a plane are related by the homography H = R + t * n^T / d. For the true pose H is the identity
/// plus a shear, a = -kBaseline / kWallDistance, in the x-z plane. A second rotation about the y axis, by
/// 2 * atan(a / 2), leaves H - R of rank one as well, so H also splits into that rotation and a translation along
/// (sin(angle / 2), 0, cos(angle / 2)). Every match on the wall satisfies the epipolar constraint of this pose
/// exactly, yet its baseline points along the optical axis and it reconstructs part of the wall behind the cameras.
Pose planarTwinPoseB(double sign) {
    const double angle = 2.0 * std::atan(-kBaseline / kWallDistance / 2.0);
    return {{0.0, angle, 0.0}, {sign * kBaseline * std::sin(angle / 2.0), 0.0, sign * kBaseline * std::cos(angle / 2.0)}};
}

struct Outcome {
    unsigned int errorCode = 0;
    std::string errorMessage;
    /// Angle between the returned and the true relative rotation, in degrees.
    double rotationErrorDegrees = 0.0;
    /// Angle between the returned and the true baseline direction, in degrees.
    double baselineTurnDegrees = 0.0;
    double baselineLength = 0.0;
};

/// Recalibrates the pair from one rendered frame of the scene, the way the DynamicCalibration node does: one image
/// set loaded, then the multi-sensor findNewCalibration with all checks on the amount of data skipped.
Outcome recalibrate(Scene scene, const Pose& registeredPoseB, bool keepCameraCenters) {
    dcl::DynamicCalibration dynamicCalibration;
    const auto device = dynamicCalibration.addDevice();
    const dcl::Resolution resolution{kWidth, kHeight};
    const auto sensorA = dynamicCalibration.addSensor(device, calibrationHandle(Pose{}), resolution, 1);
    const auto sensorB = dynamicCalibration.addSensor(device, calibrationHandle(registeredPoseB), resolution, 2);

    const auto loaded = dynamicCalibration.loadStereoImagePair(render(scene, {0.0, 0.0, 0.0}), render(scene, {kBaseline, 0.0, 0.0}), sensorA, sensorB, 1000);
    REQUIRE(loaded.passed());

    auto result = dynamicCalibration.findNewCalibration({sensorA, sensorB}, dcl::PerformanceMode::SKIP_CHECKS, keepCameraCenters);
    Outcome outcome;
    outcome.errorCode = result.errorCode;
    outcome.errorMessage = result.errorMessage();
    if(!result.passed()) {
        return outcome;
    }
    REQUIRE(result.value.calibrations.size() == 2);

    // T_B * T_A^-1: the pose of camera B relative to camera A. The true one has no rotation and its baseline along -x.
    using dai::node::DclUtils;
    auto inverseA = DclUtils::calibrationHandleToTransform(result.value.calibrations[0]);
    dai::matrix::invertSe3Matrix4x4InPlace(inverseA);
    const auto relative = dai::matrix::matMul(DclUtils::calibrationHandleToTransform(result.value.calibrations[1]), inverseA);
    const auto rvec = dai::matrix::rotationMatrixToVector(dai::matrix::extractRotationMatrix(relative));
    const auto t = dai::matrix::extractTranslationVector(relative);
    outcome.rotationErrorDegrees = toDegrees(std::hypot(rvec[0], rvec[1], rvec[2]));
    outcome.baselineLength = std::hypot(t[0], t[1], t[2]);
    outcome.baselineTurnDegrees = toDegrees(std::acos(std::clamp(-t[0] / outcome.baselineLength, -1.0, 1.0)));
    return outcome;
}

void requireTruePose(const Outcome& outcome) {
    INFO("DCL error: " << outcome.errorMessage);
    REQUIRE(outcome.errorCode == 0);
    REQUIRE(outcome.rotationErrorDegrees < 0.05);
    REQUIRE(outcome.baselineTurnDegrees < 1.0);
    REQUIRE(outcome.baselineLength == Catch::Approx(kBaseline).epsilon(0.01));
}

/// A calibration a device could carry after a knock: camera B turned by 0.3 and 0.4 degrees about x and y.
const Pose kKnockedPoseB{{0.3 * kPi / 180.0, 0.4 * kPi / 180.0, 0.0}, kTruePoseB.tvec};

}  // namespace

TEST_CASE("DynamicCalibration rejects the planar twin of a wall", "[DynamicCalibration][Cheirality]") {
    // The solve starts on the twin and stays there: the twin fits every match on the wall exactly. Without the
    // cheirality check it comes back as a valid calibration with the baseline turned by about 94 degrees.
    const double sign = GENERATE(1.0, -1.0);
    const auto outcome = recalibrate(Scene::Wall, planarTwinPoseB(sign), true);
    INFO("Baseline turned by " << outcome.baselineTurnDegrees << " deg, rotation off by " << outcome.rotationErrorDegrees << " deg");
    REQUIRE(outcome.errorCode == FINDNEWCALIBRATION_CHEIRALITY_FAILED);
    // The node forwards this text in DynamicCalibrationResult::info.
    REQUIRE(outcome.errorMessage.find("in front of both cameras") != std::string::npos);
}

TEST_CASE("DynamicCalibration recovers the pose from a wall when it starts near the truth", "[DynamicCalibration][Cheirality]") {
    requireTruePose(recalibrate(Scene::Wall, kKnockedPoseB, true));
}

TEST_CASE("DynamicCalibration never returns the planar twin from a RANSAC start", "[DynamicCalibration][Cheirality]") {
    // With the camera centers free the solve starts from a RANSAC estimate. On a wall that estimate lands on the true
    // pose or on the twin depending on the platform, so the test accepts the true pose or the rejection, never the twin.
    const auto outcome = recalibrate(Scene::Wall, kKnockedPoseB, false);
    if(outcome.errorCode == FINDNEWCALIBRATION_CHEIRALITY_FAILED) {
        SUCCEED("The twin was rejected");
    } else {
        requireTruePose(outcome);
    }
}

TEST_CASE("DynamicCalibration cheirality check accepts a scene with depth", "[DynamicCalibration][Cheirality]") {
    const bool keepCameraCenters = GENERATE(true, false);
    requireTruePose(recalibrate(Scene::Panels, kKnockedPoseB, keepCameraCenters));
}
