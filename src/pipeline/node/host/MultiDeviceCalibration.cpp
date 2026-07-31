#include "depthai/pipeline/node/host/MultiDeviceCalibration.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <DynamicCalibration.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/MultiDeviceCalibrationResult.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/DynamicCalibrationUtils.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

namespace {

using Transform = std::vector<std::vector<float>>;

Transform identityTransform() {
    return {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
}

Transform inverted(Transform transform) {
    matrix::invertSe3Matrix4x4InPlace(transform);
    return transform;
}

/// Center of the camera whose pose w.r.t. the reference frame is `transform`, expressed in the reference frame.
std::vector<float> cameraCenter(const Transform& transform) {
    const auto rotation = matrix::extractRotationMatrix(transform);
    const auto translation = matrix::extractTranslationVector(transform);
    std::vector<float> center(3, 0.0f);
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            center[i] -= rotation[j][i] * translation[j];
        }
    }
    return center;
}

float dot(const std::vector<float>& a, const std::vector<float>& b) {
    return std::inner_product(a.begin(), a.end(), b.begin(), 0.0f);
}

std::string frameKey(const CoordinateFrame& frame) {
    return frame.deviceId + "_" + toString(frame.socket);
}

/// A camera stream the rig is estimated from.
struct CameraStream {
    CoordinateFrame frame;
    std::string inputName;
    /// Pose of this camera w.r.t. the frame the whole rig is expressed in, in meters. Initial guess, refined by DCL.
    Transform toRigBase = identityTransform();
    std::pair<unsigned, unsigned> resolution{0, 0};
    ImgTransformation transformation;
    std::shared_ptr<dcl::CameraSensorHandle> sensor;
};

}  // namespace

class MultiDeviceCalibration::Impl {
   public:
    dcl::DynamicCalibration dcl;
    std::shared_ptr<dcl::Device> dclDevice;

    std::vector<CameraStream> cameras;
    std::vector<RigEdge> initialGuesses;
    /// Known distances between camera centers, in meters, keyed by the frame pair.
    std::map<std::pair<CoordinateFrame, CoordinateFrame>, float> knownDistances;
    /// Calibration of every device involved, read from the live devices.
    std::map<std::string, CalibrationHandler> calibrations;
    /// Frame every device is represented by in the rig, i.e. its first registered camera.
    std::map<std::string, CoordinateFrame> deviceReference;

    size_t sampleCount = 10;
    bool continuous = false;
    DynamicCalibrationControl::PerformanceMode performanceMode = DynamicCalibrationControl::PerformanceMode::DEFAULT;
    MultiDeviceCalibration::Method method = MultiDeviceCalibration::Method::DYNAMIC_CALIBRATION;
    /// Synchronized image sets accumulated for the FEATURE_TRACKS method, per registered camera.
    std::vector<std::vector<std::shared_ptr<ImgFrame>>> trackFrames;

    /// Frame the whole rig is expressed in, i.e. the first registered camera.
    CoordinateFrame baseFrame;
    /// Try several strategies and keep the best-scoring one instead of trusting a single solve.
    bool autoStrategy = true;
    /// Yaw perturbations (degrees) of the initial guess explored by the automatic strategy.
    std::vector<float> guessYawOffsets = {0.0f, -30.0f, 30.0f, -60.0f, 60.0f};

    std::optional<float> findKnownDistance(const CoordinateFrame& a, const CoordinateFrame& b) const {
        const auto direct = knownDistances.find({a, b});
        if(direct != knownDistances.end()) return direct->second;
        const auto reverse = knownDistances.find({b, a});
        if(reverse != knownDistances.end()) return reverse->second;
        return std::nullopt;
    }

    /// Pose of camera `frame` w.r.t. the frame its device is represented by, in meters.
    Transform referenceToCamera(const CoordinateFrame& frame) const {
        const auto& reference = deviceReference.at(frame.deviceId);
        if(reference == frame) return identityTransform();
        return calibrations.at(frame.deviceId).getCameraExtrinsics(reference.socket, frame.socket, false, LengthUnit::METER);
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /// Initial guesses with an extra yaw rotation (about the reference vertical axis) applied to `deviceId`'s edge.
    MultiDeviceCalibrationHandler guessesWithYaw(const std::string& deviceId, float yawDegrees) const {
        auto edges = initialGuesses;
        if(yawDegrees != 0.0f) {
            const float yaw = yawDegrees * 0.017453292519943295f;  // pi / 180
            const Transform rotate = {
                {std::cos(yaw), 0.0f, std::sin(yaw), 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {-std::sin(yaw), 0.0f, std::cos(yaw), 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
            for(auto& edge : edges) {
                if(edge.from.deviceId != deviceId) continue;
                const auto current = matrix::toVecMatrix4x4(edge.transform.getTransformationMatrix(false, LengthUnit::CENTIMETER));
                const auto rotated = matrix::matMul(rotate, current);
                edge.transform.setTransformationMatrix(rotated, LengthUnit::CENTIMETER);
                edge.transform.setReferenceFrame(edge.to);
            }
        }
        return MultiDeviceCalibrationHandler(MultiDeviceCalibrationData{1, 0, edges, {}});
    }

    /// Pose of camera `frame` w.r.t. the rig base, composed from the (possibly perturbed) guesses, in meters.
    Transform seedToRigBase(const CoordinateFrame& frame, const MultiDeviceCalibrationHandler& guesses) const {
        const auto& reference = deviceReference.at(frame.deviceId);
        Transform referenceToRigBase = identityTransform();
        if(reference != baseFrame) {
            referenceToRigBase = matrix::toVecMatrix4x4(guesses.getTransform(reference, baseFrame, LengthUnit::METER));
        }
        return matrix::matMul(referenceToCamera(frame), inverted(referenceToRigBase));
    }

    /// Re-seed the DCL calibration of a camera with a new pose w.r.t. the rig base.
    void reseed(const CameraStream& camera, const Transform& toRigBase) {
        const auto calibration = DclUtils::createDclCalibration(camera.transformation.getIntrinsicMatrix(),
                                                                camera.transformation.getDistortionCoefficients(),
                                                                matrix::extractRotationMatrix(toRigBase),
                                                                matrix::extractTranslationVector(toRigBase),
                                                                camera.transformation.getDistortionModel());
        dcl.setCalibration(camera.sensor, calibration);
    }

    /// Known metric baselines within a subset of cameras, indexed by their position in the subset.
    std::vector<dcl::EdgeBaseline> baselineEdges(const std::vector<size_t>& subset) const {
        std::vector<dcl::EdgeBaseline> edges;
        for(size_t i = 0; i < subset.size(); ++i) {
            for(size_t j = i + 1; j < subset.size(); ++j) {
                const auto& a = cameras[subset[i]].frame;
                const auto& b = cameras[subset[j]].frame;
                std::optional<float> distance = findKnownDistance(a, b);
                if(!distance.has_value() && a.deviceId == b.deviceId) {
                    distance = calibrations.at(a.deviceId).getBaselineDistance(a.socket, b.socket, true, LengthUnit::METER);
                }
                if(!distance.has_value() || *distance <= 0.0f) continue;
                edges.push_back({i, j, static_cast<double>(*distance)});
            }
        }
        return edges;
    }
#endif
};

MultiDeviceCalibration::MultiDeviceCalibration() : pimpl(spimpl::make_unique_impl<Impl>()) {}
MultiDeviceCalibration::~MultiDeviceCalibration() = default;

void MultiDeviceCalibration::buildInternal() {
    sync->out.link(syncInput);
}

void MultiDeviceCalibration::addCamera(const CoordinateFrame& frame, Node::Output& source) {
    DAI_CHECK_V(frame.isQualified(), "MultiDeviceCalibration cameras must specify both a device id and a camera socket, got {}", toString(frame));
    const auto duplicate = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&frame](const CameraStream& camera) { return camera.frame == frame; });
    DAI_CHECK_V(duplicate == pimpl->cameras.end(), "Camera {} was already registered on this MultiDeviceCalibration node", toString(frame));

    CameraStream camera;
    camera.frame = frame;
    camera.inputName = frameKey(frame);
    pimpl->cameras.push_back(camera);

    auto& input = inputs[camera.inputName];
    input.setBlocking(false);
    input.setMaxSize(2);
    source.link(input);
}

std::shared_ptr<MultiDeviceCalibration> MultiDeviceCalibration::build(const std::vector<std::pair<CoordinateFrame, Node::Output*>>& sources) {
    for(const auto& [frame, source] : sources) {
        DAI_CHECK_V(source != nullptr, "MultiDeviceCalibration source for {} is null", toString(frame));
        addCamera(frame, *source);
    }
    return std::static_pointer_cast<MultiDeviceCalibration>(shared_from_this());
}

void MultiDeviceCalibration::setInitialGuess(const CoordinateFrame& from, const CoordinateFrame& to, const Extrinsics& guess) {
    RigEdge edge;
    edge.from = from;
    edge.to = to;
    edge.transform = guess;
    edge.transform.setReferenceFrame(to);
    edge.source = "initial-guess";
    pimpl->initialGuesses.push_back(edge);
}

void MultiDeviceCalibration::setKnownDistance(const CoordinateFrame& from, const CoordinateFrame& to, float distance, LengthUnit unit) {
    DAI_CHECK_V(distance > 0.0f, "MultiDeviceCalibration known distance must be positive, got {}", distance);
    pimpl->knownDistances[{from, to}] = distance * getDistanceUnitScale(LengthUnit::METER, unit);
}

void MultiDeviceCalibration::setSampleCount(size_t sampleCount) {
    DAI_CHECK_V(sampleCount >= 1, "MultiDeviceCalibration needs at least one image set, got {}", sampleCount);
    pimpl->sampleCount = sampleCount;
}

void MultiDeviceCalibration::setContinuous(bool continuous) {
    pimpl->continuous = continuous;
}

void MultiDeviceCalibration::setPerformanceMode(DynamicCalibrationControl::PerformanceMode mode) {
    pimpl->performanceMode = mode;
    pimpl->autoStrategy = false;
}

void MultiDeviceCalibration::setAutoStrategy(bool enable) {
    pimpl->autoStrategy = enable;
}

void MultiDeviceCalibration::setGuessYawSweep(const std::vector<float>& offsetsDegrees) {
    pimpl->guessYawOffsets = offsetsDegrees.empty() ? std::vector<float>{0.0f} : offsetsDegrees;
}

void MultiDeviceCalibration::setMethod(Method method) {
    pimpl->method = method;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

}  // namespace node
}  // namespace dai

    #include <opencv2/calib3d.hpp>
    #include <opencv2/core.hpp>
    #include <opencv2/features2d.hpp>
    #include <opencv2/imgproc.hpp>

namespace dai {
namespace node {

namespace {

// Target-free relative-pose estimation ported from luxonis/multicamera_calibration. All linear algebra is done with
// double precision cv::Mat / cv::Vec to mirror the reference implementation closely.

/// Matching thresholds of the feature-tracks method.
struct FeatureConfig {
    double ratioTest = 0.8;
    double stereoMaxReprojErrorPx = 1.0;
    double crossRansacReprojErrorPx = 2.0;
};

/// Intrinsics, distortion and metric stereo pose of one device.
struct StereoGeometry {
    cv::Mat kLeft, distLeft, kRight, distRight;  // CV_64F
    cv::Mat leftFromRight;                       // 4x4 CV_64F, meters (pose of the right camera in the left frame)
};

/// Metric relative pose of one device pair and the quality metrics of the estimate.
struct TracksPoseResult {
    bool ok = false;
    cv::Mat refFromOther;  // 4x4 CV_64F, meters: pose of the other device's left camera in the reference left frame
    int numTracks = 0;
    int numInliers = 0;
    double reprojRmsePx = 0.0;
    double scaleRmseM = 0.0;
    std::string error;
};

cv::Mat toMat33(const std::vector<std::vector<float>>& values) {
    cv::Mat matrix(3, 3, CV_64F);
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) matrix.at<double>(i, j) = static_cast<double>(values[i][j]);
    return matrix;
}

cv::Mat orthonormalizeRotation(const cv::Mat& rotation) {
    cv::Mat u, w, vt;
    cv::SVD::compute(rotation, w, u, vt);
    cv::Mat result = u * vt;
    if(cv::determinant(result) < 0) {
        u.col(2) *= -1;
        result = u * vt;
    }
    return result;
}

double meanFocal(const cv::Mat& k) {
    return 0.5 * (k.at<double>(0, 0) + k.at<double>(1, 1));
}

StereoGeometry deviceGeometry(const CalibrationHandler& calibration, CameraBoardSocket leftSocket, CameraBoardSocket rightSocket, int width, int height) {
    StereoGeometry geometry;
    geometry.kLeft = toMat33(calibration.getCameraIntrinsics(leftSocket, width, height));
    geometry.kRight = toMat33(calibration.getCameraIntrinsics(rightSocket, width, height));

    const auto distL = calibration.getDistortionCoefficients(leftSocket);
    const auto distR = calibration.getDistortionCoefficients(rightSocket);
    geometry.distLeft = cv::Mat(1, std::min<int>(8, static_cast<int>(distL.size())), CV_64F);
    geometry.distRight = cv::Mat(1, std::min<int>(8, static_cast<int>(distR.size())), CV_64F);
    for(int i = 0; i < geometry.distLeft.cols; ++i) geometry.distLeft.at<double>(0, i) = static_cast<double>(distL[i]);
    for(int i = 0; i < geometry.distRight.cols; ++i) geometry.distRight.at<double>(0, i) = static_cast<double>(distR[i]);

    const auto rotation = calibration.getCameraRotationMatrix(leftSocket, rightSocket);
    const auto translation = calibration.getCameraTranslationVector(leftSocket, rightSocket, false, LengthUnit::METER);
    cv::Mat rightFromLeft = cv::Mat::eye(4, 4, CV_64F);
    orthonormalizeRotation(toMat33(rotation)).copyTo(rightFromLeft(cv::Rect(0, 0, 3, 3)));
    for(int i = 0; i < 3; ++i) rightFromLeft.at<double>(i, 3) = static_cast<double>(translation[i]);
    geometry.leftFromRight = rightFromLeft.inv();
    return geometry;
}

cv::Mat toGray(const cv::Mat& frame) {
    cv::Mat gray = frame;
    if(gray.channels() == 3)
        cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    else if(gray.channels() == 1 && gray.dims == 2)
        gray = gray;  // already single channel
    if(gray.depth() != CV_8U) gray.convertTo(gray, CV_8U);
    return gray;
}

std::vector<cv::Point2d> keypointsToPoints(const std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& indices) {
    std::vector<cv::Point2d> points;
    points.reserve(indices.size());
    for(const int index : indices) points.emplace_back(keypoints[index].pt.x, keypoints[index].pt.y);
    return points;
}

/// Ratio-tested, cross-checked mutual nearest neighbors between two descriptor sets (queryIdx -> trainIdx).
std::map<int, int> ratioCrossMatches(const cv::Mat& descriptorsA, const cv::Mat& descriptorsB, double ratio) {
    if(descriptorsA.empty() || descriptorsB.empty()) return {};
    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<std::vector<cv::DMatch>> forward, reverse;
    matcher.knnMatch(descriptorsA, descriptorsB, forward, 2);
    matcher.knnMatch(descriptorsB, descriptorsA, reverse, 2);

    const auto filterKnn = [ratio](const std::vector<std::vector<cv::DMatch>>& matches) {
        std::map<int, std::pair<int, float>> good;
        for(const auto& pair : matches) {
            if(pair.size() < 2) continue;
            if(static_cast<double>(pair[0].distance) < ratio * static_cast<double>(pair[1].distance)) {
                const auto existing = good.find(pair[0].queryIdx);
                if(existing == good.end() || pair[0].distance < existing->second.second) good[pair[0].queryIdx] = {pair[0].trainIdx, pair[0].distance};
            }
        }
        return good;
    };
    const auto forwardGood = filterKnn(forward);
    const auto reverseGood = filterKnn(reverse);
    std::map<int, int> crossChecked;
    for(const auto& [queryIndex, trained] : forwardGood) {
        const auto reverseMatch = reverseGood.find(trained.first);
        if(reverseMatch != reverseGood.end() && reverseMatch->second.first == queryIndex) crossChecked[queryIndex] = trained.first;
    }
    return crossChecked;
}

std::vector<cv::Point2d> normalizeUndistort(const std::vector<cv::Point2d>& pointsPx, const cv::Mat& intrinsics) {
    if(pointsPx.empty()) return {};
    std::vector<cv::Point2d> normalized;
    cv::undistortPoints(pointsPx, normalized, intrinsics, cv::noArray());
    return normalized;
}

/// Triangulate corresponding pixels of two cameras of known relative pose, returning 3D points in camera A's frame.
void triangulate(const std::vector<cv::Point2d>& pointsAPx,
                 const std::vector<cv::Point2d>& pointsBPx,
                 const cv::Mat& intrinsicsA,
                 const cv::Mat& intrinsicsB,
                 const cv::Mat& transformAFromB,
                 std::vector<cv::Point3d>& pointsA,
                 std::vector<cv::Point3d>& pointsB,
                 std::vector<char>& positiveDepth) {
    const size_t count = pointsAPx.size();
    pointsA.assign(count, cv::Point3d(std::nan(""), std::nan(""), std::nan("")));
    pointsB.assign(count, cv::Point3d(std::nan(""), std::nan(""), std::nan("")));
    positiveDepth.assign(count, 0);
    if(count == 0) return;

    const auto normalizedA = normalizeUndistort(pointsAPx, intrinsicsA);
    const auto normalizedB = normalizeUndistort(pointsBPx, intrinsicsB);
    const cv::Mat projectionA = cv::Mat::eye(3, 4, CV_64F);
    const cv::Mat transformBFromA = transformAFromB.inv();
    const cv::Mat projectionB = transformBFromA(cv::Rect(0, 0, 4, 3)).clone();

    cv::Mat matA(2, static_cast<int>(count), CV_64F), matB(2, static_cast<int>(count), CV_64F);
    for(size_t i = 0; i < count; ++i) {
        matA.at<double>(0, i) = normalizedA[i].x;
        matA.at<double>(1, i) = normalizedA[i].y;
        matB.at<double>(0, i) = normalizedB[i].x;
        matB.at<double>(1, i) = normalizedB[i].y;
    }
    cv::Mat points4d;
    cv::triangulatePoints(projectionA, projectionB, matA, matB, points4d);

    const cv::Mat rotationBFromA = transformBFromA(cv::Rect(0, 0, 3, 3));
    for(size_t i = 0; i < count; ++i) {
        const double w = points4d.at<double>(3, static_cast<int>(i));
        if(std::abs(w) <= 1e-8) continue;
        const cv::Point3d pa(
            points4d.at<double>(0, static_cast<int>(i)) / w, points4d.at<double>(1, static_cast<int>(i)) / w, points4d.at<double>(2, static_cast<int>(i)) / w);
        cv::Mat pbMat = rotationBFromA * (cv::Mat_<double>(3, 1) << pa.x, pa.y, pa.z);
        const cv::Point3d pb(pbMat.at<double>(0) + transformBFromA.at<double>(0, 3),
                             pbMat.at<double>(1) + transformBFromA.at<double>(1, 3),
                             pbMat.at<double>(2) + transformBFromA.at<double>(2, 3));
        pointsA[i] = pa;
        pointsB[i] = pb;
        positiveDepth[i] = (pa.z > 0.0 && pb.z > 0.0) ? 1 : 0;
    }
}

/// Pinhole projection of already-undistorted camera points (no distortion), NaN where the point is behind the camera.
cv::Point2d projectPoint(const cv::Point3d& point, const cv::Mat& intrinsics) {
    if(!(point.z > 1e-8) || !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) return {std::nan(""), std::nan("")};
    return {intrinsics.at<double>(0, 0) * point.x / point.z + intrinsics.at<double>(0, 2),
            intrinsics.at<double>(1, 1) * point.y / point.z + intrinsics.at<double>(1, 2)};
}

/// Keep only stereo matches whose triangulated point reprojects within tolerance in both views.
std::map<int, int> filterMatchesByReprojection(const std::vector<cv::KeyPoint>& keypointsA,
                                               const std::vector<cv::KeyPoint>& keypointsB,
                                               const std::map<int, int>& matches,
                                               const cv::Mat& intrinsicsA,
                                               const cv::Mat& intrinsicsB,
                                               const cv::Mat& transformAFromB,
                                               double maxReprojectionErrorPx) {
    if(matches.empty()) return {};
    std::vector<int> queryIndices, trainIndices;
    for(const auto& [query, train] : matches) {
        queryIndices.push_back(query);
        trainIndices.push_back(train);
    }
    const auto pointsAPx = keypointsToPoints(keypointsA, queryIndices);
    const auto pointsBPx = keypointsToPoints(keypointsB, trainIndices);

    std::vector<cv::Point3d> pointsA, pointsB;
    std::vector<char> positiveDepth;
    triangulate(pointsAPx, pointsBPx, intrinsicsA, intrinsicsB, transformAFromB, pointsA, pointsB, positiveDepth);

    std::map<int, int> filtered;
    for(size_t i = 0; i < queryIndices.size(); ++i) {
        if(!positiveDepth[i]) continue;
        const auto projectedA = projectPoint(pointsA[i], intrinsicsA);
        const auto projectedB = projectPoint(pointsB[i], intrinsicsB);
        if(!std::isfinite(projectedA.x) || !std::isfinite(projectedB.x)) continue;
        const double errorA = cv::norm(projectedA - pointsAPx[i]);
        const double errorB = cv::norm(projectedB - pointsBPx[i]);
        if(errorA <= maxReprojectionErrorPx && errorB <= maxReprojectionErrorPx) filtered[queryIndices[i]] = trainIndices[i];
    }
    return filtered;
}

/// Keep only cross-device matches consistent with a fundamental matrix estimated by RANSAC.
std::map<int, int> filterMatchesByFundamental(const std::vector<cv::KeyPoint>& keypointsA,
                                              const std::vector<cv::KeyPoint>& keypointsB,
                                              const std::map<int, int>& matches,
                                              double ransacReprojErrorPx) {
    if(matches.size() < 8) return matches;
    std::vector<int> queryIndices, trainIndices;
    std::vector<cv::Point2d> pointsA, pointsB;
    for(const auto& [query, train] : matches) {
        queryIndices.push_back(query);
        trainIndices.push_back(train);
        pointsA.emplace_back(keypointsA[query].pt.x, keypointsA[query].pt.y);
        pointsB.emplace_back(keypointsB[train].pt.x, keypointsB[train].pt.y);
    }
    cv::Mat mask;
    cv::Mat fundamental;
    for(const int method : {static_cast<int>(cv::USAC_MAGSAC), static_cast<int>(cv::FM_RANSAC)}) {
        try {
            fundamental = cv::findFundamentalMat(pointsA, pointsB, method, ransacReprojErrorPx, 0.999, mask);
        } catch(const cv::Exception&) {
            fundamental = cv::Mat();
            mask = cv::Mat();
        }
        if(!fundamental.empty() && !mask.empty()) break;
    }
    if(fundamental.empty() || mask.empty()) return {};

    std::map<int, int> filtered;
    for(size_t i = 0; i < queryIndices.size(); ++i)
        if(mask.at<uchar>(static_cast<int>(i))) filtered[queryIndices[i]] = trainIndices[i];
    return filtered;
}

double medianOf(std::vector<double> values) {
    if(values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const size_t mid = values.size() / 2;
    return values.size() % 2 == 0 ? 0.5 * (values[mid - 1] + values[mid]) : values[mid];
}

/// Robust scalar that scales the unit translation so the two metric point clouds align along it (median + Huber).
bool robustScalarScaleFit(const std::vector<cv::Point3d>& pointsCam1,
                          const std::vector<cv::Point3d>& pointsCam3,
                          const cv::Matx33d& rotation1From3,
                          const cv::Vec3d& translationUnit,
                          double& scaleOut,
                          double& rmseOut) {
    cv::Vec3d unit = translationUnit;
    const double unitNorm = cv::norm(unit);
    if(unitNorm < 1e-12) return false;
    unit /= unitNorm;

    std::vector<double> perTrackScales;
    std::vector<cv::Vec3d> residualVectors;
    perTrackScales.reserve(pointsCam1.size());
    residualVectors.reserve(pointsCam1.size());
    for(size_t i = 0; i < pointsCam1.size(); ++i) {
        const cv::Vec3d p1(pointsCam1[i].x, pointsCam1[i].y, pointsCam1[i].z);
        const cv::Vec3d p3(pointsCam3[i].x, pointsCam3[i].y, pointsCam3[i].z);
        const cv::Vec3d residual = p1 - rotation1From3 * p3;
        if(!std::isfinite(residual[0]) || !std::isfinite(residual[1]) || !std::isfinite(residual[2])) continue;
        residualVectors.push_back(residual);
        perTrackScales.push_back(residual.dot(unit));
    }
    if(perTrackScales.size() < 4) return false;

    double scale = medianOf(perTrackScales);
    std::vector<double> deviations;
    deviations.reserve(perTrackScales.size());
    for(const double value : perTrackScales) deviations.push_back(std::abs(value - scale));
    const double mad = medianOf(deviations);
    const double sigma = std::max(1.4826 * mad, 1e-4);

    std::vector<size_t> inliers;
    for(size_t i = 0; i < perTrackScales.size(); ++i)
        if(std::abs(perTrackScales[i] - scale) <= 3.0 * sigma) inliers.push_back(i);
    if(inliers.size() < 4) {
        inliers.clear();
        for(size_t i = 0; i < perTrackScales.size(); ++i) inliers.push_back(i);
    }

    const double huberScale = std::max(1.345 * sigma, 1e-4);
    for(int iteration = 0; iteration < 5; ++iteration) {
        double weightSum = 0.0, weightedValue = 0.0;
        for(const size_t index : inliers) {
            const double residual = perTrackScales[index] - scale;
            const double weight = std::abs(residual) > huberScale ? huberScale / std::abs(residual) : 1.0;
            weightSum += weight;
            weightedValue += weight * perTrackScales[index];
        }
        if(weightSum > 0.0) scale = weightedValue / weightSum;
    }

    double squaredSum = 0.0;
    for(const size_t index : inliers) {
        const cv::Vec3d pointResidual = residualVectors[index] - scale * unit;
        squaredSum += pointResidual.dot(pointResidual);
    }
    scaleOut = scale;
    rmseOut = inliers.empty() ? 0.0 : std::sqrt(squaredSum / static_cast<double>(inliers.size()));
    return true;
}

cv::Vec3d normalize(const cv::Vec3d& vector) {
    const double norm = cv::norm(vector);
    return norm < 1e-12 ? vector : vector / norm;
}

/// 4N reprojection residuals of the two-view pose over triangulated points; behind-camera points get a large penalty.
std::vector<double> poseReprojectionResiduals(const cv::Matx33d& rotation3From1,
                                              const cv::Vec3d& translation3From1,
                                              const std::vector<cv::Point2d>& points1Px,
                                              const std::vector<cv::Point2d>& points3Px,
                                              const std::vector<cv::Point2d>& points1Norm,
                                              const std::vector<cv::Point2d>& points3Norm,
                                              const cv::Mat& intrinsics1,
                                              const cv::Mat& intrinsics3) {
    const size_t count = points1Px.size();
    std::vector<double> residuals(4 * count, 1e3);

    cv::Mat transform3From1 = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat(rotation3From1).copyTo(transform3From1(cv::Rect(0, 0, 3, 3)));
    for(int i = 0; i < 3; ++i) transform3From1.at<double>(i, 3) = translation3From1[i];

    std::vector<cv::Point3d> pointsCam1, pointsCam3;
    std::vector<char> positiveDepth;
    triangulate(points1Norm, points3Norm, cv::Mat::eye(3, 3, CV_64F), cv::Mat::eye(3, 3, CV_64F), transform3From1.inv(), pointsCam1, pointsCam3, positiveDepth);

    for(size_t i = 0; i < count; ++i) {
        if(!positiveDepth[i]) continue;
        const auto projected1 = projectPoint(pointsCam1[i], intrinsics1);
        const auto projected3 = projectPoint(pointsCam3[i], intrinsics3);
        if(!std::isfinite(projected1.x) || !std::isfinite(projected3.x)) continue;
        residuals[4 * i + 0] = projected1.x - points1Px[i].x;
        residuals[4 * i + 1] = projected1.y - points1Px[i].y;
        residuals[4 * i + 2] = projected3.x - points3Px[i].x;
        residuals[4 * i + 3] = projected3.y - points3Px[i].y;
    }
    return residuals;
}

double rootMeanSquare(const std::vector<double>& residuals) {
    if(residuals.empty()) return 0.0;
    double sum = 0.0;
    for(const double value : residuals) sum += value * value;
    return std::sqrt(sum / static_cast<double>(residuals.size()));
}

/// Levenberg-Marquardt refinement of the two-view pose (rvec + unit translation) under a Huber loss.
void refinePose(cv::Matx33d& rotation3From1,
                cv::Vec3d& translation3From1,
                const std::vector<cv::Point2d>& points1Px,
                const std::vector<cv::Point2d>& points3Px,
                const std::vector<cv::Point2d>& points1Norm,
                const std::vector<cv::Point2d>& points3Norm,
                const cv::Mat& intrinsics1,
                const cv::Mat& intrinsics3) {
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(rotation3From1), rvec);
    cv::Vec6d params;
    for(int i = 0; i < 3; ++i) params[i] = rvec.at<double>(i);
    const cv::Vec3d unit = normalize(translation3From1);
    for(int i = 0; i < 3; ++i) params[3 + i] = unit[i];

    const auto residualsOf = [&](const cv::Vec6d& p) {
        cv::Mat r;
        const cv::Mat rotationVector = (cv::Mat_<double>(3, 1) << p[0], p[1], p[2]);
        cv::Rodrigues(rotationVector, r);
        cv::Matx33d rotation;
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j) rotation(i, j) = r.at<double>(i, j);
        const cv::Vec3d translation = normalize(cv::Vec3d(p[3], p[4], p[5]));
        return poseReprojectionResiduals(rotation, translation, points1Px, points3Px, points1Norm, points3Norm, intrinsics1, intrinsics3);
    };

    const double fScale = 1.0;
    const auto huberCost = [fScale](const std::vector<double>& residuals) {
        double cost = 0.0;
        for(const double value : residuals) {
            const double absValue = std::abs(value);
            cost += absValue <= fScale ? 0.5 * value * value : fScale * (absValue - 0.5 * fScale);
        }
        return cost;
    };

    std::vector<double> residuals = residualsOf(params);
    double cost = huberCost(residuals);
    double lambda = 1e-3;
    const double epsilon = 1e-6;

    for(int iteration = 0; iteration < 30; ++iteration) {
        const size_t residualCount = residuals.size();
        cv::Mat jacobian(static_cast<int>(residualCount), 6, CV_64F);
        for(int column = 0; column < 6; ++column) {
            cv::Vec6d plus = params, minus = params;
            plus[column] += epsilon;
            minus[column] -= epsilon;
            const auto residualsPlus = residualsOf(plus);
            const auto residualsMinus = residualsOf(minus);
            for(size_t row = 0; row < residualCount; ++row)
                jacobian.at<double>(static_cast<int>(row), column) = (residualsPlus[row] - residualsMinus[row]) / (2.0 * epsilon);
        }

        cv::Matx66d hessian = cv::Matx66d::zeros();
        cv::Vec6d gradient = cv::Vec6d::all(0.0);
        for(size_t row = 0; row < residualCount; ++row) {
            const double absResidual = std::abs(residuals[row]);
            const double weight = absResidual <= fScale ? 1.0 : fScale / absResidual;  // squared IRLS weight
            for(int i = 0; i < 6; ++i) {
                const double ji = jacobian.at<double>(static_cast<int>(row), i);
                gradient[i] += weight * ji * residuals[row];
                for(int j = 0; j < 6; ++j) hessian(i, j) += weight * ji * jacobian.at<double>(static_cast<int>(row), j);
            }
        }

        bool stepAccepted = false;
        for(int attempt = 0; attempt < 8 && !stepAccepted; ++attempt) {
            cv::Matx66d damped = hessian;
            for(int i = 0; i < 6; ++i) damped(i, i) += lambda * hessian(i, i);
            cv::Mat deltaMat;
            if(!cv::solve(cv::Mat(damped), cv::Mat(-gradient), deltaMat, cv::DECOMP_SVD)) {
                lambda *= 10.0;
                continue;
            }
            cv::Vec6d delta;
            for(int i = 0; i < 6; ++i) delta[i] = deltaMat.at<double>(i);
            const cv::Vec6d candidate = params + delta;
            const auto candidateResiduals = residualsOf(candidate);
            const double candidateCost = huberCost(candidateResiduals);
            if(candidateCost < cost) {
                params = candidate;
                residuals = candidateResiduals;
                cost = candidateCost;
                lambda = std::max(lambda * 0.5, 1e-9);
                stepAccepted = true;
            } else {
                lambda *= 4.0;
            }
        }
        if(!stepAccepted) break;
    }

    cv::Mat refined;
    const cv::Mat refinedVector = (cv::Mat_<double>(3, 1) << params[0], params[1], params[2]);
    cv::Rodrigues(refinedVector, refined);
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) rotation3From1(i, j) = refined.at<double>(i, j);
    translation3From1 = normalize(cv::Vec3d(params[3], params[4], params[5]));
}

/// Accumulated four-view tracks and reference-camera correspondences across all captured frames.
struct TrackBundle {
    std::vector<cv::Point2d> points1Px, points3Px;            // reference<->other (CAM_B) matches
    std::vector<cv::Point2d> track1, track2, track3, track4;  // four-view track pixels per camera
};

/// Detect SIFT features, build stereo + cross-device matches and accumulate four-view tracks over all frames.
TrackBundle detectTracks(const std::vector<cv::Mat>& refLeft,
                         const std::vector<cv::Mat>& refRight,
                         const std::vector<cv::Mat>& otherLeft,
                         const std::vector<cv::Mat>& otherRight,
                         const StereoGeometry& refGeometry,
                         const StereoGeometry& otherGeometry,
                         const FeatureConfig& config) {
    TrackBundle bundle;
    auto sift = cv::SIFT::create();
    const size_t frames = std::min({refLeft.size(), refRight.size(), otherLeft.size(), otherRight.size()});

    for(size_t frame = 0; frame < frames; ++frame) {
        cv::Mat image1, image2, image3, image4;
        cv::undistort(toGray(refLeft[frame]), image1, refGeometry.kLeft, refGeometry.distLeft);
        cv::undistort(toGray(refRight[frame]), image2, refGeometry.kRight, refGeometry.distRight);
        cv::undistort(toGray(otherLeft[frame]), image3, otherGeometry.kLeft, otherGeometry.distLeft);
        cv::undistort(toGray(otherRight[frame]), image4, otherGeometry.kRight, otherGeometry.distRight);

        std::vector<cv::KeyPoint> keypoints1, keypoints2, keypoints3, keypoints4;
        cv::Mat descriptors1, descriptors2, descriptors3, descriptors4;
        sift->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
        sift->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
        sift->detectAndCompute(image3, cv::noArray(), keypoints3, descriptors3);
        sift->detectAndCompute(image4, cv::noArray(), keypoints4, descriptors4);

        const auto matches12 = filterMatchesByReprojection(keypoints1,
                                                           keypoints2,
                                                           ratioCrossMatches(descriptors1, descriptors2, config.ratioTest),
                                                           refGeometry.kLeft,
                                                           refGeometry.kRight,
                                                           refGeometry.leftFromRight,
                                                           config.stereoMaxReprojErrorPx);
        const auto matches13 = filterMatchesByFundamental(
            keypoints1, keypoints3, ratioCrossMatches(descriptors1, descriptors3, config.ratioTest), config.crossRansacReprojErrorPx);
        const auto matches24 = filterMatchesByFundamental(
            keypoints2, keypoints4, ratioCrossMatches(descriptors2, descriptors4, config.ratioTest), config.crossRansacReprojErrorPx);
        const auto matches34 = filterMatchesByReprojection(keypoints3,
                                                           keypoints4,
                                                           ratioCrossMatches(descriptors3, descriptors4, config.ratioTest),
                                                           otherGeometry.kLeft,
                                                           otherGeometry.kRight,
                                                           otherGeometry.leftFromRight,
                                                           config.stereoMaxReprojErrorPx);

        for(const auto& [index1, index3] : matches13) {
            bundle.points1Px.emplace_back(keypoints1[index1].pt.x, keypoints1[index1].pt.y);
            bundle.points3Px.emplace_back(keypoints3[index3].pt.x, keypoints3[index3].pt.y);
        }

        std::set<std::tuple<int, int, int, int>> usedTracks;
        for(const auto& [index1, index2] : matches12) {
            const auto matched13 = matches13.find(index1);
            if(matched13 == matches13.end()) continue;
            const int index3 = matched13->second;
            const auto matched24 = matches24.find(index2);
            const auto matched34 = matches34.find(index3);
            if(matched24 == matches24.end() || matched34 == matches34.end() || matched24->second != matched34->second) continue;
            const int index4 = matched24->second;
            const auto track = std::make_tuple(index1, index2, index3, index4);
            if(!usedTracks.insert(track).second) continue;
            bundle.track1.emplace_back(keypoints1[index1].pt.x, keypoints1[index1].pt.y);
            bundle.track2.emplace_back(keypoints2[index2].pt.x, keypoints2[index2].pt.y);
            bundle.track3.emplace_back(keypoints3[index3].pt.x, keypoints3[index3].pt.y);
            bundle.track4.emplace_back(keypoints4[index4].pt.x, keypoints4[index4].pt.y);
        }
    }
    return bundle;
}

template <typename T>
std::vector<T> selectByMask(const std::vector<T>& values, const cv::Mat& mask) {
    std::vector<T> selected;
    for(size_t i = 0; i < values.size(); ++i)
        if(mask.at<uchar>(static_cast<int>(i))) selected.push_back(values[i]);
    return selected;
}

/// Estimate the metric pose of the other device's reference camera in the reference device's frame from four-view
/// tracks: essential matrix (rotation + translation direction) refined by reprojection, then a robust metric scale.
TracksPoseResult computeRelativePose(const std::vector<cv::Mat>& refLeft,
                                     const std::vector<cv::Mat>& refRight,
                                     const std::vector<cv::Mat>& otherLeft,
                                     const std::vector<cv::Mat>& otherRight,
                                     const StereoGeometry& refGeometry,
                                     const StereoGeometry& otherGeometry,
                                     const FeatureConfig& config) {
    TracksPoseResult result;
    const TrackBundle bundle = detectTracks(refLeft, refRight, otherLeft, otherRight, refGeometry, otherGeometry, config);
    if(bundle.points1Px.size() < 8) {
        result.error = fmt::format("only {} reference<->other correspondences, need at least 8", bundle.points1Px.size());
        return result;
    }
    if(bundle.track1.size() < 4) {
        result.error = fmt::format("only {} four-view tracks, need at least 4", bundle.track1.size());
        return result;
    }

    std::vector<cv::Point2d> points1Px = bundle.points1Px;
    std::vector<cv::Point2d> points3Px = bundle.points3Px;
    auto points1Norm = normalizeUndistort(points1Px, refGeometry.kLeft);
    auto points3Norm = normalizeUndistort(points3Px, otherGeometry.kLeft);

    const double threshold = 1.0 / (0.5 * (meanFocal(refGeometry.kLeft) + meanFocal(otherGeometry.kLeft)));
    cv::Mat essentialMask;
    const cv::Mat essential = cv::findEssentialMat(points1Norm, points3Norm, cv::Mat::eye(3, 3, CV_64F), cv::RANSAC, 0.999, threshold, essentialMask);
    if(essential.empty() || essentialMask.empty()) {
        result.error = "essential matrix estimation failed";
        return result;
    }
    points1Px = selectByMask(points1Px, essentialMask);
    points3Px = selectByMask(points3Px, essentialMask);
    points1Norm = selectByMask(points1Norm, essentialMask);
    points3Norm = selectByMask(points3Norm, essentialMask);
    if(points1Px.size() < 8) {
        result.error = fmt::format("only {} essential-matrix inliers, need at least 8", points1Px.size());
        return result;
    }

    cv::Mat rotationMat, translationMat, poseMask = cv::Mat::ones(static_cast<int>(points1Norm.size()), 1, CV_8U) * 255;
    cv::recoverPose(essential, points1Norm, points3Norm, cv::Mat::eye(3, 3, CV_64F), rotationMat, translationMat, poseMask);
    points1Px = selectByMask(points1Px, poseMask);
    points3Px = selectByMask(points3Px, poseMask);
    points1Norm = selectByMask(points1Norm, poseMask);
    points3Norm = selectByMask(points3Norm, poseMask);
    if(points1Px.size() < 8) {
        result.error = fmt::format("only {} recoverPose inliers, need at least 8", points1Px.size());
        return result;
    }

    cv::Matx33d rotation3From1;
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) rotation3From1(i, j) = rotationMat.at<double>(i, j);
    cv::Vec3d translation3From1 = normalize(cv::Vec3d(translationMat.at<double>(0), translationMat.at<double>(1), translationMat.at<double>(2)));

    refinePose(rotation3From1, translation3From1, points1Px, points3Px, points1Norm, points3Norm, refGeometry.kLeft, otherGeometry.kLeft);
    result.reprojRmsePx = rootMeanSquare(
        poseReprojectionResiduals(rotation3From1, translation3From1, points1Px, points3Px, points1Norm, points3Norm, refGeometry.kLeft, otherGeometry.kLeft));

    // T_1_3 (reference-from-other) is the inverse of the recovered other-from-reference pose; only its direction is
    // known, so its magnitude is fixed next from the metric tracks.
    cv::Mat transform3From1 = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat(rotation3From1).copyTo(transform3From1(cv::Rect(0, 0, 3, 3)));
    for(int i = 0; i < 3; ++i) transform3From1.at<double>(i, 3) = translation3From1[i];
    cv::Mat transform1From3 = transform3From1.inv();
    cv::Matx33d rotation1From3;
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) rotation1From3(i, j) = transform1From3.at<double>(i, j);
    const cv::Vec3d translationUnit =
        normalize(cv::Vec3d(transform1From3.at<double>(0, 3), transform1From3.at<double>(1, 3), transform1From3.at<double>(2, 3)));

    // Metric scale: triangulate the same tracks in each device's own stereo pair and fit the scalar aligning them.
    std::vector<cv::Point3d> pointsCam1, pointsCam3, pointsCam1Other, pointsCam3Other;
    std::vector<char> valid12, valid34;
    triangulate(bundle.track1, bundle.track2, refGeometry.kLeft, refGeometry.kRight, refGeometry.leftFromRight, pointsCam1, pointsCam1Other, valid12);
    triangulate(bundle.track3, bundle.track4, otherGeometry.kLeft, otherGeometry.kRight, otherGeometry.leftFromRight, pointsCam3, pointsCam3Other, valid34);

    std::vector<cv::Point3d> alignedCam1, alignedCam3;
    for(size_t i = 0; i < pointsCam1.size(); ++i)
        if(valid12[i] && valid34[i]) {
            alignedCam1.push_back(pointsCam1[i]);
            alignedCam3.push_back(pointsCam3[i]);
        }
    if(alignedCam1.size() < 4) {
        result.error = fmt::format("only {} valid triangulated tracks, need at least 4", alignedCam1.size());
        return result;
    }

    double scale = 0.0, scaleRmse = 0.0;
    if(!robustScalarScaleFit(alignedCam1, alignedCam3, rotation1From3, translationUnit, scale, scaleRmse)) {
        result.error = "robust scale fit failed";
        return result;
    }

    const cv::Vec3d translation = scale * translationUnit;
    for(int i = 0; i < 3; ++i) transform1From3.at<double>(i, 3) = translation[i];

    result.ok = true;
    result.refFromOther = transform1From3;
    result.numTracks = static_cast<int>(alignedCam1.size());
    result.numInliers = static_cast<int>(alignedCam1.size());
    result.scaleRmseM = scaleRmse;
    return result;
}

}  // namespace

void MultiDeviceCalibration::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    auto& cameras = pimpl->cameras;

    DAI_CHECK_V(cameras.size() >= 2, "MultiDeviceCalibration needs at least two cameras, got {}. Register them with build() or addCamera().", cameras.size());
    std::set<std::string> deviceIds;
    for(const auto& camera : cameras) {
        deviceIds.insert(camera.frame.deviceId);
    }
    DAI_CHECK_V(deviceIds.size() >= 2,
                "MultiDeviceCalibration estimates transformations between different devices, but all its cameras are on device {}. Use the "
                "DynamicCalibration node for a single device.",
                *deviceIds.begin());

    // Per-device calibration is authoritative, so read it from the live devices
    for(const auto& device : getParentPipeline().getAllAssignedDevices()) {
        if(deviceIds.count(device->getDeviceId()) == 0) continue;
        pimpl->calibrations.emplace(device->getDeviceId(), device->getCalibration());
    }
    for(const auto& deviceId : deviceIds) {
        DAI_CHECK_V(pimpl->calibrations.count(deviceId) == 1,
                    "MultiDeviceCalibration was given cameras of device {}, but no such device is assigned to the pipeline",
                    deviceId);
    }

    // The rig is expressed w.r.t. the first registered camera; the initial pose of every other camera is composed from
    // the user supplied inter-device guesses and the intra-device calibration of its own device.
    const auto baseFrame = cameras.front().frame;
    pimpl->baseFrame = baseFrame;
    for(const auto& camera : cameras) {
        pimpl->deviceReference.emplace(camera.frame.deviceId, camera.frame);
    }
    // The feature-tracks method estimates the geometry from scratch, so it needs neither an initial guess nor the
    // dynamic calibration library seeding below.
    if(pimpl->method == Method::DYNAMIC_CALIBRATION) {
        const MultiDeviceCalibrationHandler guesses(MultiDeviceCalibrationData{1, 0, pimpl->initialGuesses, {}});
        for(auto& camera : cameras) {
            const auto& reference = pimpl->deviceReference.at(camera.frame.deviceId);
            Transform referenceToRigBase = identityTransform();
            if(reference != baseFrame) {
                DAI_CHECK_V(guesses.canTransform(reference, baseFrame),
                            "MultiDeviceCalibration has no initial guess connecting {} to {}. Supply a rough one with setInitialGuess().",
                            toString(reference),
                            toString(baseFrame));
                referenceToRigBase = matrix::toVecMatrix4x4(guesses.getTransform(reference, baseFrame, LengthUnit::METER));
            }
            // T_camera<-rigBase == T_camera<-deviceReference * T_deviceReference<-rigBase
            camera.toRigBase = matrix::matMul(pimpl->referenceToCamera(camera.frame), inverted(referenceToRigBase));
        }
    }

    // Wait for the first synchronized set to learn the resolutions and intrinsics actually produced
    logger->info("Waiting for the first synchronized image set of {} cameras", cameras.size());
    std::shared_ptr<MessageGroup> group;
    {
        auto blockEvent = this->inputBlockEvent();
        group = syncInput.get<MessageGroup>();
    }
    if(group == nullptr) return;
    for(auto& camera : cameras) {
        const auto frame = group->get<ImgFrame>(camera.inputName);
        DAI_CHECK_V(frame != nullptr, "MultiDeviceCalibration is missing an image of camera {}", toString(camera.frame));
        camera.resolution = {frame->getWidth(), frame->getHeight()};
        camera.transformation = frame->getTransformation();
    }

    // The feature-tracks method just accumulates the raw synchronized image sets and solves them itself.
    if(pimpl->method == Method::FEATURE_TRACKS) {
        pimpl->trackFrames.assign(cameras.size(), {});
        size_t collected = 0;
        while(mainLoop()) {
            if(group == nullptr) {
                auto blockEvent = this->inputBlockEvent();
                group = syncInput.get<MessageGroup>();
                if(group == nullptr) continue;
            }
            bool complete = true;
            for(const auto& camera : cameras) {
                if(group->get<ImgFrame>(camera.inputName) == nullptr) {
                    logger->trace("Missing image of camera {} in the synchronized group", toString(camera.frame));
                    complete = false;
                    break;
                }
            }
            if(complete) {
                for(size_t i = 0; i < cameras.size(); ++i) pimpl->trackFrames[i].push_back(group->get<ImgFrame>(cameras[i].inputName));
                ++collected;
            }
            group = nullptr;
            if(collected < pimpl->sampleCount) continue;

            collected = 0;
            estimateFromTracks();
            for(auto& frames : pimpl->trackFrames) frames.clear();
            if(!pimpl->continuous) {
                logger->info("Rig calibration emitted, MultiDeviceCalibration is done. Use setContinuous(true) to keep estimating.");
                return;
            }
        }
        return;
    }

    pimpl->dclDevice = pimpl->dcl.addDevice();
    for(auto& camera : cameras) {
        const auto calibration = DclUtils::createDclCalibration(camera.transformation.getIntrinsicMatrix(),
                                                                camera.transformation.getDistortionCoefficients(),
                                                                matrix::extractRotationMatrix(camera.toRigBase),
                                                                matrix::extractTranslationVector(camera.toRigBase),
                                                                camera.transformation.getDistortionModel());
        // All physical devices are registered as a single DCL device on purpose - DCL rejects sensor sets spanning
        // several of its devices, and the socket-less overload avoids the socket collisions between our devices.
        camera.sensor = pimpl->dcl.addSensor(pimpl->dclDevice, calibration, dcl::resolution_t{camera.resolution.first, camera.resolution.second});
    }

    size_t loaded = 0;
    while(mainLoop()) {
        if(group == nullptr) {
            auto blockEvent = this->inputBlockEvent();
            group = syncInput.get<MessageGroup>();
            if(group == nullptr) continue;
        }

        dcl::DeviceImageList images;
        images.reserve(cameras.size());
        dcl::timestamp_t timestamp = 0;
        bool complete = true;
        for(const auto& camera : cameras) {
            const auto frame = group->get<ImgFrame>(camera.inputName);
            if(frame == nullptr) {
                logger->trace("Missing image of camera {} in the synchronized group", toString(camera.frame));
                complete = false;
                break;
            }
            if(timestamp == 0) {
                timestamp = static_cast<dcl::timestamp_t>(frame->getTimestamp().time_since_epoch().count());
            }
            auto cvFrame = frame->getCvFrame();
            images.emplace_back(camera.sensor, DclUtils::cvMatToImageData(cvFrame));
        }
        group = nullptr;
        if(!complete) continue;

        auto loadResult = pimpl->dcl.loadImages(images, timestamp);
        if(!loadResult.passed()) {
            logger->trace("Failed to load the synchronized image set: {}", loadResult.errorMessage());
            continue;
        }
        if(++loaded < pimpl->sampleCount) continue;

        loaded = 0;
        estimate();
        if(!pimpl->continuous) {
            logger->info("Rig calibration emitted, MultiDeviceCalibration is done. Use setContinuous(true) to keep estimating.");
            return;
        }
    }
}

void MultiDeviceCalibration::estimate() {
    auto& logger = ThreadedNode::pimpl->logger;
    const auto& cameras = pimpl->cameras;
    const size_t numCameras = cameras.size();

    // Cameras that never observed a common scene cannot be related, so decompose the set into the components DCL can
    // actually solve - it expects a single connected one.
    std::vector<size_t> component(numCameras);
    std::iota(component.begin(), component.end(), 0);
    const std::function<size_t(size_t)> root = [&component, &root](size_t index) {
        return component[index] == index ? index : component[index] = root(component[index]);
    };
    for(size_t i = 0; i < numCameras; ++i) {
        for(size_t j = i + 1; j < numCameras; ++j) {
            auto confidence = pimpl->dcl.computeDataConfidence(cameras[i].sensor, cameras[j].sensor);
            if(!confidence.passed() || confidence.value <= 0.0) continue;
            component[root(i)] = root(j);
        }
    }
    std::map<size_t, std::vector<size_t>> components;
    for(size_t i = 0; i < numCameras; ++i) {
        components[root(i)].push_back(i);
    }

    MultiDeviceCalibrationData rig;
    rig.timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::vector<std::string> notes;
    double dataConfidence = 0.0;

    // A single solve can land in a poor local minimum or fail a coverage gate, so instead of trusting one attempt the
    // node sweeps several strategies and keeps, per device, the edge that scores best on the dynamic calibration
    // library's own metrics. The attempted performance modes, whether the initial camera centers are kept and the yaw
    // perturbations of the initial guess make up the search; an explicit setPerformanceMode() narrows it to one solve.
    std::vector<DynamicCalibrationControl::PerformanceMode> modes;
    if(pimpl->autoStrategy) {
        modes = {DynamicCalibrationControl::PerformanceMode::DEFAULT,
                 DynamicCalibrationControl::PerformanceMode::RELAXED_COVERAGE,
                 DynamicCalibrationControl::PerformanceMode::OPTIMIZE_PERFORMANCE,
                 DynamicCalibrationControl::PerformanceMode::STATIC_SCENERY,
                 DynamicCalibrationControl::PerformanceMode::SKIP_CHECKS};
    } else {
        modes = {pimpl->performanceMode};
    }
    // keepCameraCenters=true is incompatible with the known baseline edges the scale relies on, so the centers are
    // always solved; the strategies vary instead by performance mode, joint vs. pairwise and the guess perturbation.
    const std::vector<bool> keepCentersOptions = {false};
    const std::vector<float> yawOffsets = pimpl->autoStrategy ? pimpl->guessYawOffsets : std::vector<float>{0.0f};

    // Best-scoring result kept for one inter-device edge across all attempted strategies.
    struct BestEdge {
        bool found = false;
        double confidence = -1.0;
        double sampson = std::numeric_limits<double>::max();
        Transform transform;  // T_baseReference<-deviceReference
        std::string approach;
    };

    for(const auto& [componentRoot, indices] : components) {
        (void)componentRoot;
        std::set<std::string> componentDevices;
        for(const auto index : indices) {
            componentDevices.insert(cameras[index].frame.deviceId);
        }
        if(componentDevices.size() < 2) {
            notes.push_back(fmt::format("cameras of device {} did not observe a scene shared with another device", *componentDevices.begin()));
            continue;
        }
        const auto baseDeviceId = pimpl->baseFrame.deviceId;
        if(componentDevices.count(baseDeviceId) == 0) {
            notes.push_back(fmt::format("devices [{}] did not share a scene with the reference device {}", fmt::join(componentDevices, ", "), baseDeviceId));
            continue;
        }

        {
            std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> componentSensors;
            for(const auto index : indices) componentSensors.push_back(cameras[index].sensor);
            auto confidence = pimpl->dcl.computeDataConfidence(componentSensors);
            if(confidence.passed()) dataConfidence = std::max(dataConfidence, confidence.value);
        }

        std::map<std::string, std::vector<size_t>> byDevice;
        for(const auto index : indices) byDevice[cameras[index].frame.deviceId].push_back(index);

        const auto& baseReference = pimpl->deviceReference.at(baseDeviceId);

        // Only the inter-device edges are the result - the intra-device geometry stays with the device. Emitting them
        // as a star around the reference device keeps the rig a forest and every edge independent.
        for(const auto& deviceId : componentDevices) {
            if(deviceId == baseDeviceId) continue;
            const auto& deviceReferenceFrame = pimpl->deviceReference.at(deviceId);

            BestEdge best;
            const auto evaluate = [&](const std::vector<size_t>& subset,
                                      float yawDegrees,
                                      DynamicCalibrationControl::PerformanceMode mode,
                                      bool keepCenters,
                                      const char* approach) {
                const auto guesses = pimpl->guessesWithYaw(deviceId, yawDegrees);
                std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> sensors;
                sensors.reserve(subset.size());
                for(const auto index : subset) {
                    pimpl->reseed(cameras[index], pimpl->seedToRigBase(cameras[index].frame, guesses));
                    sensors.push_back(cameras[index].sensor);
                }

                auto result =
                    pimpl->dcl.findNewCalibration(sensors, DclUtils::daiPerformanceModeToDclPerformanceMode(mode), keepCenters, pimpl->baselineEdges(subset));
                if(!result.passed() || result.value.calibrations.size() != sensors.size()) return;

                std::map<CoordinateFrame, Transform> estimated;
                for(size_t k = 0; k < subset.size(); ++k) {
                    estimated.emplace(cameras[subset[k]].frame, DclUtils::calibrationHandleToTransform(result.value.calibrations[k]));
                }
                const auto baseIt = estimated.find(baseReference);
                const auto deviceIt = estimated.find(deviceReferenceFrame);
                if(baseIt == estimated.end() || deviceIt == estimated.end()) return;

                // Score the calibration itself, so SKIP_CHECKS solves that ignore coverage do not win on merit alone.
                double confidence = result.value.dataConfidence;
                auto confResult = pimpl->dcl.computeCalibrationConfidence(result.value.calibrations, sensors);
                if(confResult.passed()) confidence = confResult.value;
                const double sampson = result.value.sampsonErrorNew;

                const bool better =
                    !best.found || confidence > best.confidence + 1e-6 || (std::abs(confidence - best.confidence) <= 1e-6 && sampson < best.sampson);
                if(!better) return;
                best.found = true;
                best.confidence = confidence;
                best.sampson = sampson;
                best.transform = matrix::matMul(baseIt->second, inverted(deviceIt->second));
                best.approach = fmt::format("{}, mode {}, keepCenters {}, yaw {:+.0f}deg", approach, static_cast<int>(mode), keepCenters, yawDegrees);
            };

            // Pairwise: only the reference device and this one, which is robust when not every pair shares a scene.
            std::vector<size_t> pairSubset = byDevice.at(baseDeviceId);
            pairSubset.insert(pairSubset.end(), byDevice.at(deviceId).begin(), byDevice.at(deviceId).end());
            for(const float yaw : yawOffsets) {
                for(const auto mode : modes) {
                    for(const bool keepCenters : keepCentersOptions) {
                        evaluate(pairSubset, yaw, mode, keepCenters, "pairwise");
                    }
                }
            }

            // Joint: solve all cameras of the component together, letting the other devices constrain this one.
            if(componentDevices.size() > 2) {
                const std::vector<size_t> jointSubset(indices.begin(), indices.end());
                for(const auto mode : modes) {
                    for(const bool keepCenters : keepCentersOptions) {
                        evaluate(jointSubset, 0.0f, mode, keepCenters, "joint");
                    }
                }
            }

            if(!best.found) {
                notes.push_back(fmt::format("no strategy produced a valid calibration for device {}", deviceId));
                logger->warn("No strategy produced a valid calibration for device {}", deviceId);
                continue;
            }

            auto transform = best.transform;
            const auto scale = resolveScale(transform, baseReference, deviceReferenceFrame, notes);
            for(int i = 0; i < 3; ++i) {
                transform[i][3] *= scale;
            }
            auto translation = matrix::extractTranslationVector(transform);
            for(auto& value : translation) {
                value *= getDistanceUnitScale(LengthUnit::CENTIMETER, LengthUnit::METER);
            }

            RigEdge edge;
            edge.from = deviceReferenceFrame;
            edge.to = baseReference;
            edge.transform =
                Extrinsics(matrix::extractRotationMatrix(transform), Point3f(translation[0], translation[1], translation[2]), baseReference.socket);
            edge.transform.setReferenceFrame(baseReference);
            edge.timestamp = rig.timestamp;
            edge.source = "multi-device-calibration";
            rig.edges.push_back(edge);
            notes.push_back(fmt::format("device {}: {} (confidence {:.3f}, Sampson {:.4f})", deviceId, best.approach, best.confidence, best.sampson));
            logger->info("Device {} rig edge from {} (confidence {:.3f}, Sampson error {:.4f})", deviceId, best.approach, best.confidence, best.sampson);
        }
    }

    const auto info = fmt::format("{}", fmt::join(notes, "; "));
    if(rig.edges.empty()) {
        logger->warn("No inter-device transformation could be estimated: {}", info);
        rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(info));
        return;
    }
    logger->info("Estimated {} inter-device transformation(s){}{}", rig.edges.size(), info.empty() ? "" : ", ", info);
    rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(rig, dataConfidence, info));
}

void MultiDeviceCalibration::estimateFromTracks() {
    auto& logger = ThreadedNode::pimpl->logger;
    const auto& cameras = pimpl->cameras;

    // Each device is a stereo pair: its reference camera (first registered) is the left, the second is the right.
    struct DeviceCameras {
        std::string deviceId;
        size_t leftIndex = 0;
        size_t rightIndex = 0;
        bool hasRight = false;
    };
    std::vector<DeviceCameras> devices;
    const auto deviceIndex = [&devices](const std::string& deviceId) -> DeviceCameras& {
        for(auto& device : devices)
            if(device.deviceId == deviceId) return device;
        devices.push_back(DeviceCameras{deviceId, 0, 0, false});
        return devices.back();
    };
    for(size_t i = 0; i < cameras.size(); ++i) {
        auto& device = deviceIndex(cameras[i].frame.deviceId);
        if(cameras[i].frame == pimpl->deviceReference.at(cameras[i].frame.deviceId)) {
            device.leftIndex = i;
        } else if(!device.hasRight) {
            device.rightIndex = i;
            device.hasRight = true;
        }
    }
    for(const auto& device : devices) {
        DAI_CHECK_V(device.hasRight,
                    "MultiDeviceCalibration FEATURE_TRACKS needs a stereo pair (two cameras) per device, but device {} has only one registered camera",
                    device.deviceId);
    }

    const auto toVecMatrix = [](const cv::Mat& matrix) {
        std::vector<std::vector<float>> values(4, std::vector<float>(4, 0.0f));
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < 4; ++j) values[i][j] = static_cast<float>(matrix.at<double>(i, j));
        return values;
    };

    // Factory intrinsics/distortion and metric stereo pose of every device.
    std::map<std::string, StereoGeometry> geometries;
    for(const auto& device : devices) {
        const auto& leftCamera = cameras[device.leftIndex];
        const auto& rightCamera = cameras[device.rightIndex];
        geometries.emplace(device.deviceId,
                           deviceGeometry(pimpl->calibrations.at(device.deviceId),
                                          leftCamera.frame.socket,
                                          rightCamera.frame.socket,
                                          static_cast<int>(leftCamera.resolution.first),
                                          static_cast<int>(leftCamera.resolution.second)));
    }

    const std::string baseDeviceId = pimpl->baseFrame.deviceId;
    const FeatureConfig config;

    // Grey-scale views of every device, ready for feature detection.
    const auto viewsOf = [&](size_t cameraIndex) {
        std::vector<cv::Mat> views;
        views.reserve(pimpl->trackFrames[cameraIndex].size());
        for(const auto& frame : pimpl->trackFrames[cameraIndex]) views.push_back(frame->getCvFrame());
        return views;
    };

    MultiDeviceCalibrationData rig;
    rig.timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::vector<std::string> notes;

    // Spanning tree: start from the reference device and attach every other device through the pair estimate with the
    // most four-view tracks, so a badly overlapping direct pair can still be reached by chaining through a third one.
    std::map<std::string, cv::Mat> baseFromDevice;  // 4x4, meters
    baseFromDevice.emplace(baseDeviceId, cv::Mat::eye(4, 4, CV_64F));

    bool progress = true;
    while(progress) {
        progress = false;
        for(const auto& device : devices) {
            if(baseFromDevice.count(device.deviceId)) continue;
            TracksPoseResult bestResult;
            std::string bestReference;
            for(const auto& reference : devices) {
                if(!baseFromDevice.count(reference.deviceId)) continue;
                const auto result = computeRelativePose(viewsOf(reference.leftIndex),
                                                        viewsOf(reference.rightIndex),
                                                        viewsOf(device.leftIndex),
                                                        viewsOf(device.rightIndex),
                                                        geometries.at(reference.deviceId),
                                                        geometries.at(device.deviceId),
                                                        config);
                if(!result.ok) {
                    logger->trace("Pair {} <- {} failed: {}", reference.deviceId, device.deviceId, result.error);
                    continue;
                }
                if(!bestResult.ok || result.numTracks > bestResult.numTracks) {
                    bestResult = result;
                    bestReference = reference.deviceId;
                }
            }
            if(!bestResult.ok) continue;

            baseFromDevice.emplace(device.deviceId, baseFromDevice.at(bestReference) * bestResult.refFromOther);
            const double translationMagnitude = cv::norm(bestResult.refFromOther(cv::Rect(3, 0, 1, 3)));
            notes.push_back(fmt::format("device {} <- {}: {} tracks, reproj RMSE {:.3f} px, scale RMSE {:.3f} m, |t| {:.3f} m",
                                        bestReference,
                                        device.deviceId,
                                        bestResult.numTracks,
                                        bestResult.reprojRmsePx,
                                        bestResult.scaleRmseM,
                                        translationMagnitude));
            logger->info("Estimated pose of device {} relative to {}: {} tracks, reproj RMSE {:.3f} px, scale RMSE {:.3f} m, |t| {:.3f} m",
                         device.deviceId,
                         bestReference,
                         bestResult.numTracks,
                         bestResult.reprojRmsePx,
                         bestResult.scaleRmseM,
                         translationMagnitude);
            progress = true;
        }
    }

    const auto& baseReference = pimpl->deviceReference.at(baseDeviceId);
    for(const auto& device : devices) {
        if(device.deviceId == baseDeviceId) continue;
        const auto found = baseFromDevice.find(device.deviceId);
        if(found == baseFromDevice.end()) {
            notes.push_back(fmt::format("device {} could not be connected to the reference device {}", device.deviceId, baseDeviceId));
            logger->warn("Device {} could not be connected to the reference device {}", device.deviceId, baseDeviceId);
            continue;
        }
        const auto& deviceReferenceFrame = pimpl->deviceReference.at(device.deviceId);

        RigEdge edge;
        edge.from = deviceReferenceFrame;
        edge.to = baseReference;
        edge.transform = Extrinsics(toVecMatrix(found->second), baseReference.socket, LengthUnit::METER);
        edge.transform.setReferenceFrame(baseReference);
        edge.timestamp = rig.timestamp;
        edge.source = "tracks-relative-pose";
        rig.edges.push_back(edge);
    }

    const auto info = fmt::format("{}", fmt::join(notes, "; "));
    if(rig.edges.empty()) {
        logger->warn("No inter-device transformation could be estimated: {}", info);
        rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(info));
        return;
    }
    logger->info("Estimated {} inter-device transformation(s) from feature tracks{}{}", rig.edges.size(), info.empty() ? "" : ", ", info);
    rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(rig, 1.0, info));
}

float MultiDeviceCalibration::resolveScale(const std::vector<std::vector<float>>& transform,
                                           const CoordinateFrame& baseReference,
                                           const CoordinateFrame& reference,
                                           std::vector<std::string>& notes) const {
    // The translation between cameras of different devices is only observable up to scale, so it has to be fixed by a
    // known distance. Scaling the edge translation by `s` moves the whole device along the edge, so the distance
    // between a camera of the base device (center `centerA`, unaffected) and one of the moved device is
    // ||s * t + v|| - one quadratic equation in `s`.
    for(const auto& [pair, distance] : pimpl->knownDistances) {
        for(const auto ordered : {std::pair{pair.first, pair.second}, std::pair{pair.second, pair.first}}) {
            const auto& a = ordered.first;
            const auto& b = ordered.second;
            if(a.deviceId != baseReference.deviceId || b.deviceId != reference.deviceId) continue;

            // Centers expressed in the base device's reference frame
            const auto centerA = cameraCenter(pimpl->referenceToCamera(a));
            const auto translation = matrix::extractTranslationVector(transform);
            const auto centerB = cameraCenter(matrix::matMul(pimpl->referenceToCamera(b), inverted(transform)));

            std::vector<float> offset(3);
            for(int i = 0; i < 3; ++i) {
                offset[i] = centerB[i] - translation[i] - centerA[i];
            }
            const float a2 = dot(translation, translation);
            const float b2 = 2.0f * dot(translation, offset);
            const float c2 = dot(offset, offset) - distance * distance;
            const float discriminant = b2 * b2 - 4.0f * a2 * c2;
            if(a2 <= 0.0f || discriminant < 0.0f) {
                notes.push_back(fmt::format("the known distance between {} and {} cannot be matched, keeping the estimated scale", toString(a), toString(b)));
                return 1.0f;
            }
            const float rootDiscriminant = std::sqrt(discriminant);
            const float first = (-b2 + rootDiscriminant) / (2.0f * a2);
            const float second = (-b2 - rootDiscriminant) / (2.0f * a2);
            const float scale = std::abs(first - 1.0f) <= std::abs(second - 1.0f) ? first : second;
            if(scale <= 0.0f) {
                notes.push_back(
                    fmt::format("the known distance between {} and {} implies a non-positive scale, keeping the estimated one", toString(a), toString(b)));
                return 1.0f;
            }
            return scale;
        }
    }

    notes.push_back(
        fmt::format("no known distance fixes the scale between {} and {}, so the magnitude of the initial guess is kept - supply one with setKnownDistance()",
                    toString(baseReference),
                    toString(reference)));
    return 1.0f;
}

#else

void MultiDeviceCalibration::run() {
    DAI_CHECK_V(false, "MultiDeviceCalibration requires depthai to be built with OpenCV support");
}

void MultiDeviceCalibration::estimate() {}

void MultiDeviceCalibration::estimateFromTracks() {}

float MultiDeviceCalibration::resolveScale(const std::vector<std::vector<float>>&,
                                           const CoordinateFrame&,
                                           const CoordinateFrame&,
                                           std::vector<std::string>&) const {
    return 1.0f;
}

#endif

}  // namespace node
}  // namespace dai
