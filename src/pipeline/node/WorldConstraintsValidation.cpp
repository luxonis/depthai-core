#include "WorldConstraintsValidation.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <numeric>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/calib3d.hpp>
    #include <opencv2/core.hpp>
#endif

namespace dai {
namespace node {
namespace detail {

bool ConstraintResidual::fits() const {
    constexpr double MAX_RELATIVE_ERROR = 0.15;
    constexpr double MAX_ERROR_METERS = 0.01;
    return measured && std::isfinite(*measured) && std::isfinite(configured) && configured > 0.0
           && std::abs(*measured - configured) <= std::max(MAX_RELATIVE_ERROR * configured, MAX_ERROR_METERS);
}

bool epipolarErrorContradictsConstraints(double currentPx, double newPx) {
    constexpr double MAX_ERROR_PX = 0.5;
    constexpr double MAX_ERROR_GROWTH = 2.0;
    return !std::isfinite(currentPx) || !std::isfinite(newPx) || currentPx < 0.0 || newPx < 0.0
           || (newPx > MAX_ERROR_PX && newPx > MAX_ERROR_GROWTH * currentPx);
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
namespace {
/// A camera of the consistency check in OpenCV form.
struct ReconstructionCamera {
    cv::Matx33d intrinsics;
    std::vector<double> distortion;
    bool fisheye = false;
    cv::Matx33d rotation;  // base to camera
    cv::Vec3d translation;

    /// The pixels as points of the normalized image plane, lens distortion removed.
    std::vector<cv::Point2d> normalized(const std::vector<cv::Point2d>& pixels) const {
        std::vector<cv::Point2d> points;
        if(pixels.empty()) {
            return points;
        }
        if(fisheye) {
            cv::fisheye::undistortPoints(pixels, points, intrinsics, distortion);
        } else {
            cv::undistortPoints(pixels, points, intrinsics, distortion);
        }
        return points;
    }
    cv::Vec3d toCamera(const cv::Vec3d& basePoint) const {
        return rotation * basePoint + translation;
    }
    cv::Vec3d center() const {
        return -(rotation.t() * translation);
    }
};

ReconstructionCamera reconstructionCamera(const CalibrationHandler& calibration,
                                          CameraBoardSocket base,
                                          CameraBoardSocket socket,
                                          const ObservationCamera& observed) {
    const auto& intrinsics = observed.intrinsics;
    const auto pose = calibration.getCameraExtrinsics(base, socket, false, LengthUnit::METER);
    const auto& distortion = observed.distortion;
    ReconstructionCamera camera;
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            camera.intrinsics(row, col) = intrinsics.at(row).at(col);
            camera.rotation(row, col) = pose.at(row).at(col);
        }
        camera.translation[row] = pose.at(row).at(3);
    }
    camera.fisheye = observed.distortionModel == CameraModel::Fisheye;
    const size_t count = camera.fisheye ? 4
                                        : (distortion.size() >= 14   ? 14
                                           : distortion.size() >= 12 ? 12
                                           : distortion.size() >= 8  ? 8
                                           : distortion.size() >= 5  ? 5
                                           : distortion.size() >= 4  ? 4
                                                                     : 0);
    camera.distortion.assign(distortion.begin(), distortion.begin() + static_cast<std::ptrdiff_t>(std::min(count, distortion.size())));
    if(camera.fisheye) camera.distortion.resize(4, 0.0);
    return camera;
}

/// The point of the base frame seen at the given normalized image points, by linear triangulation; none when fewer
/// than two cameras see it or it does not lie in front of all of them.
std::optional<cv::Vec3d> triangulate(const std::vector<std::pair<const ReconstructionCamera*, cv::Point2d>>& views) {
    if(views.size() < 2) {
        return std::nullopt;
    }
    cv::Mat system(static_cast<int>(2 * views.size()), 4, CV_64F);
    for(size_t idx = 0; idx < views.size(); ++idx) {
        const auto& [camera, point] = views[idx];
        for(int col = 0; col < 4; ++col) {
            const auto entry = [&](int row) { return col < 3 ? camera->rotation(row, col) : camera->translation[row]; };
            system.at<double>(static_cast<int>(2 * idx), col) = point.x * entry(2) - entry(0);
            system.at<double>(static_cast<int>(2 * idx + 1), col) = point.y * entry(2) - entry(1);
        }
    }
    cv::Mat solution;
    cv::SVD::solveZ(system, solution);
    const double w = solution.at<double>(3);
    if(std::abs(w) < 1e-12) {
        return std::nullopt;
    }
    const cv::Vec3d point(solution.at<double>(0) / w, solution.at<double>(1) / w, solution.at<double>(2) / w);
    for(const auto& view : views) {
        if(!(view.first->toCamera(point)[2] > 0.0)) {
            return std::nullopt;
        }
    }
    return point;
}

std::optional<double> median(std::vector<double> values) {
    values.erase(std::remove_if(values.begin(), values.end(), [](double value) { return !std::isfinite(value); }), values.end());
    if(values.empty()) {
        return std::nullopt;
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

}  // namespace

std::vector<ConstraintResidual> worldConstraintResiduals(const WorldConstraints& constraints,
                                                         const std::deque<MarkerObservationSet>& observationSets,
                                                         const CalibrationHandler& calibration,
                                                         CameraBoardSocket base,
                                                         const std::map<CameraBoardSocket, ObservationCamera>& cameras) {
    std::map<CameraBoardSocket, ReconstructionCamera> converted;
    for(const auto& [socket, observed] : cameras) {
        converted.emplace(socket, reconstructionCamera(calibration, base, socket, observed));
    }
    // A referenced socket need not supply images, but its final camera center is still needed.
    for(const auto& distance : constraints.socketDistances) {
        if(converted.count(distance.socket) == 0) {
            converted.emplace(distance.socket,
                              reconstructionCamera(calibration,
                                                   base,
                                                   distance.socket,
                                                   {calibration.getCameraIntrinsics(distance.socket),
                                                    calibration.getDistortionCoefficients(distance.socket),
                                                    calibration.getDistortionModel(distance.socket)}));
        }
    }
    const auto toPixels = [](const std::vector<std::array<float, 2>>& points) {
        std::vector<cv::Point2d> pixels;
        pixels.reserve(points.size());
        for(const auto& point : points) {
            pixels.emplace_back(point[0], point[1]);
        }
        return pixels;
    };

    // Per configured measurement, in configuration order, its reconstructed value in every frame set.
    std::vector<std::vector<double>> markerDistances(constraints.markerDistances.size());
    std::vector<std::vector<double>> socketDistances(constraints.socketDistances.size());
    std::vector<std::vector<double>> markerSizes(constraints.markerSizes.size());
    for(const auto& set : observationSets) {
        std::map<int32_t, cv::Vec3d> points;
        for(const auto& [markerId, centers] : set.centers) {
            std::vector<std::pair<const ReconstructionCamera*, cv::Point2d>> views;
            for(const auto& [socket, center] : centers) {
                const auto camera = converted.find(socket);
                if(camera != converted.end()) {
                    views.emplace_back(&camera->second, camera->second.normalized({cv::Point2d(center[0], center[1])}).front());
                }
            }
            if(const auto point = triangulate(views)) {
                points.emplace(markerId, *point);
            }
        }
        for(size_t idx = 0; idx < constraints.markerDistances.size(); ++idx) {
            const auto a = points.find(constraints.markerDistances[idx].markerA);
            const auto b = points.find(constraints.markerDistances[idx].markerB);
            if(a != points.end() && b != points.end()) {
                markerDistances[idx].push_back(cv::norm(a->second - b->second));
            }
        }
        for(size_t idx = 0; idx < constraints.socketDistances.size(); ++idx) {
            const auto point = points.find(constraints.socketDistances[idx].marker);
            const auto camera = converted.find(constraints.socketDistances[idx].socket);
            if(point != points.end() && camera != converted.end()) {
                socketDistances[idx].push_back(cv::norm(point->second - camera->second.center()));
            }
        }
        for(size_t idx = 0; idx < constraints.markerSizes.size(); ++idx) {
            const auto markerId = constraints.markerSizes[idx].marker;
            const auto outlines = set.outlines.find(markerId);
            const auto point = points.find(markerId);
            if(outlines == set.outlines.end() || point == points.end()) {
                continue;
            }
            if(constraints.markerType == MarkerType::ARUCO) {
                // The side of the square between its triangulated corners.
                std::vector<cv::Vec3d> corners;
                for(size_t corner = 0; corner < 4; ++corner) {
                    std::vector<std::pair<const ReconstructionCamera*, cv::Point2d>> views;
                    for(const auto& [socket, outline] : outlines->second) {
                        const auto camera = converted.find(socket);
                        if(camera != converted.end() && outline.corners.size() == 4) {
                            views.emplace_back(&camera->second, camera->second.normalized(toPixels({outline.corners[corner]})).front());
                        }
                    }
                    if(const auto position = triangulate(views)) {
                        corners.push_back(*position);
                    }
                }
                if(corners.size() == 4) {
                    double perimeter = 0.0;
                    for(size_t corner = 0; corner < 4; ++corner) {
                        perimeter += cv::norm(corners[corner] - corners[(corner + 1) % 4]);
                    }
                    markerSizes[idx].push_back(perimeter / 4.0);
                }
            } else {
                // The ring is a circle: its largest angular radius is not foreshortened, so it gives the diameter at the
                // triangulated range of the center.
                std::vector<double> diameters;
                const auto centers = set.centers.find(markerId);
                for(const auto& [socket, outline] : outlines->second) {
                    const auto camera = converted.find(socket);
                    if(camera == converted.end() || outline.outerEdge.empty() || centers == set.centers.end() || centers->second.count(socket) == 0) {
                        continue;
                    }
                    const auto& centerPx = centers->second.at(socket);
                    const auto centerPoint = camera->second.normalized({cv::Point2d(centerPx[0], centerPx[1])}).front();
                    const auto centerRay = cv::normalize(cv::Vec3d(centerPoint.x, centerPoint.y, 1.0));
                    double largestAngle = 0.0;
                    for(const auto& edge : camera->second.normalized(toPixels(outline.outerEdge))) {
                        const auto ray = cv::normalize(cv::Vec3d(edge.x, edge.y, 1.0));
                        largestAngle = std::max(largestAngle, std::acos(std::clamp(ray.dot(centerRay), -1.0, 1.0)));
                    }
                    diameters.push_back(2.0 * cv::norm(camera->second.toCamera(point->second)) * std::tan(largestAngle));
                }
                if(!diameters.empty()) {
                    markerSizes[idx].push_back(std::accumulate(diameters.begin(), diameters.end(), 0.0) / static_cast<double>(diameters.size()));
                }
            }
        }
    }

    std::vector<ConstraintResidual> residuals;
    for(size_t idx = 0; idx < constraints.markerDistances.size(); ++idx) {
        const auto& distance = constraints.markerDistances[idx];
        residuals.push_back(
            {fmt::format("the distance between markers {} and {}", distance.markerA, distance.markerB), distance.distance, median(markerDistances[idx])});
    }
    for(size_t idx = 0; idx < constraints.socketDistances.size(); ++idx) {
        const auto& distance = constraints.socketDistances[idx];
        residuals.push_back({fmt::format("the distance between marker {} and {}", distance.marker, toString(distance.socket)),
                             distance.distance,
                             median(socketDistances[idx])});
    }
    for(size_t idx = 0; idx < constraints.markerSizes.size(); ++idx) {
        residuals.push_back(
            {fmt::format("the size of marker {}", constraints.markerSizes[idx].marker), constraints.markerSizes[idx].size, median(markerSizes[idx])});
    }
    return residuals;
}
#endif

}  // namespace detail
}  // namespace node
}  // namespace dai
