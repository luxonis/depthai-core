#pragma once

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <opencv2/core.hpp>
    #include <string>
    #include <vector>

namespace dai {
namespace impl {

/** Tuning for inter-device metric-scale estimation. */
struct ScaleParams {
    double ratio = 0.8;
    double stereoGatePx = 1.5;
    double depthMin = 0.4;
    double depthMax = 12.0;
    double dedupPx = 2.0;
    int minCorrespondences = 8;
    int minInliers = 12;
    double maxRmse = 0.30;
    bool useAsift = true;
    unsigned ransacSeed = 0;
};

/** One device represented by a calibrated stereo pair and synchronized images. */
struct StereoDeviceViews {
    std::vector<cv::Mat> left;
    std::vector<cv::Mat> right;
    cv::Matx33d intrinsicsLeft = cv::Matx33d::eye();
    cv::Matx33d intrinsicsRight = cv::Matx33d::eye();
    std::vector<double> distortionLeft;
    std::vector<double> distortionRight;
    cv::Matx44d leftFromRight = cv::Matx44d::eye();
};

/** Outcome of metric scale recovery from the shared scene. */
struct InterDeviceScaleResult {
    bool observable = false;
    double distanceMeters = 0.0;
    cv::Matx33d rotationAFromB = cv::Matx33d::eye();
    cv::Vec3d translationAFromB{0.0, 0.0, 0.0};
    int correspondences = 0;
    int inliers = 0;
    double rmseMeters = 0.0;
    std::string note;
};

InterDeviceScaleResult estimateInterDeviceScale(const StereoDeviceViews& deviceA, const StereoDeviceViews& deviceB, const ScaleParams& params = {});

struct RigidFit {
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d t{0.0, 0.0, 0.0};
    std::vector<char> inliers;
    double rmse = 0.0;
    bool ok = false;
};

RigidFit kabschRansac(
    const std::vector<cv::Point3d>& pointsA, const std::vector<cv::Point3d>& pointsB, double threshold, int iterations = 2000, unsigned seed = 0);

bool kabsch(const std::vector<cv::Point3d>& pointsA, const std::vector<cv::Point3d>& pointsB, cv::Matx33d& R, cv::Vec3d& t);

void triangulateMetric(const std::vector<cv::Point2d>& leftPx,
                       const std::vector<cv::Point2d>& rightPx,
                       const cv::Matx33d& intrinsicsLeft,
                       const cv::Matx33d& intrinsicsRight,
                       const cv::Matx44d& leftFromRight,
                       double gatePx,
                       std::vector<cv::Point3d>& pointsLeft,
                       std::vector<char>& keep);

}  // namespace impl
}  // namespace dai

#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT
