#pragma once

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <opencv2/core.hpp>
    #include <string>
    #include <vector>

namespace dai {
namespace impl {

/// Tuning for the inter-device metric-scale estimation.
struct ScaleParams {
    double ratio = 0.8;          ///< Lowe ratio for descriptor matching.
    double stereoGatePx = 1.5;   ///< Max reprojection error (px) for an accepted stereo point.
    double depthMin = 0.4;       ///< Min triangulated depth (m); rejects unreliable near points.
    double depthMax = 12.0;      ///< Max triangulated depth (m); rejects unreliable far points.
    double dedupPx = 2.0;        ///< ASIFT keypoint deduplication bucket size (px).
    int minCorrespondences = 8;  ///< Below this many four-view correspondences the scale is unobservable.
    int minInliers = 12;         ///< Quality gate: minimum robust inliers.
    double maxRmse = 0.30;       ///< Quality gate: maximum alignment RMSE (m).
    bool useAsift = true;        ///< Affine-simulated SIFT, needed for heterogeneous FOV/viewpoint.
    unsigned ransacSeed = 0;     ///< Deterministic RANSAC seeding.
};

/// A single device seen as a calibrated stereo pair, with per-frame images.
struct StereoDeviceViews {
    std::vector<cv::Mat> left;   ///< Per-frame images of the reference (left / CAM_B) camera.
    std::vector<cv::Mat> right;  ///< Per-frame images of the second (right / CAM_C) camera.
    cv::Matx33d intrinsicsLeft = cv::Matx33d::eye();
    cv::Matx33d intrinsicsRight = cv::Matx33d::eye();
    std::vector<double> distortionLeft;   ///< Up to 8 distortion coefficients.
    std::vector<double> distortionRight;  ///< Up to 8 distortion coefficients.
    cv::Matx44d leftFromRight = cv::Matx44d::eye();  ///< Metric stereo transform (meters).
};

/// Outcome of the inter-device metric-scale estimation.
struct InterDeviceScaleResult {
    bool observable = false;                        ///< True only if the scene constrains the scale and gates pass.
    double distanceMeters = 0.0;                    ///< Distance between the two devices' reference camera centers.
    cv::Matx33d rotationAFromB = cv::Matx33d::eye();  ///< R with X_A ~= R * X_B + t.
    cv::Vec3d translationAFromB{0.0, 0.0, 0.0};       ///< t with X_A ~= R * X_B + t (meters).
    int correspondences = 0;                        ///< Number of four-view metric correspondences found.
    int inliers = 0;                                ///< Robust inliers of the rigid fit.
    double rmseMeters = 0.0;                         ///< RMSE of the rigid fit (meters).
    std::string note;                               ///< Human-readable explanation, esp. when unobservable.
};

/// Estimate the metric distance between two devices' reference cameras purely from a shared scene, using each
/// device's known stereo baseline to fix scale: ASIFT matching + metric stereo triangulation + four-view
/// descriptor association + robust 3D-3D rigid alignment. Reports `observable=false` (never a guess) when the
/// scene does not constrain the scale.
InterDeviceScaleResult estimateInterDeviceScale(const StereoDeviceViews& deviceA,
                                                const StereoDeviceViews& deviceB,
                                                const ScaleParams& params = {});

// --- Lower-level pieces, exposed for unit testing ---

/// Robust rigid alignment A ~= R*B + t via Kabsch + RANSAC + Huber IRLS refinement.
struct RigidFit {
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d t{0.0, 0.0, 0.0};
    std::vector<char> inliers;
    double rmse = 0.0;
    bool ok = false;
};
RigidFit kabschRansac(const std::vector<cv::Point3d>& pointsA,
                      const std::vector<cv::Point3d>& pointsB,
                      double threshold,
                      int iterations = 2000,
                      unsigned seed = 0);

/// Closed-form least-squares rigid transform A ~= R*B + t (no outlier rejection).
bool kabsch(const std::vector<cv::Point3d>& pointsA, const std::vector<cv::Point3d>& pointsB, cv::Matx33d& R, cv::Vec3d& t);

/// Metric triangulation of matched pixel pairs from already-undistorted images (pinhole K, no distortion).
/// Points are returned in the left camera frame; `keep` gates by reprojection error and positive depth.
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
