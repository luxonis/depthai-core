#pragma once

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/warpers.hpp>

#include "depthai/pipeline/node/host/Stitching.hpp"

namespace dai {
namespace node {

/**
 * OpenCV stitching defaults shared by the panorama and planar projection implementations.
 */
namespace stitching {

constexpr double REGISTRATION_RESOLUTION = 0.6;
constexpr double SEAM_ESTIMATION_RESOLUTION = 0.1;
constexpr double COMPOSITING_RESOLUTION = -1.0;
constexpr float MATCH_CONFIDENCE = 0.65f;
constexpr float BLEND_STRENGTH = 5.0f;

inline cv::Ptr<cv::Feature2D> createFeaturesFinder() {
    return cv::SIFT::create();
}

inline cv::Ptr<cv::detail::FeaturesMatcher> createFeaturesMatcher() {
    return cv::makePtr<cv::detail::BestOf2NearestMatcher>(false, MATCH_CONFIDENCE);
}

inline cv::Ptr<cv::detail::Estimator> createEstimator() {
    return cv::makePtr<cv::detail::HomographyBasedEstimator>();
}

inline cv::Ptr<cv::detail::BundleAdjusterBase> createBundleAdjuster() {
    return cv::makePtr<cv::detail::BundleAdjusterRay>();
}

inline cv::Ptr<cv::WarperCreator> createWarper(Stitching::CameraModel model) {
    switch(model) {
        case Stitching::CameraModel::SPHERICAL:
            return cv::makePtr<cv::SphericalWarper>();
        case Stitching::CameraModel::PINHOLE:
            return cv::makePtr<cv::PlaneWarper>();
        case Stitching::CameraModel::CYLINDRICAL:
            return cv::makePtr<cv::CylindricalWarper>();
    }
    return cv::makePtr<cv::SphericalWarper>();
}

inline cv::Ptr<cv::detail::SeamFinder> createSeamFinder(Stitching::SeamFinder finder) {
    switch(finder) {
        case Stitching::SeamFinder::NONE:
            return cv::makePtr<cv::detail::NoSeamFinder>();
        case Stitching::SeamFinder::VORONOI:
            return cv::makePtr<cv::detail::VoronoiSeamFinder>();
        case Stitching::SeamFinder::DP_COLOR:
            return cv::makePtr<cv::detail::DpSeamFinder>(cv::detail::DpSeamFinder::COLOR);
        case Stitching::SeamFinder::DP_COLOR_GRAD:
            return cv::makePtr<cv::detail::DpSeamFinder>(cv::detail::DpSeamFinder::COLOR_GRAD);
        case Stitching::SeamFinder::GRAPHCUT_COLOR:
            return cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
        case Stitching::SeamFinder::GRAPHCUT_COLOR_GRAD:
            return cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR_GRAD);
    }
    return cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
}

/**
 * @param panoSizeHint Size the blending width is derived from, the way OpenCV's stitching_detailed sample does it
 */
inline cv::Ptr<cv::detail::Blender> createBlender(const cv::Size& panoSizeHint) {
    const float blendWidth = std::sqrt(static_cast<float>(panoSizeHint.area())) * BLEND_STRENGTH / 100.0f;
    if(blendWidth < 1.0f) {
        return cv::detail::Blender::createDefault(cv::detail::Blender::NO, false);
    }
    auto multiBand = cv::makePtr<cv::detail::MultiBandBlender>();
    multiBand->setNumBands(static_cast<int>(std::ceil(std::log(static_cast<double>(blendWidth)) / std::log(2.0)) - 1.0));
    return multiBand;
}

}  // namespace stitching
}  // namespace node
}  // namespace dai
