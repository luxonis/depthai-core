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
 * Translation of the Stitching node settings into the OpenCV objects doing the work. Shared by the panorama and the
 * planar projection implementations.
 */
namespace stitching {

inline cv::InterpolationFlags toInterpolationFlag(Stitching::Interpolation interpolation) {
    switch(interpolation) {
        case Stitching::Interpolation::NEAREST:
            return cv::INTER_NEAREST;
        case Stitching::Interpolation::LINEAR:
            return cv::INTER_LINEAR;
        case Stitching::Interpolation::CUBIC:
            return cv::INTER_CUBIC;
        case Stitching::Interpolation::AREA:
            return cv::INTER_AREA;
        case Stitching::Interpolation::LANCZOS4:
            return cv::INTER_LANCZOS4;
    }
    return cv::INTER_LINEAR;
}

inline int toExposureCompensatorType(Stitching::ExposureCompensator compensator) {
    switch(compensator) {
        case Stitching::ExposureCompensator::NONE:
            return cv::detail::ExposureCompensator::NO;
        case Stitching::ExposureCompensator::GAIN:
            return cv::detail::ExposureCompensator::GAIN;
        case Stitching::ExposureCompensator::GAIN_BLOCKS:
            return cv::detail::ExposureCompensator::GAIN_BLOCKS;
        case Stitching::ExposureCompensator::CHANNELS:
            return cv::detail::ExposureCompensator::CHANNELS;
        case Stitching::ExposureCompensator::CHANNELS_BLOCKS:
            return cv::detail::ExposureCompensator::CHANNELS_BLOCKS;
    }
    return cv::detail::ExposureCompensator::GAIN_BLOCKS;
}

inline cv::Ptr<cv::Feature2D> createFeaturesFinder(Stitching::FeaturesFinder finder) {
    switch(finder) {
        case Stitching::FeaturesFinder::ORB:
            return cv::ORB::create();
        case Stitching::FeaturesFinder::SIFT:
            return cv::SIFT::create();
        case Stitching::FeaturesFinder::AKAZE:
            return cv::AKAZE::create();
        case Stitching::FeaturesFinder::BRISK:
            return cv::BRISK::create();
    }
    return cv::ORB::create();
}

inline cv::Ptr<cv::detail::FeaturesMatcher> createFeaturesMatcher(Stitching::FeaturesMatcher matcher, Stitching::FeaturesFinder finder, float matchConfidence) {
    // Matches OpenCV's own defaults when the user did not override the confidence
    const float confidence = matchConfidence >= 0.0f ? matchConfidence : (finder == Stitching::FeaturesFinder::ORB ? 0.3f : 0.65f);
    if(matcher == Stitching::FeaturesMatcher::AFFINE) {
        return cv::makePtr<cv::detail::AffineBestOf2NearestMatcher>(false, false, confidence);
    }
    return cv::makePtr<cv::detail::BestOf2NearestMatcher>(false, confidence);
}

inline cv::Ptr<cv::detail::Estimator> createEstimator(Stitching::Estimator estimator) {
    if(estimator == Stitching::Estimator::AFFINE) {
        return cv::makePtr<cv::detail::AffineBasedEstimator>();
    }
    return cv::makePtr<cv::detail::HomographyBasedEstimator>();
}

inline cv::Ptr<cv::detail::BundleAdjusterBase> createBundleAdjuster(Stitching::BundleAdjuster adjuster) {
    switch(adjuster) {
        case Stitching::BundleAdjuster::NONE:
            return cv::makePtr<cv::detail::NoBundleAdjuster>();
        case Stitching::BundleAdjuster::RAY:
            return cv::makePtr<cv::detail::BundleAdjusterRay>();
        case Stitching::BundleAdjuster::REPROJECTION:
            return cv::makePtr<cv::detail::BundleAdjusterReproj>();
        case Stitching::BundleAdjuster::AFFINE:
            return cv::makePtr<cv::detail::BundleAdjusterAffine>();
        case Stitching::BundleAdjuster::AFFINE_PARTIAL:
            return cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();
    }
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
inline cv::Ptr<cv::detail::Blender> createBlender(Stitching::Blender blender, float blendStrength, const cv::Size& panoSizeHint) {
    const float blendWidth = std::sqrt(static_cast<float>(panoSizeHint.area())) * blendStrength / 100.0f;
    if(blender == Stitching::Blender::NONE || blendWidth < 1.0f) {
        return cv::detail::Blender::createDefault(cv::detail::Blender::NO, false);
    }
    if(blender == Stitching::Blender::FEATHER) {
        auto feather = cv::makePtr<cv::detail::FeatherBlender>();
        feather->setSharpness(1.0f / blendWidth);
        return feather;
    }
    auto multiBand = cv::makePtr<cv::detail::MultiBandBlender>();
    multiBand->setNumBands(static_cast<int>(std::ceil(std::log(static_cast<double>(blendWidth)) / std::log(2.0)) - 1.0));
    return multiBand;
}

}  // namespace stitching
}  // namespace node
}  // namespace dai
