#include "depthai/pipeline/node/host/Stitching.hpp"

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/warpers.hpp>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/PimplImpl.hpp"

namespace dai {
namespace node {

namespace {

std::string statusToString(cv::Stitcher::Status status) {
    switch(status) {
        case cv::Stitcher::OK:
            return "OK";
        case cv::Stitcher::ERR_NEED_MORE_IMGS:
            return "need more images, too few overlapping features";
        case cv::Stitcher::ERR_HOMOGRAPHY_EST_FAIL:
            return "homography estimation failed";
        case cv::Stitcher::ERR_CAMERA_PARAMS_ADJUST_FAIL:
            return "camera parameters adjustment failed";
    }
    return "unknown error";
}

/**
 * Chordal L2 mean of rotation matrices: average elementwise, then project back onto SO(3) with SVD.
 */
cv::Mat averageRotations(const std::vector<cv::Mat>& rotations) {
    cv::Mat sum = cv::Mat::zeros(3, 3, CV_64F);
    for(const auto& rotation : rotations) {
        cv::Mat r;
        rotation.convertTo(r, CV_64F);
        sum += r;
    }

    cv::SVD svd(sum / static_cast<double>(rotations.size()), cv::SVD::FULL_UV);
    cv::Mat mean = svd.u * svd.vt;
    if(cv::determinant(mean) < 0) {
        cv::Mat correction = cv::Mat::eye(3, 3, CV_64F);
        correction.at<double>(2, 2) = -1.0;
        mean = svd.u * correction * svd.vt;
    }

    cv::Mat result;
    mean.convertTo(result, CV_32F);
    return result;
}

std::vector<cv::detail::CameraParams> averageCameras(const std::vector<std::vector<cv::detail::CameraParams>>& samples) {
    std::vector<cv::detail::CameraParams> averaged = samples.front();
    const auto count = static_cast<double>(samples.size());

    for(size_t cam = 0; cam < averaged.size(); ++cam) {
        double focal = 0.0;
        double aspect = 0.0;
        double ppx = 0.0;
        double ppy = 0.0;
        cv::Mat translation = cv::Mat::zeros(samples.front()[cam].t.size(), CV_64F);
        std::vector<cv::Mat> rotations;
        rotations.reserve(samples.size());

        for(const auto& sample : samples) {
            focal += sample[cam].focal;
            aspect += sample[cam].aspect;
            ppx += sample[cam].ppx;
            ppy += sample[cam].ppy;
            cv::Mat t;
            sample[cam].t.convertTo(t, CV_64F);
            translation += t;
            rotations.push_back(sample[cam].R);
        }

        averaged[cam].focal = focal / count;
        averaged[cam].aspect = aspect / count;
        averaged[cam].ppx = ppx / count;
        averaged[cam].ppy = ppy / count;
        averaged[cam].t = translation / count;
        averaged[cam].R = averageRotations(rotations);
    }

    return averaged;
}

}  // namespace

class Stitching::Impl {
   public:
    Mode mode = Mode::PANORAMA;
    CameraModel cameraModel = CameraModel::SPHERICAL;
    bool continuous = false;
    uint32_t estimationFrames = 10;

    double registrationResolution = 0.6;
    double seamEstimationResolution = 0.1;
    double compositingResolution = -1.0;
    double panoConfidenceThreshold = 1.0;
    WaveCorrection waveCorrection = WaveCorrection::HORIZONTAL;
    Interpolation interpolation = Interpolation::LINEAR;
    FeaturesFinder featuresFinder = FeaturesFinder::ORB;
    FeaturesMatcher featuresMatcher = FeaturesMatcher::HOMOGRAPHY;
    float matchConfidence = -1.0f;
    Estimator estimator = Estimator::HOMOGRAPHY;
    BundleAdjuster bundleAdjuster = BundleAdjuster::RAY;
    ExposureCompensator exposureCompensator = ExposureCompensator::GAIN_BLOCKS;
    SeamFinder seamFinder = SeamFinder::GRAPHCUT_COLOR;
    Blender blender = Blender::MULTI_BAND;
    float blendStrength = 5.0f;

    cv::Ptr<cv::Stitcher> stitcher;
    std::vector<std::vector<cv::detail::CameraParams>> samples;
    bool transformFixed = false;

    void invalidate() {
        stitcher.release();
        samples.clear();
        transformFixed = false;
    }

    /**
     * (Re)create the stitcher. panoSizeHint is used to scale the blending width the way
     * OpenCV's stitching_detailed sample does.
     */
    void create(const cv::Size& panoSizeHint) {
        stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);

        stitcher->setRegistrationResol(registrationResolution);
        stitcher->setSeamEstimationResol(seamEstimationResolution);
        stitcher->setCompositingResol(compositingResolution);
        stitcher->setPanoConfidenceThresh(panoConfidenceThreshold);
        stitcher->setWaveCorrection(waveCorrection != WaveCorrection::NONE);
        if(waveCorrection != WaveCorrection::NONE) {
            stitcher->setWaveCorrectKind(waveCorrection == WaveCorrection::HORIZONTAL ? cv::detail::WAVE_CORRECT_HORIZ : cv::detail::WAVE_CORRECT_VERT);
        }
        stitcher->setInterpolationFlags(toInterpolationFlag(interpolation));
        stitcher->setFeaturesFinder(createFeaturesFinder());
        stitcher->setFeaturesMatcher(createFeaturesMatcher());
        stitcher->setEstimator(createEstimator());
        stitcher->setBundleAdjuster(createBundleAdjuster());
        stitcher->setWarper(createWarper());
        stitcher->setExposureCompensator(cv::detail::ExposureCompensator::createDefault(toExposureCompensatorType(exposureCompensator)));
        stitcher->setSeamFinder(createSeamFinder());
        stitcher->setBlender(createBlender(panoSizeHint));
    }

   private:
    static cv::InterpolationFlags toInterpolationFlag(Interpolation interpolation) {
        switch(interpolation) {
            case Interpolation::NEAREST:
                return cv::INTER_NEAREST;
            case Interpolation::LINEAR:
                return cv::INTER_LINEAR;
            case Interpolation::CUBIC:
                return cv::INTER_CUBIC;
            case Interpolation::AREA:
                return cv::INTER_AREA;
            case Interpolation::LANCZOS4:
                return cv::INTER_LANCZOS4;
        }
        return cv::INTER_LINEAR;
    }

    static int toExposureCompensatorType(ExposureCompensator compensator) {
        switch(compensator) {
            case ExposureCompensator::NONE:
                return cv::detail::ExposureCompensator::NO;
            case ExposureCompensator::GAIN:
                return cv::detail::ExposureCompensator::GAIN;
            case ExposureCompensator::GAIN_BLOCKS:
                return cv::detail::ExposureCompensator::GAIN_BLOCKS;
            case ExposureCompensator::CHANNELS:
                return cv::detail::ExposureCompensator::CHANNELS;
            case ExposureCompensator::CHANNELS_BLOCKS:
                return cv::detail::ExposureCompensator::CHANNELS_BLOCKS;
        }
        return cv::detail::ExposureCompensator::GAIN_BLOCKS;
    }

    cv::Ptr<cv::Feature2D> createFeaturesFinder() const {
        switch(featuresFinder) {
            case FeaturesFinder::ORB:
                return cv::ORB::create();
            case FeaturesFinder::SIFT:
                return cv::SIFT::create();
            case FeaturesFinder::AKAZE:
                return cv::AKAZE::create();
            case FeaturesFinder::BRISK:
                return cv::BRISK::create();
        }
        return cv::ORB::create();
    }

    cv::Ptr<cv::detail::FeaturesMatcher> createFeaturesMatcher() const {
        // Matches OpenCV's own defaults when the user did not override the confidence
        const float confidence = matchConfidence >= 0.0f ? matchConfidence : (featuresFinder == FeaturesFinder::ORB ? 0.3f : 0.65f);
        if(featuresMatcher == FeaturesMatcher::AFFINE) {
            return cv::makePtr<cv::detail::AffineBestOf2NearestMatcher>(false, false, confidence);
        }
        return cv::makePtr<cv::detail::BestOf2NearestMatcher>(false, confidence);
    }

    cv::Ptr<cv::detail::Estimator> createEstimator() const {
        if(estimator == Estimator::AFFINE) {
            return cv::makePtr<cv::detail::AffineBasedEstimator>();
        }
        return cv::makePtr<cv::detail::HomographyBasedEstimator>();
    }

    cv::Ptr<cv::detail::BundleAdjusterBase> createBundleAdjuster() const {
        switch(bundleAdjuster) {
            case BundleAdjuster::NONE:
                return cv::makePtr<cv::detail::NoBundleAdjuster>();
            case BundleAdjuster::RAY:
                return cv::makePtr<cv::detail::BundleAdjusterRay>();
            case BundleAdjuster::REPROJECTION:
                return cv::makePtr<cv::detail::BundleAdjusterReproj>();
            case BundleAdjuster::AFFINE:
                return cv::makePtr<cv::detail::BundleAdjusterAffine>();
            case BundleAdjuster::AFFINE_PARTIAL:
                return cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();
        }
        return cv::makePtr<cv::detail::BundleAdjusterRay>();
    }

    cv::Ptr<cv::WarperCreator> createWarper() const {
        switch(cameraModel) {
            case CameraModel::SPHERICAL:
                return cv::makePtr<cv::SphericalWarper>();
            case CameraModel::PINHOLE:
                return cv::makePtr<cv::PlaneWarper>();
            case CameraModel::CYLINDRICAL:
                return cv::makePtr<cv::CylindricalWarper>();
        }
        return cv::makePtr<cv::SphericalWarper>();
    }

    cv::Ptr<cv::detail::SeamFinder> createSeamFinder() const {
        switch(seamFinder) {
            case SeamFinder::NONE:
                return cv::makePtr<cv::detail::NoSeamFinder>();
            case SeamFinder::VORONOI:
                return cv::makePtr<cv::detail::VoronoiSeamFinder>();
            case SeamFinder::DP_COLOR:
                return cv::makePtr<cv::detail::DpSeamFinder>(cv::detail::DpSeamFinder::COLOR);
            case SeamFinder::DP_COLOR_GRAD:
                return cv::makePtr<cv::detail::DpSeamFinder>(cv::detail::DpSeamFinder::COLOR_GRAD);
            case SeamFinder::GRAPHCUT_COLOR:
                return cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
            case SeamFinder::GRAPHCUT_COLOR_GRAD:
                return cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR_GRAD);
        }
        return cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
    }

    cv::Ptr<cv::detail::Blender> createBlender(const cv::Size& panoSizeHint) const {
        const float blendWidth = std::sqrt(static_cast<float>(panoSizeHint.area())) * blendStrength / 100.0f;
        if(blender == Blender::NONE || blendWidth < 1.0f) {
            return cv::detail::Blender::createDefault(cv::detail::Blender::NO, false);
        }
        if(blender == Blender::FEATHER) {
            auto feather = cv::makePtr<cv::detail::FeatherBlender>();
            feather->setSharpness(1.0f / blendWidth);
            return feather;
        }
        auto multiBand = cv::makePtr<cv::detail::MultiBandBlender>();
        multiBand->setNumBands(static_cast<int>(std::ceil(std::log(blendWidth) / std::log(2.0)) - 1.0));
        return multiBand;
    }
};

Stitching::Stitching() = default;

Stitching::~Stitching() = default;

void Stitching::buildInternal() {
    sync->out.link(inSync);
    sync->setRunOnHost(true);
}

std::shared_ptr<Stitching> Stitching::build(size_t numInputs) {
    DAI_CHECK_V(inputNames.empty(), "Stitching node was already built");
    DAI_CHECK_V(numInputs >= 2, "Stitching node needs at least two inputs, got {}", numInputs);

    for(size_t i = 0; i < numInputs; ++i) {
        auto name = fmt::format("input{}", i);
        auto& input = inputs[name];
        input.setBlocking(false);
        input.setMaxSize(4);
        inputNames.push_back(std::move(name));
    }

    return std::static_pointer_cast<Stitching>(shared_from_this());
}

std::shared_ptr<Stitching> Stitching::build(const std::vector<Node::Output*>& sources) {
    build(sources.size());

    for(size_t i = 0; i < sources.size(); ++i) {
        DAI_CHECK_V(sources[i] != nullptr, "Stitching source {} is null", i);
        sources[i]->link(inputs[inputNames[i]]);
    }

    return std::static_pointer_cast<Stitching>(shared_from_this());
}

size_t Stitching::getNumInputs() const {
    return inputNames.size();
}

void Stitching::setSyncThreshold(std::chrono::nanoseconds syncThreshold) {
    sync->setSyncThreshold(syncThreshold);
}

void Stitching::setMode(Mode mode) {
    DAI_CHECK_V(mode != Mode::PLANAR_PROJECTION, "Stitching mode PLANAR_PROJECTION is not implemented yet");
    impl->mode = mode;
    impl->invalidate();
}

Stitching::Mode Stitching::getMode() const {
    return impl->mode;
}

void Stitching::setCameraModel(CameraModel model) {
    impl->cameraModel = model;
    impl->invalidate();
}

Stitching::CameraModel Stitching::getCameraModel() const {
    return impl->cameraModel;
}

void Stitching::setContinuous(bool continuous) {
    impl->continuous = continuous;
    impl->invalidate();
}

bool Stitching::getContinuous() const {
    return impl->continuous;
}

void Stitching::setEstimationFrames(uint32_t frames) {
    DAI_CHECK_V(frames >= 1, "Stitching needs at least one estimation frame");
    impl->estimationFrames = frames;
    impl->invalidate();
}

uint32_t Stitching::getEstimationFrames() const {
    return impl->estimationFrames;
}

void Stitching::resetTransform() {
    impl->invalidate();
}

void Stitching::setRegistrationResolution(double megapixels) {
    impl->registrationResolution = megapixels;
    impl->invalidate();
}

double Stitching::getRegistrationResolution() const {
    return impl->registrationResolution;
}

void Stitching::setSeamEstimationResolution(double megapixels) {
    impl->seamEstimationResolution = megapixels;
    impl->invalidate();
}

double Stitching::getSeamEstimationResolution() const {
    return impl->seamEstimationResolution;
}

void Stitching::setCompositingResolution(double megapixels) {
    impl->compositingResolution = megapixels;
    impl->invalidate();
}

double Stitching::getCompositingResolution() const {
    return impl->compositingResolution;
}

void Stitching::setPanoConfidenceThreshold(double threshold) {
    impl->panoConfidenceThreshold = threshold;
    impl->invalidate();
}

double Stitching::getPanoConfidenceThreshold() const {
    return impl->panoConfidenceThreshold;
}

void Stitching::setWaveCorrection(WaveCorrection correction) {
    impl->waveCorrection = correction;
    impl->invalidate();
}

Stitching::WaveCorrection Stitching::getWaveCorrection() const {
    return impl->waveCorrection;
}

void Stitching::setInterpolation(Interpolation interpolation) {
    impl->interpolation = interpolation;
    impl->invalidate();
}

Stitching::Interpolation Stitching::getInterpolation() const {
    return impl->interpolation;
}

void Stitching::setFeaturesFinder(FeaturesFinder finder) {
    impl->featuresFinder = finder;
    impl->invalidate();
}

Stitching::FeaturesFinder Stitching::getFeaturesFinder() const {
    return impl->featuresFinder;
}

void Stitching::setFeaturesMatcher(FeaturesMatcher matcher, float matchConf) {
    impl->featuresMatcher = matcher;
    impl->matchConfidence = matchConf;
    impl->invalidate();
}

Stitching::FeaturesMatcher Stitching::getFeaturesMatcher() const {
    return impl->featuresMatcher;
}

void Stitching::setEstimator(Estimator estimator) {
    impl->estimator = estimator;
    impl->invalidate();
}

Stitching::Estimator Stitching::getEstimator() const {
    return impl->estimator;
}

void Stitching::setBundleAdjuster(BundleAdjuster adjuster) {
    impl->bundleAdjuster = adjuster;
    impl->invalidate();
}

Stitching::BundleAdjuster Stitching::getBundleAdjuster() const {
    return impl->bundleAdjuster;
}

void Stitching::setExposureCompensator(ExposureCompensator compensator) {
    impl->exposureCompensator = compensator;
    impl->invalidate();
}

Stitching::ExposureCompensator Stitching::getExposureCompensator() const {
    return impl->exposureCompensator;
}

void Stitching::setSeamFinder(SeamFinder finder) {
    impl->seamFinder = finder;
    impl->invalidate();
}

Stitching::SeamFinder Stitching::getSeamFinder() const {
    return impl->seamFinder;
}

void Stitching::setBlender(Blender blender, float strength) {
    impl->blender = blender;
    impl->blendStrength = strength;
    impl->invalidate();
}

Stitching::Blender Stitching::getBlender() const {
    return impl->blender;
}

void Stitching::run() {
    DAI_CHECK_V(!inputNames.empty(), "Stitching node was not built, call build() with the sources to stitch");
    auto& logger = pimpl->logger;

    while(mainLoop()) {
        std::shared_ptr<MessageGroup> group = nullptr;
        {
            auto blockEvent = this->inputBlockEvent();
            group = inSync.get<MessageGroup>();
        }
        if(group == nullptr) continue;

        std::vector<cv::Mat> images;
        std::shared_ptr<ImgFrame> first;
        images.reserve(inputNames.size());
        for(const auto& name : inputNames) {
            auto frame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(name));
            DAI_CHECK_V(frame != nullptr, "Stitching input {} did not receive an ImgFrame", name);
            if(first == nullptr) first = frame;

            auto image = frame->getCvFrame();
            if(image.channels() == 1) {
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            }
            images.push_back(std::move(image));
        }

        if(!impl->stitcher) {
            impl->create(cv::Size(images.front().cols * static_cast<int>(images.size()), images.front().rows));
        }

        cv::Mat pano;
        cv::Stitcher::Status status = cv::Stitcher::OK;
        try {
            if(impl->continuous) {
                status = impl->stitcher->stitch(images, pano);
            } else if(impl->transformFixed) {
                status = impl->stitcher->composePanorama(images, pano);
            } else {
                status = impl->stitcher->estimateTransform(images);
                if(status == cv::Stitcher::OK) {
                    // Images below the confidence threshold are left out of the panorama, and
                    // composePanorama only accepts the ones that made it into the component
                    const auto component = impl->stitcher->component();
                    std::vector<cv::Mat> contributing;
                    contributing.reserve(component.size());
                    for(auto index : component) {
                        contributing.push_back(images[index]);
                    }
                    if(component.size() != images.size() && logger) {
                        logger->warn("Stitching used {} of {} inputs, the rest did not match confidently", component.size(), images.size());
                    }

                    status = impl->stitcher->composePanorama(contributing, pano);

                    // Averaging is only meaningful while the component stays the same, otherwise
                    // cameras() is indexed differently from one group to the next
                    if(status == cv::Stitcher::OK && component.size() == images.size()) {
                        impl->samples.push_back(impl->stitcher->cameras());
                        if(impl->samples.size() >= impl->estimationFrames) {
                            impl->stitcher->setTransform(images, averageCameras(impl->samples));
                            impl->transformFixed = true;
                            impl->samples.clear();
                        }
                    }
                }
            }
        } catch(const cv::Exception& e) {
            if(logger) logger->warn("Stitching failed: {}", e.what());
            continue;
        }

        if(status != cv::Stitcher::OK || pano.empty()) {
            if(logger) logger->warn("Stitching failed: {}", statusToString(status));
            continue;
        }

        if(pano.depth() != CV_8U) {
            pano.convertTo(pano, CV_8U);
        }

        auto stitched = std::make_shared<ImgFrame>();
        stitched->setCvFrame(pano, ImgFrame::Type::BGR888i);
        stitched->setBufferMetadataFrom(first);
        out.send(stitched);
    }
}

}  // namespace node
}  // namespace dai
