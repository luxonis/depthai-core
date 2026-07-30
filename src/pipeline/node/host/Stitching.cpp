#include "depthai/pipeline/node/host/Stitching.hpp"

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching.hpp>

#include "PlanarStitcher.hpp"
#include "StitchingCompositing.hpp"
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

    std::optional<Plane> plane;
    std::optional<VirtualCamera> view;
    uint32_t maxViewWidth = 1920;
    uint32_t maxViewHeight = 1920;
    float maxRange = 1000.0f;
    float minIncidenceAngle = 5.0f;

    cv::Ptr<cv::Stitcher> stitcher;
    std::vector<std::vector<cv::detail::CameraParams>> samples;
    bool transformFixed = false;
    PlanarStitcher planar;

    void invalidate() {
        stitcher.release();
        samples.clear();
        transformFixed = false;
        planar.reset();
    }

    /// Push the planar projection settings into the implementation, dropping what was built from the old ones.
    void configurePlanar() {
        PlanarStitcher::Config config;
        config.plane = plane;
        config.view = view;
        config.maxViewWidth = maxViewWidth;
        config.maxViewHeight = maxViewHeight;
        config.maxRange = maxRange;
        config.minIncidenceAngle = minIncidenceAngle;
        config.seamEstimationResolution = seamEstimationResolution;
        config.interpolation = interpolation;
        config.exposureCompensator = exposureCompensator;
        config.seamFinder = seamFinder;
        config.blender = blender;
        config.blendStrength = blendStrength;
        planar.setConfig(config);
    }

    /// Metadata describing the rendered image: a pinhole camera placed in the reference frame of the inputs.
    ImgTransformation viewTransformation() const {
        const auto& resolved = planar.getResolvedView();
        Extrinsics extrinsics;
        extrinsics.setTransformationMatrix(resolved.pose, resolved.unit);
        extrinsics.setReferenceFrame(planar.getReferenceFrame());
        return {resolved.width, resolved.height, resolved.intrinsics, dai::CameraModel::Perspective, {}, extrinsics};
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
        stitcher->setInterpolationFlags(stitching::toInterpolationFlag(interpolation));
        stitcher->setFeaturesFinder(stitching::createFeaturesFinder(featuresFinder));
        stitcher->setFeaturesMatcher(stitching::createFeaturesMatcher(featuresMatcher, featuresFinder, matchConfidence));
        stitcher->setEstimator(stitching::createEstimator(estimator));
        stitcher->setBundleAdjuster(stitching::createBundleAdjuster(bundleAdjuster));
        stitcher->setWarper(stitching::createWarper(cameraModel));
        stitcher->setExposureCompensator(cv::detail::ExposureCompensator::createDefault(stitching::toExposureCompensatorType(exposureCompensator)));
        stitcher->setSeamFinder(stitching::createSeamFinder(seamFinder));
        stitcher->setBlender(stitching::createBlender(blender, blendStrength, panoSizeHint));
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
    impl->mode = mode;
    impl->invalidate();
}

Stitching::Mode Stitching::getMode() const {
    return impl->mode;
}

Stitching::VirtualCamera Stitching::VirtualCamera::lookAt(
    const Point3f& position, const Point3f& target, const Point3f& up, float hFovDegrees, uint32_t width, uint32_t height, LengthUnit unit) {
    DAI_CHECK_V(width > 0 && height > 0, "The view must not be empty, got {}x{} pixels", width, height);
    DAI_CHECK_V(hFovDegrees > 0.0f && hFovDegrees < 180.0f, "The horizontal field of view must be within (0, 180) degrees, got {}", hFovDegrees);

    const cv::Vec3d towardsTarget(target.x - position.x, target.y - position.y, target.z - position.z);
    DAI_CHECK_V(cv::norm(towardsTarget) > 1e-6, "The camera cannot look at its own position");
    const cv::Vec3d forward = cv::normalize(towardsTarget);
    cv::Vec3d right = forward.cross(cv::Vec3d(up.x, up.y, up.z));
    DAI_CHECK_V(cv::norm(right) > 1e-6, "The up direction must not be parallel to the optical axis");
    right = cv::normalize(right);
    const cv::Vec3d down = forward.cross(right);

    VirtualCamera camera;
    camera.width = width;
    camera.height = height;
    camera.unit = unit;
    const auto focal = static_cast<float>(0.5 * width / std::tan(0.5 * static_cast<double>(hFovDegrees) * CV_PI / 180.0));
    camera.intrinsics = {{{focal, 0.0f, 0.5f * (width - 1)}, {0.0f, focal, 0.5f * (height - 1)}, {0.0f, 0.0f, 1.0f}}};
    camera.pose = {{{static_cast<float>(right[0]), static_cast<float>(down[0]), static_cast<float>(forward[0]), position.x},
                    {static_cast<float>(right[1]), static_cast<float>(down[1]), static_cast<float>(forward[1]), position.y},
                    {static_cast<float>(right[2]), static_cast<float>(down[2]), static_cast<float>(forward[2]), position.z},
                    {0.0f, 0.0f, 0.0f, 1.0f}}};
    return camera;
}

void Stitching::setPlane(const Plane& plane) {
    DAI_CHECK_V(plane.normal.x != 0.0f || plane.normal.y != 0.0f || plane.normal.z != 0.0f, "The plane normal must not be a zero vector");
    impl->plane = plane;
    impl->invalidate();
}

void Stitching::setPlane(const Point3f& point, const Point3f& normal, LengthUnit unit) {
    setPlane(Plane{point, normal, unit});
}

std::optional<Stitching::Plane> Stitching::getPlane() const {
    return impl->plane;
}

void Stitching::setView(const VirtualCamera& view) {
    DAI_CHECK_V(view.width > 0 && view.height > 0, "The view must not be empty, got {}x{} pixels", view.width, view.height);
    DAI_CHECK_V(view.intrinsics[0][0] > 0.0f && view.intrinsics[1][1] > 0.0f, "The view needs a positive focal length");
    impl->view = view;
    impl->invalidate();
}

void Stitching::setViewAuto() {
    impl->view.reset();
    impl->invalidate();
}

std::optional<Stitching::VirtualCamera> Stitching::getView() const {
    return impl->view;
}

void Stitching::setMaxViewSize(uint32_t width, uint32_t height) {
    DAI_CHECK_V(width > 0 && height > 0, "The maximum view size must not be empty, got {}x{} pixels", width, height);
    impl->maxViewWidth = width;
    impl->maxViewHeight = height;
    impl->invalidate();
}

void Stitching::setMaxRange(float range, LengthUnit unit) {
    DAI_CHECK_V(range > 0.0f, "The maximum range must be positive, got {}", range);
    impl->maxRange = range * getDistanceUnitScale(LengthUnit::CENTIMETER, unit);
    impl->invalidate();
}

float Stitching::getMaxRange(LengthUnit unit) const {
    return impl->maxRange * getDistanceUnitScale(unit, LengthUnit::CENTIMETER);
}

void Stitching::setMinIncidenceAngle(float degrees) {
    DAI_CHECK_V(degrees >= 0.0f && degrees < 90.0f, "The minimum incidence angle must be within [0, 90) degrees, got {}", degrees);
    impl->minIncidenceAngle = degrees;
    impl->invalidate();
}

float Stitching::getMinIncidenceAngle() const {
    return impl->minIncidenceAngle;
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
        std::vector<ImgTransformation> transformations;
        std::shared_ptr<ImgFrame> first;
        images.reserve(inputNames.size());
        transformations.reserve(inputNames.size());
        for(const auto& name : inputNames) {
            auto frame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(name));
            DAI_CHECK_V(frame != nullptr, "Stitching input {} did not receive an ImgFrame", name);
            if(first == nullptr) first = frame;

            auto image = frame->getCvFrame();
            if(image.channels() == 1) {
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            }
            images.push_back(std::move(image));
            transformations.push_back(frame->getTransformation());
        }

        if(impl->mode == Mode::PLANAR_PROJECTION) {
            cv::Mat projected;
            try {
                if(!impl->planar.isPrepared()) {
                    impl->configurePlanar();
                    impl->planar.prepare(transformations);
                    const auto& resolved = impl->planar.getResolvedView();
                    if(logger) {
                        logger->info("Planar projection rendering {}x{} pixels of the plane in {}",
                                     resolved.width,
                                     resolved.height,
                                     toString(impl->planar.getReferenceFrame()));
                    }
                }
                projected = impl->planar.compose(images);
            } catch(const cv::Exception& e) {
                if(logger) logger->warn("Planar projection failed: {}", e.what());
                impl->planar.reset();
                continue;
            }

            auto stitched = std::make_shared<ImgFrame>();
            stitched->setCvFrame(projected, ImgFrame::Type::BGR888i);
            stitched->setBufferMetadataFrom(first);
            stitched->setTransformation(impl->viewTransformation());
            out.send(stitched);
            continue;
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
                    if(component.size() != images.size()) {
                        if(logger) {
                            logger->warn("Stitching used {} of {} inputs, the rest did not match confidently", component.size(), images.size());
                        }
                        // The sizes cv::Stitcher kept from matching are still indexed by the full
                        // input set, so composing only the component warps against the wrong ones.
                        // Re-registering the component on its own keeps the two consistent.
                        status = impl->stitcher->stitch(contributing, pano);
                    } else {
                        status = impl->stitcher->composePanorama(contributing, pano);
                    }

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
