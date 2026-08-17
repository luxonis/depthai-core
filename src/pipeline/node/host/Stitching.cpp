#include "depthai/pipeline/node/host/Stitching.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching.hpp>
#include <optional>
#include <utility>

#include "PlanarStitcher.hpp"
#include "StitchingCompositing.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/PimplImpl.hpp"

namespace dai {

StitchingProperties::VirtualCamera StitchingProperties::VirtualCamera::lookAt(
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

/** Decorates OpenCV's matcher and scores a candidate by confidence weighted by geometrically consistent inliers. */
class ScoringFeaturesMatcher : public cv::detail::FeaturesMatcher {
   public:
    explicit ScoringFeaturesMatcher(cv::Ptr<cv::detail::FeaturesMatcher> matcher)
        : cv::detail::FeaturesMatcher(matcher->isThreadSafe()), matcher(std::move(matcher)) {}

    void resetScore() {
        std::lock_guard<std::mutex> lock(scoreMutex);
        score = 0.0;
    }

    double getScore() const {
        std::lock_guard<std::mutex> lock(scoreMutex);
        return score;
    }

    void collectGarbage() override {
        matcher->collectGarbage();
    }

   protected:
    void match(const cv::detail::ImageFeatures& features1, const cv::detail::ImageFeatures& features2, cv::detail::MatchesInfo& matchesInfo) override {
        (*matcher)(features1, features2, matchesInfo);
        std::lock_guard<std::mutex> lock(scoreMutex);
        score += std::max(0.0, matchesInfo.confidence) * static_cast<double>(std::max(0, matchesInfo.num_inliers));
    }

   private:
    cv::Ptr<cv::detail::FeaturesMatcher> matcher;
    mutable std::mutex scoreMutex;
    double score = 0.0;
};

void appendMatrix(std::vector<double>& values, const cv::Mat& matrix) {
    cv::Mat flattened;
    matrix.reshape(1, 1).convertTo(flattened, CV_64F);
    const auto* begin = flattened.ptr<double>();
    values.insert(values.end(), begin, begin + flattened.total());
}

struct RegistrationCandidate {
    RegistrationCandidate(double score, std::vector<cv::detail::CameraParams> cameras) : score(score), cameras(std::move(cameras)) {
        for(const auto& camera : this->cameras) {
            geometryKey.insert(geometryKey.end(), {camera.focal, camera.aspect, camera.ppx, camera.ppy});
            appendMatrix(geometryKey, camera.R);
            appendMatrix(geometryKey, camera.t);
        }
    }

    bool isBetterThan(const RegistrationCandidate& other) const {
        if(score != other.score) return score > other.score;
        return std::lexicographical_compare(geometryKey.begin(), geometryKey.end(), other.geometryKey.begin(), other.geometryKey.end());
    }

    double score;
    std::vector<cv::detail::CameraParams> cameras;
    /// Deterministically selects one geometry when matching quality is exactly tied.
    std::vector<double> geometryKey;
};

}  // namespace

class Stitching::Impl {
   public:
    cv::Ptr<cv::Stitcher> stitcher;
    cv::Ptr<ScoringFeaturesMatcher> scoringMatcher;
    std::optional<RegistrationCandidate> bestCandidate;
    uint32_t candidatesEvaluated = 0;
    bool transformFixed = false;
    PlanarStitcher planar;

    void invalidate() {
        stitcher.release();
        scoringMatcher.release();
        bestCandidate.reset();
        candidatesEvaluated = 0;
        transformFixed = false;
        planar.reset();
    }

    /// Push the planar projection settings into the implementation, dropping what was built from the old ones.
    void configurePlanar(const StitchingProperties& properties) {
        PlanarStitcher::Config config;
        config.plane = properties.plane;
        config.view = properties.view;
        config.maxViewWidth = properties.maxViewWidth;
        config.maxViewHeight = properties.maxViewHeight;
        config.maxRange = properties.maxRange;
        config.minIncidenceAngle = properties.minIncidenceAngle;
        config.seamFinder = properties.seamFinder;
        planar.setConfig(config);
    }

    /// Metadata describing the rendered image: a pinhole camera placed in the common origin frame of the inputs.
    ImgTransformation viewTransformation() const {
        const auto& resolved = planar.getResolvedView();
        Extrinsics extrinsics;
        extrinsics.setTransformationMatrix(resolved.pose, resolved.unit);
        extrinsics.toCameraSocket = planar.getOrigin();
        return {resolved.width, resolved.height, resolved.intrinsics, dai::CameraModel::Perspective, {}, extrinsics};
    }

    /**
     * (Re)create the stitcher. panoSizeHint is used to scale the blending width the way
     * OpenCV's stitching_detailed sample does.
     */
    void createPanoramaStitcher(const cv::Size& panoSizeHint, const StitchingProperties& properties) {
        stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);

        stitcher->setRegistrationResol(stitching::REGISTRATION_RESOLUTION);
        stitcher->setSeamEstimationResol(stitching::SEAM_ESTIMATION_RESOLUTION);
        stitcher->setCompositingResol(stitching::COMPOSITING_RESOLUTION);
        stitcher->setPanoConfidenceThresh(properties.panoConfidenceThreshold);
        stitcher->setWaveCorrection(true);
        stitcher->setWaveCorrectKind(cv::detail::WAVE_CORRECT_HORIZ);
        stitcher->setInterpolationFlags(cv::INTER_LINEAR);
        stitcher->setFeaturesFinder(stitching::createFeaturesFinder());
        scoringMatcher = cv::makePtr<ScoringFeaturesMatcher>(stitching::createFeaturesMatcher());
        stitcher->setFeaturesMatcher(scoringMatcher);
        stitcher->setEstimator(stitching::createEstimator());
        stitcher->setBundleAdjuster(stitching::createBundleAdjuster());
        stitcher->setWarper(stitching::createWarper(properties.cameraModel));
        stitcher->setExposureCompensator(cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN_BLOCKS));
        stitcher->setSeamFinder(stitching::createSeamFinder(properties.seamFinder));
        stitcher->setBlender(stitching::createBlender(panoSizeHint));
    }

    cv::Size panoramaSize(const std::vector<cv::Mat>& images) const {
        const auto cameras = stitcher->cameras();
        DAI_CHECK_V(cameras.size() == images.size(), "Stitching camera and image counts differ");

        std::vector<double> focals;
        focals.reserve(cameras.size());
        for(const auto& camera : cameras) focals.push_back(camera.focal);
        std::sort(focals.begin(), focals.end());
        const auto middle = focals.size() / 2;
        const double warpedImageScale = focals.size() % 2 == 0 ? 0.5 * (focals[middle - 1] + focals[middle]) : focals[middle];

        double composeScale = 1.0;
        if(stitching::COMPOSITING_RESOLUTION > 0.0) {
            composeScale = std::min(1.0, std::sqrt(stitching::COMPOSITING_RESOLUTION * 1e6 / static_cast<double>(images.front().size().area())));
        }
        const double composeWorkAspect = composeScale / stitcher->workScale();
        auto warper = stitcher->warper()->create(static_cast<float>(warpedImageScale * composeWorkAspect));

        cv::Rect canvas;
        for(size_t i = 0; i < images.size(); ++i) {
            auto camera = cameras[i];
            camera.focal *= composeWorkAspect;
            camera.ppx *= composeWorkAspect;
            camera.ppy *= composeWorkAspect;
            cv::Mat intrinsics;
            camera.K().convertTo(intrinsics, CV_32F);
            const cv::Size imageSize(cvRound(images[i].cols * composeScale), cvRound(images[i].rows * composeScale));
            const auto roi = warper->warpRoi(imageSize, intrinsics, camera.R);
            canvas = i == 0 ? roi : canvas | roi;
        }
        return canvas.size();
    }

    bool panoramaFits(const std::vector<cv::Mat>& images, cv::Size& size, const StitchingProperties& properties) const {
        if(properties.maxPanoramaWidth == std::numeric_limits<uint32_t>::max() && properties.maxPanoramaHeight == std::numeric_limits<uint32_t>::max())
            return true;
        size = panoramaSize(images);
        return size.width > 0 && size.height > 0 && static_cast<uint32_t>(size.width) <= properties.maxPanoramaWidth
               && static_cast<uint32_t>(size.height) <= properties.maxPanoramaHeight;
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

void Stitching::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool Stitching::runOnHost() const {
    return runOnHostVar;
}

void Stitching::setMode(Mode mode) {
    properties.mode = mode;
    impl->invalidate();
}

Stitching::Mode Stitching::getMode() const {
    return properties.mode;
}

void Stitching::setPlane(const Plane& plane) {
    DAI_CHECK_V(plane.normal.x != 0.0f || plane.normal.y != 0.0f || plane.normal.z != 0.0f, "The plane normal must not be a zero vector");
    properties.plane = plane;
    impl->invalidate();
}

void Stitching::setPlane(const Point3f& point, const Point3f& normal, LengthUnit unit) {
    setPlane(Plane{point, normal, unit});
}

std::optional<Stitching::Plane> Stitching::getPlane() const {
    return properties.plane;
}

void Stitching::setView(const VirtualCamera& view) {
    DAI_CHECK_V(view.width > 0 && view.height > 0, "The view must not be empty, got {}x{} pixels", view.width, view.height);
    DAI_CHECK_V(view.intrinsics[0][0] > 0.0f && view.intrinsics[1][1] > 0.0f, "The view needs a positive focal length");
    properties.view = view;
    impl->invalidate();
}

void Stitching::setViewAuto() {
    properties.view.reset();
    impl->invalidate();
}

std::optional<Stitching::VirtualCamera> Stitching::getView() const {
    return properties.view;
}

void Stitching::setMaxViewSize(uint32_t width, uint32_t height) {
    DAI_CHECK_V(width > 0 && height > 0, "The maximum view size must not be empty, got {}x{} pixels", width, height);
    properties.maxViewWidth = width;
    properties.maxViewHeight = height;
    impl->invalidate();
}

void Stitching::setMaxRange(float range, LengthUnit unit) {
    DAI_CHECK_V(range > 0.0f, "The maximum range must be positive, got {}", range);
    properties.maxRange = range * getDistanceUnitScale(LengthUnit::CENTIMETER, unit);
    impl->invalidate();
}

float Stitching::getMaxRange(LengthUnit unit) const {
    return properties.maxRange * getDistanceUnitScale(unit, LengthUnit::CENTIMETER);
}

void Stitching::setMinIncidenceAngle(float degrees) {
    DAI_CHECK_V(degrees >= 0.0f && degrees < 90.0f, "The minimum incidence angle must be within [0, 90) degrees, got {}", degrees);
    properties.minIncidenceAngle = degrees;
    impl->invalidate();
}

float Stitching::getMinIncidenceAngle() const {
    return properties.minIncidenceAngle;
}

void Stitching::setCameraModel(CameraModel model) {
    properties.cameraModel = model;
    impl->invalidate();
}

Stitching::CameraModel Stitching::getCameraModel() const {
    return properties.cameraModel;
}

void Stitching::setContinuous(bool continuous) {
    properties.continuous = continuous;
    impl->invalidate();
}

bool Stitching::getContinuous() const {
    return properties.continuous;
}

void Stitching::setEstimationFrames(uint32_t frames) {
    DAI_CHECK_V(frames >= 1, "Stitching needs at least one estimation frame");
    properties.estimationFrames = frames;
    impl->invalidate();
}

uint32_t Stitching::getEstimationFrames() const {
    return properties.estimationFrames;
}

void Stitching::setMaxPanoramaSize(uint32_t width, uint32_t height) {
    DAI_CHECK_V(width > 0 && height > 0, "The maximum panorama size must not be empty, got {}x{} pixels", width, height);
    properties.maxPanoramaWidth = width;
    properties.maxPanoramaHeight = height;
}

void Stitching::resetTransform() {
    impl->invalidate();
}

void Stitching::setPanoConfidenceThreshold(double threshold) {
    properties.panoConfidenceThreshold = threshold;
    impl->invalidate();
}

double Stitching::getPanoConfidenceThreshold() const {
    return properties.panoConfidenceThreshold;
}

void Stitching::setSeamFinder(SeamFinder finder) {
    properties.seamFinder = finder;
    impl->invalidate();
}

Stitching::SeamFinder Stitching::getSeamFinder() const {
    return properties.seamFinder;
}

void Stitching::run() {
    DAI_CHECK_V(!inputNames.empty(), "Stitching node was not built, call build() with the sources to stitch");
    auto& logger = pimpl->logger;
    if(logger && properties.mode == Mode::PANORAMA) {
        if(properties.continuous) {
            logger->info("Panorama stitching running in continuous estimation mode");
        } else {
            logger->info("Panorama stitching running in best-of-{} mode; waiting for {} valid candidates before emitting panoramas",
                         properties.estimationFrames,
                         properties.estimationFrames);
        }
    }

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

        if(properties.mode == Mode::PLANAR_PROJECTION) {
            cv::Mat projected;
            try {
                if(!impl->planar.isPrepared()) {
                    impl->configurePlanar(properties);
                    impl->planar.prepare(transformations);
                    const auto& resolved = impl->planar.getResolvedView();
                    if(logger) {
                        logger->info("Planar projection rendering {}x{} pixels relative to origin {}",
                                     resolved.width,
                                     resolved.height,
                                     toString(impl->planar.getOrigin()));
                    }
                } else {
                    impl->planar.validateTransformations(transformations);
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
            impl->createPanoramaStitcher(cv::Size(images.front().cols * static_cast<int>(images.size()), images.front().rows), properties);
        }

        cv::Mat pano;
        cv::Stitcher::Status status = cv::Stitcher::OK;
        const auto composePanorama = [&](const std::vector<cv::Mat>& contributing) -> std::optional<cv::Stitcher::Status> {
            cv::Size panoramaSize;
            if(!impl->panoramaFits(contributing, panoramaSize, properties)) {
                if(logger) {
                    logger->debug("Stitching rejected a {}x{} panorama exceeding the configured {}x{} maximum",
                                  panoramaSize.width,
                                  panoramaSize.height,
                                  properties.maxPanoramaWidth,
                                  properties.maxPanoramaHeight);
                }
                return std::nullopt;
            }
            return impl->stitcher->composePanorama(contributing, pano);
        };
        const auto estimateAndComposePanorama = [&](std::vector<cv::Mat> contributing) -> std::optional<cv::Stitcher::Status> {
            while(true) {
                const auto estimationStatus = impl->stitcher->estimateTransform(contributing);
                if(estimationStatus != cv::Stitcher::OK) return estimationStatus;

                const auto component = impl->stitcher->component();
                if(component.size() == contributing.size()) return composePanorama(contributing);
                if(logger) logger->debug("Stitching used {} of {} inputs, the rest did not match confidently", component.size(), contributing.size());

                std::vector<cv::Mat> matched;
                matched.reserve(component.size());
                for(auto index : component) matched.push_back(contributing[index]);
                contributing = std::move(matched);
            }
        };
        try {
            if(properties.continuous) {
                impl->scoringMatcher->resetScore();
                const auto stitchingStatus = estimateAndComposePanorama(images);
                if(!stitchingStatus.has_value()) continue;
                status = *stitchingStatus;
            } else if(impl->transformFixed) {
                const auto compositionStatus = composePanorama(images);
                if(!compositionStatus.has_value()) continue;
                status = *compositionStatus;
            } else {
                impl->scoringMatcher->resetScore();
                status = impl->stitcher->estimateTransform(images);
                if(status == cv::Stitcher::OK) {
                    const auto component = impl->stitcher->component();
                    if(component.size() != images.size()) {
                        if(logger) {
                            logger->debug("Stitching used {} of {} inputs, the rest did not match confidently", component.size(), images.size());
                        }
                        continue;
                    }

                    cv::Size candidateSize;
                    if(!impl->panoramaFits(images, candidateSize, properties)) {
                        if(logger) {
                            logger->debug("Stitching rejected a {}x{} panorama exceeding the configured {}x{} maximum",
                                          candidateSize.width,
                                          candidateSize.height,
                                          properties.maxPanoramaWidth,
                                          properties.maxPanoramaHeight);
                        }
                        continue;
                    }

                    RegistrationCandidate candidate(impl->scoringMatcher->getScore(), impl->stitcher->cameras());
                    if(!impl->bestCandidate.has_value() || candidate.isBetterThan(*impl->bestCandidate)) {
                        impl->bestCandidate = std::move(candidate);
                    }
                    ++impl->candidatesEvaluated;
                    if(impl->candidatesEvaluated < properties.estimationFrames) continue;

                    status = impl->stitcher->setTransform(images, impl->bestCandidate->cameras);
                    if(status == cv::Stitcher::OK) {
                        const auto compositionStatus = composePanorama(images);
                        if(!compositionStatus.has_value()) continue;
                        status = *compositionStatus;
                        impl->transformFixed = status == cv::Stitcher::OK;
                    }
                }
            }
        } catch(const cv::Exception& e) {
            if(logger) logger->warn("Stitching failed: {}", e.what());
            continue;
        }

        if(status != cv::Stitcher::OK || pano.empty()) {
            if(logger) logger->debug("Stitching failed: {}", statusToString(status));
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

template class Pimpl<node::Stitching::Impl>;

}  // namespace dai
