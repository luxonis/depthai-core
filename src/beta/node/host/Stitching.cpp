#include "depthai/beta/node/Stitching.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching.hpp>
#include <optional>
#include <stdexcept>
#include <utility>

#include "beta/utilities/Stitching/FixedPanoramaCompositor.hpp"
#include "beta/utilities/Stitching/PlanarStitcher.hpp"
#include "beta/utilities/Stitching/StitchingCompositing.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace node {

using utilities::FixedPanoramaCompositor;
using utilities::PlanarStitcher;
namespace stitching = utilities::stitching;

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
    FixedPanoramaCompositor fixedPanorama;
    PlanarStitcher planar;

    void invalidate() {
        stitcher.release();
        scoringMatcher.release();
        bestCandidate.reset();
        candidatesEvaluated = 0;
        transformFixed = false;
        fixedPanorama.reset();
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
        const bool waveCorrection =
            properties.cameraModel == Stitching::CameraModel::SPHERICAL || properties.cameraModel == Stitching::CameraModel::CYLINDRICAL;
        stitcher->setWaveCorrection(waveCorrection);
        if(waveCorrection) {
            stitcher->setWaveCorrectKind(cv::detail::WAVE_CORRECT_HORIZ);
        }
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

    void prepareFixedPanorama(const std::vector<cv::Mat>& images, const StitchingProperties& properties) {
        FixedPanoramaCompositor::Config config;
        config.cameraModel = properties.cameraModel;
        config.seamFinder = properties.seamFinder;
        config.compositingResolution = stitching::COMPOSITING_RESOLUTION;
        config.seamEstimationResolution = stitching::SEAM_ESTIMATION_RESOLUTION;
        fixedPanorama.setConfig(config);
        fixedPanorama.prepare(images, stitcher->cameras(), stitcher->workScale());
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

void Stitching::initializeHostState() {
    impl = std::make_shared<Impl>();
}

void Stitching::run() {
    DAI_CHECK_V(impl != nullptr, "Stitching host state was not initialized");
    DAI_CHECK_V(!inputNames.empty(), "Stitching node was not built, call build() with the sources to stitch");
    auto& logger = pimpl->logger;
    bool modeLogged = false;

    while(mainLoop()) {
        std::shared_ptr<MessageGroup> group = nullptr;
        {
            auto blockEvent = this->inputBlockEvent();
            group = inSync.get<MessageGroup>();
        }
        if(group == nullptr) continue;

        bool invalidateState = false;
        StitchingProperties currentProperties;
        {
            std::lock_guard<std::mutex> lock(hostPropertiesMutex);
            invalidateState = hostStateInvalidated.exchange(false, std::memory_order_acq_rel);
            currentProperties = properties;
        }
        if(invalidateState) {
            impl->invalidate();
        }
        if(!modeLogged && logger && currentProperties.mode == Mode::PANORAMA) {
            if(currentProperties.continuous) {
                logger->info("Panorama stitching running in continuous estimation mode");
            } else {
                logger->info("Panorama stitching running in best-of-{} mode; waiting for {} valid candidates before emitting panoramas",
                             currentProperties.estimationFrames,
                             currentProperties.estimationFrames);
            }
            modeLogged = true;
        }

        std::vector<cv::Mat> images;
        std::vector<ImgTransformation> transformations;
        std::shared_ptr<ImgFrame> first;
        images.reserve(inputNames.size());
        transformations.reserve(inputNames.size());
        for(const auto& name : inputNames) {
            const auto entry = group->group.find(name);
            DAI_CHECK_V(entry != group->group.end(), "Stitching input {} is missing from the synchronized group", name);
            auto frame = std::dynamic_pointer_cast<ImgFrame>(entry->second);
            DAI_CHECK_V(frame != nullptr, "Stitching input {} did not receive an ImgFrame", name);
            if(first == nullptr) first = frame;

            auto image = frame->getCvFrame();
            if(image.channels() == 1) {
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            }
            images.push_back(std::move(image));
            transformations.push_back(frame->getTransformation());
        }

        if(currentProperties.mode == Mode::PLANAR_PROJECTION) {
            cv::Mat projected;
            if(!impl->planar.isPrepared()) {
                // Configuration failures are fatal; only failures caused by the current input group are recoverable.
                impl->configurePlanar(currentProperties);
            }
            try {
                if(!impl->planar.isPrepared()) {
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
            } catch(const std::runtime_error& e) {
                if(logger) logger->warn("Planar projection rejected the current input group: {}", e.what());
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
            impl->createPanoramaStitcher(cv::Size(images.front().cols * static_cast<int>(images.size()), images.front().rows), currentProperties);
        }

        cv::Mat pano;
        cv::Stitcher::Status status = cv::Stitcher::OK;
        const auto composePanorama = [&](const std::vector<cv::Mat>& contributing) -> std::optional<cv::Stitcher::Status> {
            cv::Size panoramaSize;
            if(!impl->panoramaFits(contributing, panoramaSize, currentProperties)) {
                if(logger) {
                    logger->debug("Stitching rejected a {}x{} panorama exceeding the configured {}x{} maximum",
                                  panoramaSize.width,
                                  panoramaSize.height,
                                  currentProperties.maxPanoramaWidth,
                                  currentProperties.maxPanoramaHeight);
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
            if(currentProperties.continuous) {
                impl->scoringMatcher->resetScore();
                const auto stitchingStatus = estimateAndComposePanorama(images);
                if(!stitchingStatus.has_value()) continue;
                status = *stitchingStatus;
            } else if(impl->transformFixed) {
                pano = impl->fixedPanorama.compose(images);
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
                    if(!impl->panoramaFits(images, candidateSize, currentProperties)) {
                        if(logger) {
                            logger->debug("Stitching rejected a {}x{} panorama exceeding the configured {}x{} maximum",
                                          candidateSize.width,
                                          candidateSize.height,
                                          currentProperties.maxPanoramaWidth,
                                          currentProperties.maxPanoramaHeight);
                        }
                        continue;
                    }

                    RegistrationCandidate candidate(impl->scoringMatcher->getScore(), impl->stitcher->cameras());
                    if(!impl->bestCandidate.has_value() || candidate.isBetterThan(*impl->bestCandidate)) {
                        impl->bestCandidate = std::move(candidate);
                    }
                    ++impl->candidatesEvaluated;
                    if(impl->candidatesEvaluated < currentProperties.estimationFrames) continue;

                    status = impl->stitcher->setTransform(images, impl->bestCandidate->cameras);
                    if(status == cv::Stitcher::OK) {
                        impl->prepareFixedPanorama(images, currentProperties);
                        pano = impl->fixedPanorama.compose(images);
                        impl->transformFixed = true;
                        if(logger) {
                            const auto size = impl->fixedPanorama.getCanvasSize();
                            logger->info("Panorama composition fixed at {}x{}; reusing warp maps, seams and exposure parameters", size.width, size.height);
                        }
                    }
                }
            }
        } catch(const cv::Exception& e) {
            if(logger) logger->warn("Stitching failed: {}", e.what());
            impl->transformFixed = false;
            impl->fixedPanorama.reset();
            continue;
        } catch(const std::exception& e) {
            if(logger) logger->warn("Stitching failed: {}", e.what());
            impl->transformFixed = false;
            impl->fixedPanorama.reset();
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
}  // namespace beta
}  // namespace dai
