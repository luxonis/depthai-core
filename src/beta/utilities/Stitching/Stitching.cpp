#include "depthai/beta/node/Stitching.hpp"

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && defined(DEPTHAI_HAVE_OPENCV_STITCHING)

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
    #include "depthai/utility/matrixOps.hpp"
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

bool isUndistorted(const ImgTransformation& transformation) {
    const auto coefficients = transformation.getDistortionCoefficients();
    return std::all_of(coefficients.begin(), coefficients.end(), [](float coefficient) {
        return std::isfinite(coefficient) && std::abs(coefficient) <= matrix::MATRIX_EQ_EPSILON;
    });
}

void alignCamerasToMeanYAxis(std::vector<cv::detail::CameraParams>& cameras) {
    constexpr double AXIS_EPSILON = 1e-6;
    DAI_CHECK_V(!cameras.empty(), "Calibrated panorama needs at least one camera to determine its mean Y axis");

    cv::Vec3d meanYAxis(0.0, 0.0, 0.0);
    for(const auto& camera : cameras) {
        cv::Mat rotation;
        camera.R.convertTo(rotation, CV_64F);
        meanYAxis += cv::Vec3d(rotation.at<double>(0, 1), rotation.at<double>(1, 1), rotation.at<double>(2, 1));
    }
    meanYAxis /= static_cast<double>(cameras.size());

    const double meanNorm = cv::norm(meanYAxis);
    DAI_CHECK_V(meanNorm > AXIS_EPSILON, "Calibrated panorama camera Y axes have no well-defined mean direction");
    meanYAxis /= meanNorm;

    const cv::Vec3d panoramaYAxis(0.0, 1.0, 0.0);
    const cv::Vec3d cross = meanYAxis.cross(panoramaYAxis);
    const double sineSquared = cross.dot(cross);
    const double cosine = meanYAxis.dot(panoramaYAxis);

    cv::Mat alignment = cv::Mat::eye(3, 3, CV_64F);
    if(sineSquared > AXIS_EPSILON * AXIS_EPSILON) {
        const cv::Mat crossMatrix = (cv::Mat_<double>(3, 3) << 0.0, -cross[2], cross[1], cross[2], 0.0, -cross[0], -cross[1], cross[0], 0.0);
        alignment += crossMatrix + crossMatrix * crossMatrix * ((1.0 - cosine) / sineSquared);
    } else if(cosine < 0.0) {
        alignment = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
    }

    for(auto& camera : cameras) {
        cv::Mat rotation;
        camera.R.convertTo(rotation, CV_64F);
        const cv::Mat aligned = alignment * rotation;
        aligned.convertTo(camera.R, CV_32F);
    }
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
    std::vector<ImgTransformation> fixedPanoramaTransformations;
    PlanarStitcher planar;

    void invalidate() {
        stitcher.release();
        scoringMatcher.release();
        bestCandidate.reset();
        candidatesEvaluated = 0;
        transformFixed = false;
        fixedPanorama.reset();
        fixedPanoramaTransformations.clear();
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

    void validateInputCalibrationOrigins(const std::vector<ImgTransformation>& transformations) const {
        DAI_CHECK_V(!transformations.empty(), "Calibrated panorama stitching needs at least one input transformation");
        DAI_CHECK_V(transformations.front().isValid(), "Calibrated panorama input 0 carries no valid image transformation");

        const auto firstExtrinsics = transformations.front().getExtrinsics();
        DAI_CHECK_V(!firstExtrinsics.toDeviceId.empty(), "Calibrated panorama stitching needs a destination device ID, but input 0 has none");
        DAI_CHECK_V(firstExtrinsics.toCameraSocket != CameraBoardSocket::AUTO,
                    "Calibrated panorama stitching needs a concrete destination camera socket, but input 0 uses AUTO");

        for(size_t i = 0; i < transformations.size(); ++i) {
            const auto& transformation = transformations[i];
            DAI_CHECK_V(transformation.isValid(), "Calibrated panorama input {} carries no valid image transformation", i);

            const auto extrinsics = transformation.getExtrinsics();
            DAI_CHECK_V(extrinsics.toDeviceId == firstExtrinsics.toDeviceId && extrinsics.toCameraSocket == firstExtrinsics.toCameraSocket,
                        "Calibrated panorama inputs must share one destination coordinate system, but input 0 uses {}/{} and input {} uses {}/{}",
                        firstExtrinsics.toDeviceId,
                        toString(firstExtrinsics.toCameraSocket),
                        i,
                        extrinsics.toDeviceId,
                        toString(extrinsics.toCameraSocket));
        }
    }

    std::vector<cv::detail::CameraParams> camerasFromInputCalibration(const std::vector<ImgTransformation>& transformations) const {
        validateInputCalibrationOrigins(transformations);

        std::vector<cv::detail::CameraParams> cameras;
        cameras.reserve(transformations.size());
        for(size_t i = 0; i < transformations.size(); ++i) {
            const auto& transformation = transformations[i];
            DAI_CHECK_V(isUndistorted(transformation), "Calibrated panorama input {} is distorted; request an undistorted camera output", i);
            const auto extrinsics = transformation.getExtrinsics();
            matrix::validateRotationMatrix3x3(extrinsics.rotationMatrix);

            const auto intrinsics = transformation.getIntrinsicMatrix();
            const float fx = intrinsics[0][0];
            const float fy = intrinsics[1][1];
            const float ppx = intrinsics[0][2];
            const float ppy = intrinsics[1][2];
            DAI_CHECK_V(std::isfinite(fx) && std::isfinite(fy) && std::isfinite(ppx) && std::isfinite(ppy) && fx > 0.0f && fy > 0.0f,
                        "Calibrated panorama input {} carries invalid intrinsics",
                        i);

            cv::detail::CameraParams camera;
            camera.focal = fx;
            camera.aspect = fy / fx;
            camera.ppx = ppx;
            camera.ppy = ppy;
            camera.R = cv::Mat(3, 3, CV_32F);
            for(int row = 0; row < 3; ++row) {
                for(int column = 0; column < 3; ++column) camera.R.at<float>(row, column) = extrinsics.rotationMatrix[row][column];
            }
            camera.t = cv::Mat::zeros(3, 1, CV_64F);
            cameras.push_back(std::move(camera));
        }
        return cameras;
    }

    void validatePreparedInputCalibration(const std::vector<ImgTransformation>& transformations) const {
        validateInputCalibrationOrigins(transformations);
        DAI_CHECK_V(transformations.size() == fixedPanoramaTransformations.size(),
                    "Calibrated panorama was prepared for {} inputs, but the current group has {}",
                    fixedPanoramaTransformations.size(),
                    transformations.size());
        for(size_t i = 0; i < transformations.size(); ++i) {
            const auto currentExtrinsics = transformations[i].getExtrinsics();
            const auto preparedExtrinsics = fixedPanoramaTransformations[i].getExtrinsics();
            DAI_CHECK_V(isUndistorted(transformations[i])
                            && matrix::mateq(transformations[i].getIntrinsicMatrix(), fixedPanoramaTransformations[i].getIntrinsicMatrix())
                            && matrix::mateq(currentExtrinsics.rotationMatrix, preparedExtrinsics.rotationMatrix)
                            && transformations[i].getSize() == fixedPanoramaTransformations[i].getSize(),
                        "Calibrated panorama input {} became distorted or changed its intrinsics, rotation, or size after preparation",
                        i);
        }
    }

    void prepareFixedPanorama(const std::vector<cv::Mat>& images,
                              const std::vector<cv::detail::CameraParams>& cameras,
                              double registrationScale,
                              FixedPanoramaCompositor::Composition composition,
                              const StitchingProperties& properties) {
        FixedPanoramaCompositor::Config config;
        config.cameraModel = properties.cameraModel;
        config.seamFinder = properties.seamFinder;
        config.compositingResolution = stitching::COMPOSITING_RESOLUTION;
        config.seamEstimationResolution = stitching::SEAM_ESTIMATION_RESOLUTION;
        config.composition = composition;
        fixedPanorama.setConfig(config);
        fixedPanorama.prepare(images, cameras, registrationScale);
    }

    void prepareEstimatedPanorama(const std::vector<cv::Mat>& images, const StitchingProperties& properties) {
        prepareFixedPanorama(images, stitcher->cameras(), stitcher->workScale(), FixedPanoramaCompositor::Composition::BLENDED, properties);
    }

    cv::Size panoramaSize(const std::vector<cv::Mat>& images,
                          const std::vector<cv::detail::CameraParams>& cameras,
                          double registrationScale,
                          const StitchingProperties& properties) const {
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
        const double composeWorkAspect = composeScale / registrationScale;
        auto warper = stitching::createWarper(properties.cameraModel)->create(static_cast<float>(warpedImageScale * composeWorkAspect));

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

    bool panoramaFits(const std::vector<cv::Mat>& images,
                      const std::vector<cv::detail::CameraParams>& cameras,
                      double registrationScale,
                      cv::Size& size,
                      const StitchingProperties& properties) const {
        if(properties.maxPanoramaWidth == std::numeric_limits<uint32_t>::max() && properties.maxPanoramaHeight == std::numeric_limits<uint32_t>::max())
            return true;
        size = panoramaSize(images, cameras, registrationScale, properties);
        return size.width > 0 && size.height > 0 && static_cast<uint32_t>(size.width) <= properties.maxPanoramaWidth
               && static_cast<uint32_t>(size.height) <= properties.maxPanoramaHeight;
    }

    bool estimatedPanoramaFits(const std::vector<cv::Mat>& images, cv::Size& size, const StitchingProperties& properties) const {
        return panoramaFits(images, stitcher->cameras(), stitcher->workScale(), size, properties);
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
            if(currentProperties.useInputCalibration) {
                logger->info("Panorama stitching using input calibration and coincident camera centers");
            } else if(currentProperties.continuous) {
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

        if(currentProperties.useInputCalibration) {
            cv::Mat pano;
            try {
                if(!impl->fixedPanorama.isPrepared()) {
                    auto cameras = impl->camerasFromInputCalibration(transformations);
                    if(currentProperties.cameraModel == CameraModel::CYLINDRICAL) {
                        alignCamerasToMeanYAxis(cameras);
                    }
                    cv::Size panoramaSize;
                    if(!impl->panoramaFits(images, cameras, 1.0, panoramaSize, currentProperties)) {
                        if(logger) {
                            logger->debug("Stitching rejected a {}x{} calibrated panorama exceeding the configured {}x{} maximum",
                                          panoramaSize.width,
                                          panoramaSize.height,
                                          currentProperties.maxPanoramaWidth,
                                          currentProperties.maxPanoramaHeight);
                        }
                        continue;
                    }
                    const auto composition = currentProperties.seamFinder == SeamFinder::NONE ? FixedPanoramaCompositor::Composition::DIRECT
                                                                                              : FixedPanoramaCompositor::Composition::BLENDED;
                    impl->prepareFixedPanorama(images, cameras, 1.0, composition, currentProperties);
                    impl->fixedPanoramaTransformations = transformations;
                    if(logger) {
                        const auto size = impl->fixedPanorama.getCanvasSize();
                        const auto* compositionName = composition == FixedPanoramaCompositor::Composition::DIRECT ? "direct" : "blended";
                        logger->info(
                            "Calibrated panorama composition fixed at {}x{}; reusing warp maps with {} composition", size.width, size.height, compositionName);
                    }
                } else {
                    impl->validatePreparedInputCalibration(transformations);
                }
                pano = impl->fixedPanorama.compose(images);
            } catch(const cv::Exception& e) {
                if(logger) logger->warn("Calibrated panorama stitching failed: {}", e.what());
                impl->fixedPanorama.reset();
                impl->fixedPanoramaTransformations.clear();
                continue;
            } catch(const std::exception& e) {
                if(logger) logger->warn("Calibrated panorama stitching rejected the current input group: {}", e.what());
                impl->fixedPanorama.reset();
                impl->fixedPanoramaTransformations.clear();
                continue;
            }

            auto stitched = std::make_shared<ImgFrame>();
            stitched->setCvFrame(pano, ImgFrame::Type::BGR888i);
            stitched->setBufferMetadataFrom(first);
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
            if(!impl->estimatedPanoramaFits(contributing, panoramaSize, currentProperties)) {
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
                    if(!impl->estimatedPanoramaFits(images, candidateSize, currentProperties)) {
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
                        impl->prepareEstimatedPanorama(images, currentProperties);
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

#endif
