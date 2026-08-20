#include "FixedPanoramaCompositor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "StitchingCompositing.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace node {

void FixedPanoramaCompositor::setConfig(const Config& config) {
    this->config = config;
    reset();
}

void FixedPanoramaCompositor::reset() {
    prepared = false;
    composeScale = 1.0;
    canvas = {};
    sources.clear();
    compensator.release();
    blender.release();
}

bool FixedPanoramaCompositor::isPrepared() const {
    return prepared;
}

cv::Size FixedPanoramaCompositor::getCanvasSize() const {
    DAI_CHECK_V(prepared, "The fixed panorama compositor was not prepared yet");
    return canvas.size();
}

void FixedPanoramaCompositor::prepare(const std::vector<cv::Mat>& images, const std::vector<cv::detail::CameraParams>& cameras, double registrationScale) {
    reset();
    DAI_CHECK_V(!images.empty(), "A fixed panorama needs at least one image");
    DAI_CHECK_V(images.size() == cameras.size(), "Got {} panorama images but {} registered cameras", images.size(), cameras.size());
    DAI_CHECK_V(registrationScale > 0.0, "The panorama registration scale must be positive, got {}", registrationScale);

    composeScale = 1.0;
    if(config.compositingResolution > 0.0) {
        composeScale = std::min(1.0, std::sqrt(config.compositingResolution * 1e6 / static_cast<double>(images.front().size().area())));
    }
    const double composeWorkAspect = composeScale / registrationScale;

    std::vector<double> focals;
    focals.reserve(cameras.size());
    for(const auto& camera : cameras) focals.push_back(camera.focal);
    std::sort(focals.begin(), focals.end());
    const auto middle = focals.size() / 2;
    const double warpedImageScale = focals.size() % 2 == 0 ? 0.5 * (focals[middle - 1] + focals[middle]) : focals[middle];
    auto warper = stitching::createWarper(config.cameraModel)->create(static_cast<float>(warpedImageScale * composeWorkAspect));

    sources.reserve(images.size());
    for(size_t i = 0; i < images.size(); ++i) {
        DAI_CHECK_V(!images[i].empty(), "Panorama input {} is empty", i);

        auto camera = cameras[i];
        camera.focal *= composeWorkAspect;
        camera.ppx *= composeWorkAspect;
        camera.ppy *= composeWorkAspect;
        cv::Mat intrinsics;
        camera.K().convertTo(intrinsics, CV_32F);

        Source source;
        source.inputSize = images[i].size();
        source.composeInputSize = cv::Size(cvRound(images[i].cols * composeScale), cvRound(images[i].rows * composeScale));

        cv::Mat mapX;
        cv::Mat mapY;
        source.roi = warper->buildMaps(source.composeInputSize, intrinsics, camera.R, mapX, mapY);
        cv::convertMaps(mapX, mapY, source.map1, source.map2, CV_16SC2);

        const cv::Mat inputMask(source.composeInputSize, CV_8U, cv::Scalar::all(255));
        cv::remap(inputMask, source.mask, source.map1, source.map2, cv::INTER_NEAREST, cv::BORDER_CONSTANT);
        source.seamMask = source.mask.clone();

        canvas = sources.empty() ? source.roi : canvas | source.roi;
        sources.push_back(std::move(source));
    }
    DAI_CHECK_V(canvas.width > 0 && canvas.height > 0, "The registered panorama has an empty canvas");

    const auto warped = warp(images);
    prepareCompositing(warped);
    blender = stitching::createBlender(canvas.size());
    prepared = true;
}

std::vector<cv::Mat> FixedPanoramaCompositor::warp(const std::vector<cv::Mat>& images) const {
    DAI_CHECK_V(images.size() == sources.size(), "Expected {} panorama images, got {}", sources.size(), images.size());

    std::vector<cv::Mat> warped(sources.size());
    for(size_t i = 0; i < sources.size(); ++i) {
        DAI_CHECK_V(images[i].size() == sources[i].inputSize,
                    "Panorama input {} changed its size from {}x{} to {}x{}",
                    i,
                    sources[i].inputSize.width,
                    sources[i].inputSize.height,
                    images[i].cols,
                    images[i].rows);
        cv::Mat composeInput;
        if(images[i].size() == sources[i].composeInputSize) {
            composeInput = images[i];
        } else {
            cv::resize(images[i], composeInput, sources[i].composeInputSize, 0, 0, cv::INTER_LINEAR_EXACT);
        }
        cv::remap(composeInput, warped[i], sources[i].map1, sources[i].map2, cv::INTER_LINEAR, cv::BORDER_REFLECT);
    }
    return warped;
}

void FixedPanoramaCompositor::prepareCompositing(const std::vector<cv::Mat>& warped) {
    compensator = cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN_BLOCKS);

    const double scale = config.seamEstimationResolution > 0.0
                             ? std::min(1.0, std::sqrt(config.seamEstimationResolution * 1e6 / static_cast<double>(canvas.size().area())))
                             : 1.0;
    std::vector<cv::Point> seamCorners;
    std::vector<cv::UMat> seamImages;
    std::vector<cv::UMat> seamMasks;
    seamCorners.reserve(sources.size());
    seamImages.reserve(sources.size());
    seamMasks.reserve(sources.size());
    for(size_t i = 0; i < sources.size(); ++i) {
        cv::Mat image;
        cv::Mat mask;
        cv::resize(warped[i], image, cv::Size(), scale, scale, cv::INTER_LINEAR_EXACT);
        cv::resize(sources[i].mask, mask, image.size(), 0, 0, cv::INTER_NEAREST);

        seamCorners.emplace_back(static_cast<int>(std::lround(sources[i].roi.x * scale)), static_cast<int>(std::lround(sources[i].roi.y * scale)));
        seamImages.push_back(image.getUMat(cv::ACCESS_READ));
        seamMasks.push_back(mask.getUMat(cv::ACCESS_RW));
    }

    compensator->feed(seamCorners, seamImages, seamMasks);
    for(auto& image : seamImages) {
        cv::UMat converted;
        image.convertTo(converted, CV_32F);
        image = std::move(converted);
    }
    stitching::createSeamFinder(config.seamFinder)->find(seamImages, seamCorners, seamMasks);
    for(size_t i = 0; i < sources.size(); ++i) {
        cv::Mat seamMask;
        cv::dilate(seamMasks[i].getMat(cv::ACCESS_READ), seamMask, cv::Mat());
        cv::resize(seamMask, seamMask, sources[i].mask.size(), 0, 0, cv::INTER_LINEAR);
        sources[i].seamMask = seamMask & sources[i].mask;
    }
}

cv::Mat FixedPanoramaCompositor::compose(const std::vector<cv::Mat>& images) {
    DAI_CHECK_V(prepared, "The fixed panorama compositor was not prepared yet");
    const auto warped = warp(images);

    blender->prepare(canvas);
    for(size_t i = 0; i < sources.size(); ++i) {
        cv::Mat compensated = warped[i];
        compensator->apply(static_cast<int>(i), sources[i].roi.tl(), compensated, sources[i].mask);

        cv::Mat converted;
        compensated.convertTo(converted, CV_16S);
        blender->feed(converted, sources[i].seamMask, sources[i].roi.tl());
    }

    cv::Mat result;
    cv::Mat resultMask;
    blender->blend(result, resultMask);
    result.convertTo(result, CV_8U);
    DAI_CHECK_V(result.size() == canvas.size(),
                "The fixed panorama blender returned {}x{}, expected the cached {}x{} canvas",
                result.cols,
                result.rows,
                canvas.width,
                canvas.height);
    return result;
}

}  // namespace node
}  // namespace beta
}  // namespace dai
