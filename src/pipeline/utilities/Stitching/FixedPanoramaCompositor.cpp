#include "FixedPanoramaCompositor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "StitchingCompositing.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace utilities {

void FixedPanoramaCompositor::setConfig(const Config& config) {
    this->config = config;
    reset();
}

void FixedPanoramaCompositor::reset() {
    prepared = false;
    composeScale = 1.0;
    warperScale = 0.0;
    canvas = {};
    sources.clear();
    blender = FixedBlender{};
}

bool FixedPanoramaCompositor::isPrepared() const {
    return prepared;
}

cv::Size FixedPanoramaCompositor::getCanvasSize() const {
    return getCanvas().size();
}

cv::Rect FixedPanoramaCompositor::getCanvas() const {
    DAI_CHECK_V(prepared, "The fixed panorama compositor was not prepared yet");
    return canvas;
}

double FixedPanoramaCompositor::getWarperScale() const {
    DAI_CHECK_V(prepared, "The fixed panorama compositor was not prepared yet");
    return warperScale;
}

cv::Mat FixedPanoramaCompositor::getSourceMask(size_t sourceIndex) const {
    DAI_CHECK_V(prepared && sourceIndex < sources.size(), "The fixed panorama source {} is unavailable", sourceIndex);
    cv::Mat mask = cv::Mat::zeros(canvas.size(), CV_8U);
    const auto& source = sources[sourceIndex];
    source.seamMask.copyTo(mask(cv::Rect(source.roi.tl() - canvas.tl(), source.seamMask.size())));
    return mask;
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
    warperScale = warpedImageScale * composeWorkAspect;
    auto warper = stitching::createWarper(config.cameraModel)->create(static_cast<float>(warperScale));

    sources.reserve(images.size());
    std::vector<FixedBlender::Source> blendSources(images.size());
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
        source.roi.width = mapX.cols;
        source.roi.height = mapX.rows;

        auto& blendSource = blendSources[i];
        blendSource.tl = source.roi.tl();
        const cv::Mat inputMask(source.composeInputSize, CV_8U, cv::Scalar::all(255));
        cv::remap(inputMask, blendSource.mask, mapX, mapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT);
        cv::convertMaps(mapX, mapY, blendSource.map1, blendSource.map2, CV_16SC2);

        canvas = sources.empty() ? source.roi : canvas | source.roi;
        sources.push_back(std::move(source));
    }
    DAI_CHECK_V(canvas.width > 0 && canvas.height > 0, "The registered panorama has an empty canvas");

    if(config.composition == Composition::BLENDED) {
        const auto seamMasks = blender.prepareBlended(canvas,
                                                      std::move(blendSources),
                                                      composeInputs(images),
                                                      *stitching::createSeamFinder(config.seamFinder),
                                                      config.seamEstimationResolution,
                                                      cv::BORDER_REFLECT);
        for(size_t i = 0; i < sources.size(); ++i) sources[i].seamMask = seamMasks[i];
    } else {
        cv::Mat later = cv::Mat::zeros(canvas.size(), CV_8U);
        for(size_t i = sources.size(); i-- > 0;) {
            auto& blendSource = blendSources[i];
            const cv::Rect placed(blendSource.tl - canvas.tl(), blendSource.mask.size());
            blendSource.seamMask = blendSource.mask & ~later(placed);
            later(placed) |= blendSource.mask;
            sources[i].seamMask = blendSource.seamMask;
        }
        blender.prepare(canvas, blendSources, 0, cv::BORDER_REFLECT);
    }
    prepared = true;
}

std::vector<cv::Mat> FixedPanoramaCompositor::composeInputs(const std::vector<cv::Mat>& images) const {
    DAI_CHECK_V(images.size() == sources.size(), "Expected {} panorama images, got {}", sources.size(), images.size());

    std::vector<cv::Mat> inputs(sources.size());
    for(size_t i = 0; i < sources.size(); ++i) {
        DAI_CHECK_V(images[i].size() == sources[i].inputSize,
                    "Panorama input {} changed its size from {}x{} to {}x{}",
                    i,
                    sources[i].inputSize.width,
                    sources[i].inputSize.height,
                    images[i].cols,
                    images[i].rows);
        if(images[i].size() == sources[i].composeInputSize) {
            inputs[i] = images[i];
        } else {
            cv::resize(images[i], inputs[i], sources[i].composeInputSize, 0, 0, cv::INTER_LINEAR_EXACT);
        }
    }
    return inputs;
}

cv::Mat FixedPanoramaCompositor::compose(const std::vector<cv::Mat>& images) {
    DAI_CHECK_V(prepared, "The fixed panorama compositor was not prepared yet");
    return blender.blend(composeInputs(images));
}

}  // namespace utilities
}  // namespace dai
