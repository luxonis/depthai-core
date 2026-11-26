#include "FixedPanoramaCompositor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "StitchingCompositing.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace utilities {

namespace {

/// Direction of a point of the projection surface, `u` and `v` in units of its radius: the inverse of the OpenCV warpers' projections.
/// The pinhole plane stands for every model without a curved surface; createWarper() rejects the distortion-only ones.
cv::Vec3d surfaceDirection(CameraModel model, double u, double v) {
    switch(model) {
        case CameraModel::Equirectangular:
            return {std::sin(CV_PI - v) * std::sin(u), std::cos(CV_PI - v), std::sin(CV_PI - v) * std::cos(u)};
        case CameraModel::Cylindrical:
            return {std::sin(u), v, std::cos(u)};
        case CameraModel::Perspective:
        case CameraModel::Fisheye:
        case CameraModel::RadialDivision:
            break;
    }
    return {u, v, 1.0};
}

/// Point of the projection surface, in units of its radius, that `direction` lands on; empty behind a flat surface.
std::optional<cv::Point2d> surfacePoint(CameraModel model, const cv::Vec3d& direction) {
    switch(model) {
        case CameraModel::Equirectangular:
            return cv::Point2d(std::atan2(direction[0], direction[2]), CV_PI - std::acos(direction[1] / cv::norm(direction)));
        case CameraModel::Cylindrical:
            return cv::Point2d(std::atan2(direction[0], direction[2]), direction[1] / std::hypot(direction[0], direction[2]));
        case CameraModel::Perspective:
        case CameraModel::Fisheye:
        case CameraModel::RadialDivision:
            break;
    }
    if(direction[2] <= 0.0) return std::nullopt;
    return cv::Point2d(direction[0] / direction[2], direction[1] / direction[2]);
}

/// Distance along `ray` from `origin`, which is inside the wall, to the ground or to the wall, whichever comes first; infinite
/// for a ray straight up.
double groundDistance(const FixedPanoramaCompositor::Ground& ground, const cv::Vec3d& origin, const cv::Vec3d& ray) {
    const cv::Vec3d alongWall = ray - ray.dot(ground.up) * ground.up;
    const cv::Vec3d fromAxis = (origin - ground.eye) - (origin - ground.eye).dot(ground.up) * ground.up;
    const double a = alongWall.dot(alongWall);
    const double b = alongWall.dot(fromAxis);
    const double c = fromAxis.dot(fromAxis) - ground.wallDistance * ground.wallDistance;
    double distance = a > 1e-12 ? (-b + std::sqrt(std::max(0.0, b * b - a * c))) / a : std::numeric_limits<double>::infinity();
    const double down = -ray.dot(ground.up);
    if(down > 0.0) distance = std::min(distance, ground.up.dot(origin - ground.point) / down);
    return distance;
}

/// Where `ray` from `origin` hits the surface, relative to `from`; a ray that hits nothing gives its own direction, a point at infinity.
cv::Vec3d surfaceHit(const FixedPanoramaCompositor::Ground& ground, const cv::Vec3d& origin, const cv::Vec3d& ray, const cv::Vec3d& from) {
    const double distance = groundDistance(ground, origin, ray);
    return std::isinf(distance) ? ray : origin + distance * ray - from;
}

cv::Vec3d cameraCenter(const cv::detail::CameraParams& camera) {
    return {camera.t.at<double>(0), camera.t.at<double>(1), camera.t.at<double>(2)};
}

/**
 * Warp maps of one camera onto the ground, seen from the eye, over the region of the surface the camera covers.
 * @param scale Radius of the projection surface in pixels
 * @param camera Intrinsics at the compose resolution, rotation from the camera to the panorama frame and center
 */
cv::Rect buildGroundMaps(CameraModel model,
                         double scale,
                         const FixedPanoramaCompositor::Ground& ground,
                         const cv::detail::CameraParams& camera,
                         const cv::Size& imageSize,
                         cv::Mat& mapX,
                         cv::Mat& mapY) {
    const cv::Rect roi = FixedPanoramaCompositor::groundRoi(model, scale, ground, camera, imageSize);
    cv::Matx33d intrinsics;
    camera.K().convertTo(intrinsics, CV_64F);
    cv::Matx33d toCamera;
    camera.R.convertTo(toCamera, CV_64F);
    toCamera = toCamera.t();
    const cv::Vec3d center = cameraCenter(camera);

    mapX.create(roi.size(), CV_32F);
    mapY.create(roi.size(), CV_32F);
    cv::parallel_for_(cv::Range(0, roi.height), [&](const cv::Range& rows) {
        for(int row = rows.start; row < rows.end; ++row) {
            auto* xs = mapX.ptr<float>(row);
            auto* ys = mapY.ptr<float>(row);
            for(int column = 0; column < roi.width; ++column) {
                const cv::Vec3d direction = surfaceDirection(model, (roi.x + column) / scale, (roi.y + row) / scale);
                const cv::Vec3d inCamera = toCamera * surfaceHit(ground, ground.eye, direction, center);
                xs[column] = -1.0f;
                ys[column] = -1.0f;
                if(inCamera[2] <= 0.0) continue;
                xs[column] = static_cast<float>(intrinsics(0, 0) * inCamera[0] / inCamera[2] + intrinsics(0, 2));
                ys[column] = static_cast<float>(intrinsics(1, 1) * inCamera[1] / inCamera[2] + intrinsics(1, 2));
            }
        }
    });
    return roi;
}

}  // namespace

cv::Rect FixedPanoramaCompositor::groundRoi(
    CameraModel model, double scale, const Ground& ground, const cv::detail::CameraParams& camera, const cv::Size& imageSize) {
    cv::Matx33d intrinsics;
    camera.K().convertTo(intrinsics, CV_64F);
    cv::Matx33d rotation;
    camera.R.convertTo(rotation, CV_64F);
    const cv::Vec3d center = cameraCenter(camera);

    // Where a grid over the image lands on the surface
    constexpr int STEP = 8;
    const cv::Matx33d pixelToRay = rotation * intrinsics.inv();
    std::vector<cv::Point2f> landed;
    for(int y = 0; y < imageSize.height + STEP; y += STEP) {
        for(int x = 0; x < imageSize.width + STEP; x += STEP) {
            const cv::Vec3d ray = pixelToRay * cv::Vec3d(std::min(x, imageSize.width - 1), std::min(y, imageSize.height - 1), 1.0);
            const auto point = surfacePoint(model, surfaceHit(ground, center, ray, ground.eye));
            if(point) landed.emplace_back(static_cast<float>(point->x * scale), static_cast<float>(point->y * scale));
        }
    }
    DAI_CHECK_V(!landed.empty(), "A panorama input does not see the projection surface");
    const cv::Rect grid = cv::boundingRect(landed);
    return {grid.x - STEP, grid.y - STEP, grid.width + 2 * STEP, grid.height + 2 * STEP};
}

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

void FixedPanoramaCompositor::prepare(const std::vector<cv::Mat>& images,
                                      const std::vector<cv::detail::CameraParams>& cameras,
                                      double registrationScale,
                                      const std::optional<Ground>& ground) {
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
        if(ground) {
            source.roi = buildGroundMaps(config.cameraModel, warperScale, *ground, camera, source.composeInputSize, mapX, mapY);
        } else {
            source.roi = warper->buildMaps(source.composeInputSize, intrinsics, camera.R, mapX, mapY);
            source.roi.width = mapX.cols;
            source.roi.height = mapX.rows;
        }

        const cv::Mat inputMask(source.composeInputSize, CV_8U, cv::Scalar::all(255));
        cv::remap(inputMask, source.mask, mapX, mapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT);
        cv::convertMaps(mapX, mapY, source.map1, source.map2, CV_16SC2);

        canvas = sources.empty() ? source.roi : canvas | source.roi;
        sources.push_back(std::move(source));
    }
    DAI_CHECK_V(canvas.width > 0 && canvas.height > 0, "The registered panorama has an empty canvas");

    if(config.composition == Composition::BLENDED) {
        const auto inputs = composeInputs(images);
        std::vector<cv::Mat> warped(sources.size());
        for(size_t i = 0; i < sources.size(); ++i) {
            cv::remap(inputs[i], warped[i], sources[i].map1, sources[i].map2, cv::INTER_LINEAR, cv::BORDER_REFLECT);
        }
        prepareCompositing(warped);
    } else {
        prepareDirectCopy();
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

void FixedPanoramaCompositor::prepareDirectCopy() {
    cv::Mat later = cv::Mat::zeros(canvas.size(), CV_8U);
    for(auto source = sources.rbegin(); source != sources.rend(); ++source) {
        const cv::Rect placed(source->roi.tl() - canvas.tl(), source->roi.size());
        source->seamMask = source->mask & ~later(placed);
        later(placed) |= source->mask;
    }
    prepareBlender(std::vector<cv::Scalar>(sources.size(), cv::Scalar::all(1.0)), 0);
}

void FixedPanoramaCompositor::prepareBlender(const std::vector<cv::Scalar>& gains, int bandWidth) {
    std::vector<FixedBlender::Source> blendSources;
    blendSources.reserve(sources.size());
    for(size_t i = 0; i < sources.size(); ++i) {
        blendSources.push_back({sources[i].roi.tl(), sources[i].mask, sources[i].seamMask, gains[i], sources[i].map1, sources[i].map2});
    }
    blender.prepare(canvas, blendSources, bandWidth);
    for(auto& source : sources) {
        source.map1.release();
        source.map2.release();
    }
}

void FixedPanoramaCompositor::prepareCompositing(const std::vector<cv::Mat>& warped) {
    std::vector<cv::Point> corners;
    std::vector<cv::Mat> masks;
    for(const auto& source : sources) {
        corners.push_back(source.roi.tl());
        masks.push_back(source.mask);
    }
    const auto gains = estimateChannelGains(corners, warped, masks);
    const auto seamMasks =
        findSeamMasks(canvas.size(), corners, warped, masks, *stitching::createSeamFinder(config.seamFinder), config.seamEstimationResolution);
    for(size_t i = 0; i < sources.size(); ++i) sources[i].seamMask = seamMasks[i];
    prepareBlender(gains, static_cast<int>(std::lround(stitching::blendWidth(canvas.size()))));
}

cv::Mat FixedPanoramaCompositor::compose(const std::vector<cv::Mat>& images) {
    DAI_CHECK_V(prepared, "The fixed panorama compositor was not prepared yet");
    return blender.blend(composeInputs(images), cv::BORDER_REFLECT);
}

}  // namespace utilities
}  // namespace dai
