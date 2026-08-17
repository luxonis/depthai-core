#include "PlanarStitcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "StitchingCompositing.hpp"
#include "pipeline/utilities/Alignment/AlignmentUtilities.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

namespace {

constexpr double PARALLEL_EPS = 1e-9;
constexpr double DEPTH_EPS = 1e-6;
/// Samples taken along every edge of an image when its footprint on the plane is traced.
constexpr int FOOTPRINT_SAMPLES_PER_EDGE = 64;
constexpr uint32_t MIN_VIEW_SIZE = 16;

double toCentimeters(double value, LengthUnit unit) {
    return value * static_cast<double>(getDistanceUnitScale(LengthUnit::CENTIMETER, unit));
}

cv::Vec3d toVec(const Point3f& point) {
    return {point.x, point.y, point.z};
}

cv::Matx33d rotationOf(const std::array<std::array<float, 4>, 4>& transform) {
    return {transform[0][0],
            transform[0][1],
            transform[0][2],
            transform[1][0],
            transform[1][1],
            transform[1][2],
            transform[2][0],
            transform[2][1],
            transform[2][2]};
}

cv::Vec3d translationOf(const std::array<std::array<float, 4>, 4>& transform) {
    return {transform[0][3], transform[1][3], transform[2][3]};
}

cv::Matx33d toMatx33(const std::array<std::array<float, 3>, 3>& matrix) {
    return {matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[1][1], matrix[1][2], matrix[2][0], matrix[2][1], matrix[2][2]};
}

cv::Vec3d normalized(const cv::Vec3d& vector) {
    const double norm = cv::norm(vector);
    return norm > PARALLEL_EPS ? vector / norm : vector;
}

CameraBoardSocket getCommonOrigin(const std::vector<ImgTransformation>& transformations) {
    DAI_CHECK_V(!transformations.empty(), "Stitching in PLANAR_PROJECTION mode needs at least one input");
    for(size_t i = 0; i < transformations.size(); ++i) {
        DAI_CHECK_V(transformations[i].isValid(), "Input {} carries no valid transformation", i);
    }

    const auto origin = transformations.front().getExtrinsics().toCameraSocket;
    DAI_CHECK_V(origin != CameraBoardSocket::AUTO,
                "Stitching in PLANAR_PROJECTION mode needs calibrated inputs with a known origin camera socket, but input 0 has no origin");
    for(size_t i = 1; i < transformations.size(); ++i) {
        const auto inputOrigin = transformations[i].getExtrinsics().toCameraSocket;
        DAI_CHECK_V(inputOrigin == origin,
                    "Stitching in PLANAR_PROJECTION mode needs all inputs expressed relative to the same origin camera socket, but input 0 uses {} and "
                    "input {} uses {}",
                    toString(origin),
                    i,
                    toString(inputOrigin));
    }
    return origin;
}

}  // namespace

void PlanarStitcher::setConfig(const Config& config) {
    this->config = config;
    reset();
}

const PlanarStitcher::Config& PlanarStitcher::getConfig() const {
    return config;
}

void PlanarStitcher::reset() {
    prepared = false;
    compositingPrepared = false;
    numInputs = 0;
    sources.clear();
    compensator.release();
}

bool PlanarStitcher::isPrepared() const {
    return prepared;
}

const Stitching::VirtualCamera& PlanarStitcher::getResolvedView() const {
    DAI_CHECK_V(prepared, "The planar projection was not prepared yet");
    return view;
}

CameraBoardSocket PlanarStitcher::getOrigin() const {
    DAI_CHECK_V(prepared, "The planar projection was not prepared yet");
    return origin;
}

void PlanarStitcher::prepare(const std::vector<ImgTransformation>& transformations) {
    reset();

    DAI_CHECK_V(config.plane.has_value(), "Stitching in PLANAR_PROJECTION mode needs a plane, set it with setPlane()");
    numInputs = transformations.size();
    origin = getCommonOrigin(transformations);
    std::vector<SourcePose> poses;
    poses.reserve(transformations.size());
    for(size_t i = 0; i < transformations.size(); ++i) {
        const auto& transformation = transformations[i];
        const auto pose = transformation.getExtrinsics().getTransformationMatrix(false, LengthUnit::CENTIMETER);
        poses.push_back({rotationOf(pose), translationOf(pose)});
    }

    planePoint = toCentimeters(1.0, config.plane->unit) * toVec(config.plane->point);
    planeNormal = normalized(toVec(config.plane->normal));
    DAI_CHECK_V(cv::norm(planeNormal) > 0.5, "The plane normal must not be a zero vector");

    // Orient the normal towards the cameras, so that "in front of the plane" is well defined
    double signedDistance = 0.0;
    for(const auto& pose : poses) {
        signedDistance += planeNormal.dot(pose.center - planePoint);
    }
    DAI_CHECK_V(std::abs(signedDistance) > PARALLEL_EPS, "The cameras lie on the plane, so the plane cannot be projected onto");
    if(signedDistance < 0.0) {
        planeNormal = -planeNormal;
    }

    view = config.view.value_or(computeView(transformations, poses));
    DAI_CHECK_V(view.width >= MIN_VIEW_SIZE && view.height >= MIN_VIEW_SIZE, "The view is {}x{} pixels, which is too small", view.width, view.height);

    const auto viewPoseScale = toCentimeters(1.0, view.unit);
    const cv::Matx33d viewRotation = rotationOf(view.pose);
    const cv::Vec3d viewCenter = viewPoseScale * translationOf(view.pose);
    const cv::Matx33d viewIntrinsicsInv = toMatx33(view.intrinsics).inv();

    const auto width = static_cast<int>(view.width);
    const auto height = static_cast<int>(view.height);
    const double maxRangeSquared = static_cast<double>(config.maxRange) * static_cast<double>(config.maxRange);
    const double minIncidenceSine = std::sin(static_cast<double>(config.minIncidenceAngle) * CV_PI / 180.0);

    // Mutable copies, the mask lookup fills a cache of the transformation
    std::vector<ImgTransformation> masks(transformations.begin(), transformations.end());
    std::vector<cv::Mat> maps(transformations.size());
    std::vector<cv::Mat> validity(transformations.size());
    for(size_t i = 0; i < transformations.size(); ++i) {
        maps[i] = cv::Mat(height, width, CV_32FC2, cv::Scalar(-1, -1));
        validity[i] = cv::Mat::zeros(height, width, CV_8U);
    }

    for(int y = 0; y < height; ++y) {
        for(int x = 0; x < width; ++x) {
            // Ray of the virtual camera, in the reference frame
            const cv::Vec3d direction = viewRotation * (viewIntrinsicsInv * cv::Vec3d(x, y, 1.0));
            const double denominator = planeNormal.dot(direction);
            if(std::abs(denominator) < PARALLEL_EPS) continue;
            const double distance = planeNormal.dot(planePoint - viewCenter) / denominator;
            if(distance <= 0.0) continue;

            const cv::Vec3d point = viewCenter + distance * direction;

            for(size_t i = 0; i < transformations.size(); ++i) {
                const cv::Vec3d toPoint = point - poses[i].center;
                const double rangeSquared = toPoint.dot(toPoint);
                if(rangeSquared > maxRangeSquared) continue;
                // Rays grazing the plane stretch a couple of pixels over a large part of it
                if(std::abs(planeNormal.dot(toPoint)) < minIncidenceSine * std::sqrt(rangeSquared)) continue;

                const cv::Vec3d inCamera = poses[i].rotation.t() * toPoint;
                if(inCamera[2] <= DEPTH_EPS) continue;

                const auto projected =
                    transformations[i].project3DPoint({static_cast<float>(inCamera[0]), static_cast<float>(inCamera[1]), static_cast<float>(inCamera[2])});
                const auto [sourceWidth, sourceHeight] = transformations[i].getSize();
                if(projected.x < 0.0f || projected.y < 0.0f || projected.x > static_cast<float>(sourceWidth) - 1.0f
                   || projected.y > static_cast<float>(sourceHeight) - 1.0f) {
                    continue;
                }
                // Regions the input image does not cover, e.g. the padding of a letterboxed frame
                if(!masks[i].getDstMaskPt(static_cast<size_t>(projected.x), static_cast<size_t>(projected.y))) continue;

                maps[i].at<cv::Vec2f>(y, x) = cv::Vec2f(projected.x, projected.y);
                validity[i].at<uint8_t>(y, x) = 255;
            }
        }
    }

    for(size_t i = 0; i < transformations.size(); ++i) {
        const cv::Rect roi = cv::boundingRect(validity[i]);
        if(roi.empty()) continue;

        Source source;
        source.index = i;
        source.roi = roi;
        source.mask = validity[i](roi).clone();
        source.seamMask = source.mask;
        const auto [sourceWidth, sourceHeight] = transformations[i].getSize();
        source.imageSize = cv::Size(static_cast<int>(sourceWidth), static_cast<int>(sourceHeight));
        cv::convertMaps(maps[i](roi), cv::noArray(), source.map1, source.map2, CV_16SC2);
        sources.push_back(std::move(source));
    }

    DAI_CHECK_V(!sources.empty(),
                "None of the {} inputs sees the plane, check the plane definition, the extrinsics of the inputs and the range and incidence limits",
                transformations.size());

    prepared = true;
}

void PlanarStitcher::validateTransformations(const std::vector<ImgTransformation>& transformations) const {
    DAI_CHECK_V(prepared, "The planar projection was not prepared yet");
    DAI_CHECK_V(transformations.size() == numInputs,
                "The planar projection was prepared for {} inputs, but the current group has {}",
                numInputs,
                transformations.size());
    const auto currentOrigin = getCommonOrigin(transformations);
    DAI_CHECK_V(currentOrigin == origin,
                "The planar projection was prepared relative to origin {}, but the current group uses {}",
                toString(origin),
                toString(currentOrigin));
}

Stitching::VirtualCamera PlanarStitcher::computeView(const std::vector<ImgTransformation>& transformations, const std::vector<SourcePose>& poses) const {
    // In-plane axes of the rendered image: the reference frame looks "up" in the image, i.e. the optical axis of the
    // reference camera, projected onto the plane, points towards the top of the image
    cv::Vec3d heading = cv::Vec3d(0, 0, 1) - planeNormal.dot(cv::Vec3d(0, 0, 1)) * planeNormal;
    if(cv::norm(heading) < 1e-3) {
        heading = cv::Vec3d(1, 0, 0) - planeNormal.dot(cv::Vec3d(1, 0, 0)) * planeNormal;
    }
    heading = normalized(heading);
    const cv::Vec3d axisV = -heading;
    const cv::Vec3d axisU = heading.cross(planeNormal);

    double minU = std::numeric_limits<double>::max();
    double maxU = std::numeric_limits<double>::lowest();
    double minV = std::numeric_limits<double>::max();
    double maxV = std::numeric_limits<double>::lowest();
    uint32_t longestSide = 0;

    const double minIncidenceSine = std::sin(static_cast<double>(config.minIncidenceAngle) * CV_PI / 180.0);

    for(size_t i = 0; i < transformations.size(); ++i) {
        const auto [width, height] = transformations[i].getSize();
        longestSide = std::max(longestSide, static_cast<uint32_t>(std::max(width, height)));

        // Trace the border of the image, the footprint of a convex field of view is bounded by it
        std::vector<dai::Point2f> border;
        border.reserve(static_cast<size_t>(4) * FOOTPRINT_SAMPLES_PER_EDGE);
        for(int sample = 0; sample < FOOTPRINT_SAMPLES_PER_EDGE; ++sample) {
            const float alpha = static_cast<float>(sample) / FOOTPRINT_SAMPLES_PER_EDGE;
            const auto x = alpha * static_cast<float>(width - 1);
            const auto y = alpha * static_cast<float>(height - 1);
            border.emplace_back(x, 0.0f);
            border.emplace_back(static_cast<float>(width - 1), y);
            border.emplace_back(static_cast<float>(width - 1) - x, static_cast<float>(height - 1));
            border.emplace_back(0.0f, static_cast<float>(height - 1) - y);
        }

        for(const auto& pixel : border) {
            const auto ray = pixelToRay(transformations[i].invTransformPoint(pixel), transformations[i]);
            const cv::Vec3d direction = normalized(poses[i].rotation * cv::Vec3d(ray[0], ray[1], ray[2]));
            const double denominator = planeNormal.dot(direction);

            // A ray that misses the plane, hits it too far away or too flat is cut off at the maximum range and
            // projected onto the plane, which keeps the view bounded by what the cameras can actually paint
            cv::Vec3d point = poses[i].center + static_cast<double>(config.maxRange) * direction;
            if(std::abs(denominator) >= PARALLEL_EPS && std::abs(denominator) >= minIncidenceSine) {
                const double distance = planeNormal.dot(planePoint - poses[i].center) / denominator;
                if(distance > 0.0 && distance <= static_cast<double>(config.maxRange)) {
                    point = poses[i].center + distance * direction;
                }
            }
            const cv::Vec3d onPlane = point - planeNormal.dot(point - planePoint) * planeNormal - planePoint;

            minU = std::min(minU, onPlane.dot(axisU));
            maxU = std::max(maxU, onPlane.dot(axisU));
            minV = std::min(minV, onPlane.dot(axisV));
            maxV = std::max(maxV, onPlane.dot(axisV));
        }
    }

    const double extentU = std::max(maxU - minU, 1.0);
    const double extentV = std::max(maxV - minV, 1.0);
    const cv::Vec3d center = planePoint + 0.5 * (minU + maxU) * axisU + 0.5 * (minV + maxV) * axisV;

    // Keep the ground sampling distance of the inputs, within the configured bounds
    double width = longestSide;
    double height = width * extentV / extentU;
    if(height > width) {
        height = longestSide;
        width = height * extentU / extentV;
    }
    const double bound = std::min(config.maxViewWidth / width, config.maxViewHeight / height);
    if(bound < 1.0) {
        width *= bound;
        height *= bound;
    }

    Stitching::VirtualCamera resolved;
    resolved.width = std::max(MIN_VIEW_SIZE, static_cast<uint32_t>(std::lround(width)));
    resolved.height = std::max(MIN_VIEW_SIZE, static_cast<uint32_t>(std::lround(height)));
    resolved.unit = LengthUnit::CENTIMETER;

    // Any distance renders the same image as long as the focal length follows it, so the camera is placed the way a
    // real one framing the area would be
    const double distance = std::max(extentU, extentV);
    const double focal = resolved.width * distance / extentU;
    resolved.intrinsics = {
        {{static_cast<float>(focal), 0.0f, 0.5f * (resolved.width - 1)}, {0.0f, static_cast<float>(focal), 0.5f * (resolved.height - 1)}, {0.0f, 0.0f, 1.0f}}};

    // Looking straight at the plane, with the in-plane axes as the image axes
    const cv::Vec3d position = center + distance * planeNormal;
    const cv::Vec3d axisZ = -planeNormal;
    resolved.pose = {{{static_cast<float>(axisU[0]), static_cast<float>(axisV[0]), static_cast<float>(axisZ[0]), static_cast<float>(position[0])},
                      {static_cast<float>(axisU[1]), static_cast<float>(axisV[1]), static_cast<float>(axisZ[1]), static_cast<float>(position[1])},
                      {static_cast<float>(axisU[2]), static_cast<float>(axisV[2]), static_cast<float>(axisZ[2]), static_cast<float>(position[2])},
                      {0.0f, 0.0f, 0.0f, 1.0f}}};

    return resolved;
}

void PlanarStitcher::prepareCompositing(const std::vector<cv::Mat>& warped) {
    std::vector<cv::Point> corners;
    std::vector<cv::UMat> images;
    std::vector<cv::UMat> masks;
    corners.reserve(sources.size());
    images.reserve(sources.size());
    masks.reserve(sources.size());
    for(size_t i = 0; i < sources.size(); ++i) {
        corners.push_back(sources[i].roi.tl());
        images.push_back(warped[i].getUMat(cv::ACCESS_READ));
        masks.push_back(sources[i].mask.getUMat(cv::ACCESS_READ));
    }

    compensator = cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN_BLOCKS);
    compensator->feed(corners, images, masks);

    // Seam estimation is the expensive part, so it runs on downscaled images, the way OpenCV's stitcher does
    const double area = static_cast<double>(view.width) * view.height;
    const double scale = std::min(1.0, std::sqrt(stitching::SEAM_ESTIMATION_RESOLUTION * 1e6 / area));

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
        image.convertTo(image, CV_32F);

        seamCorners.emplace_back(static_cast<int>(std::lround(sources[i].roi.x * scale)), static_cast<int>(std::lround(sources[i].roi.y * scale)));
        seamImages.push_back(image.getUMat(cv::ACCESS_READ));
        seamMasks.push_back(mask.getUMat(cv::ACCESS_RW));
    }

    stitching::createSeamFinder(config.seamFinder)->find(seamImages, seamCorners, seamMasks);

    for(size_t i = 0; i < sources.size(); ++i) {
        cv::Mat seamMask;
        cv::dilate(seamMasks[i].getMat(cv::ACCESS_READ), seamMask, cv::Mat());
        cv::resize(seamMask, seamMask, sources[i].mask.size(), 0, 0, cv::INTER_LINEAR);
        sources[i].seamMask = seamMask & sources[i].mask;
    }

    compositingPrepared = true;
}

cv::Mat PlanarStitcher::compose(const std::vector<cv::Mat>& images) {
    DAI_CHECK_V(prepared, "The planar projection was not prepared yet");
    DAI_CHECK_V(images.size() == numInputs, "Expected {} images, got {}", numInputs, images.size());

    std::vector<cv::Mat> warped(sources.size());
    for(size_t i = 0; i < sources.size(); ++i) {
        const cv::Mat& image = images[sources[i].index];
        DAI_CHECK_V(image.size() == sources[i].imageSize,
                    "Input {} changed its size from {}x{} to {}x{}",
                    sources[i].index,
                    sources[i].imageSize.width,
                    sources[i].imageSize.height,
                    image.cols,
                    image.rows);
        cv::remap(image, warped[i], sources[i].map1, sources[i].map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }

    if(!compositingPrepared) {
        prepareCompositing(warped);
    }

    const cv::Size viewSize(static_cast<int>(view.width), static_cast<int>(view.height));
    auto blender = stitching::createBlender(viewSize);
    blender->prepare(cv::Rect(0, 0, viewSize.width, viewSize.height));

    for(size_t i = 0; i < sources.size(); ++i) {
        compensator->apply(static_cast<int>(i), sources[i].roi.tl(), warped[i], sources[i].mask);

        cv::Mat converted;
        warped[i].convertTo(converted, CV_16S);
        blender->feed(converted, sources[i].seamMask, sources[i].roi.tl());
    }

    cv::Mat result;
    cv::Mat resultMask;
    blender->blend(result, resultMask);
    result.convertTo(result, CV_8U);

    // The blender only guarantees the result covers the fed regions, the canvas is the view
    if(result.size() != viewSize) {
        cv::Mat canvas = cv::Mat::zeros(viewSize, result.type());
        const cv::Rect overlap = cv::Rect(0, 0, result.cols, result.rows) & cv::Rect(0, 0, viewSize.width, viewSize.height);
        result(overlap).copyTo(canvas(overlap));
        result = canvas;
    }

    return result;
}

}  // namespace node
}  // namespace dai
