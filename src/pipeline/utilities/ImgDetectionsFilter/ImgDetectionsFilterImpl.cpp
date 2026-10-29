#include "ImgDetectionsFilterImpl.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "depthai/utility/ImageManipImpl.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace impl {
namespace {
// Sutherland-Hodgman intersection of two convex rectangles, also used for image bounds.
float intersectionArea(const RotatedRect& lhs, const RotatedRect& rhs) {
    const auto corners = lhs.getPoints();
    std::vector<Point2f> polygon(corners.begin(), corners.end());
    const auto clip = rhs.getPoints();
    for(std::size_t edge = 0; edge < clip.size() && !polygon.empty(); ++edge) {
        const auto a = clip[edge];
        const auto b = clip[(edge + 1) % clip.size()];
        const auto distance = [a, b](const Point2f& p) { return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x); };
        std::vector<Point2f> next;
        next.reserve(8);
        auto previous = polygon.back();
        float previousDistance = distance(previous);
        for(const auto& point : polygon) {
            const float currentDistance = distance(point);
            if((currentDistance >= 0) != (previousDistance >= 0)) {
                const float t = previousDistance / (previousDistance - currentDistance);
                next.emplace_back(previous.x + t * (point.x - previous.x), previous.y + t * (point.y - previous.y), false);
            }
            if(currentDistance >= 0) next.push_back(point);
            previous = point;
            previousDistance = currentDistance;
        }
        polygon = std::move(next);
    }
    double area = 0;
    for(std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i + 1) % polygon.size()];
        area += static_cast<double>(a.x) * static_cast<double>(b.y) - static_cast<double>(a.y) * static_cast<double>(b.x);
    }
    return static_cast<float>(std::abs(area) / 2);
}

// Axis-aligned bounds of mapped points as a RotatedRect at angle 0.
RotatedRect axisAlignedBounds(const std::vector<std::array<float, 2>>& points) {
    float minX = std::numeric_limits<float>::infinity(), minY = minX, maxX = -minX, maxY = -minX;
    for(const auto& point : points) {
        minX = std::min(minX, point[0]);
        maxX = std::max(maxX, point[0]);
        minY = std::min(minY, point[1]);
        maxY = std::max(maxY, point[1]);
    }
    return RotatedRect(Point2f((minX + maxX) / 2, (minY + maxY) / 2, false), Size2f(maxX - minX, maxY - minY, false), 0);
}

struct MaskMember {
    std::size_t key, index;
    std::array<float, 2> xBounds{-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
};

struct MaskTarget {
    std::uint8_t index;
    std::array<float, 2> xBounds;
};

struct Candidate {
    ImgDetection detection;
    RotatedRect box;
    std::size_t key;
    std::vector<MaskMember> maskMembers;
    double visibility = 1.0;
};

class DetectionRound {
    const std::vector<std::shared_ptr<ImgDetections>>& messages;
    const ImgDetectionsFilterConfig& config;
    const std::optional<ImgTransformation>& reference;
    const std::vector<std::shared_ptr<ImgFrame>>& sourceMasks;
    std::vector<MaskRemapCache>& maskRemaps;
    const bool geometry;
    std::shared_ptr<ImgDetections> output = std::make_shared<ImgDetections>();
    std::pair<std::size_t, std::size_t> size{0, 0};
    std::vector<Candidate> candidates;
    std::vector<std::size_t> ranked, kept;
    std::vector<bool> removed;

    static RotatedRect standardize(RotatedRect box) {
        box.angle = std::remainder(box.angle, 180.0f);
        if(box.angle <= -45) {
            box.angle += 90;
            std::swap(box.size.width, box.size.height);
        }
        if(box.angle > 45) {
            box.angle -= 90;
            std::swap(box.size.width, box.size.height);
        }
        return box;
    }
    void writeBox(Candidate& candidate) const {
        candidate.detection.boundingBox = candidate.box.normalize(size.first, size.second);
        const auto outer = candidate.box.getOuterRect();
        candidate.detection.xmin = outer[0] / size.first;
        candidate.detection.ymin = outer[1] / size.second;
        candidate.detection.xmax = outer[2] / size.first;
        candidate.detection.ymax = outer[3] / size.second;
    }
    void prepare() {
        if(messages.empty()) throw std::invalid_argument("ImgDetectionsFilter requires at least one linked input");
        auto newest = messages.front();
        for(const auto& message : messages) {
            if(message->getTimestamp() > newest->getTimestamp()) newest = message;
            if(!reference && !geometry) continue;
            const auto source = message->getTransformation();
            if(!source || !source->isValid()) throw std::runtime_error("ImgDetectionsFilter requires a valid input ImgTransformation");
            if(!reference || source->isEqualTransformation(*reference)) continue;
            const auto model = source->getDistortionModel();
            if(model == CameraModel::Equirectangular || model == CameraModel::Cylindrical)
                throw std::invalid_argument("ImgDetectionsFilter v1 cannot remap a panorama input to a different transformation");
            // Check compatibility even for empty messages; AUTO uses the existing identity-rotation remap rule.
            const auto fromExtrinsics = source->getExtrinsics(), toExtrinsics = reference->getExtrinsics();
            if(!fromExtrinsics.hasCompatibleCoordinateSystem(toExtrinsics)
               || (fromExtrinsics.toCameraSocket != CameraBoardSocket::AUTO && toExtrinsics.toCameraSocket != CameraBoardSocket::AUTO))
                source->getRotationMatrixTo(*reference);
        }
        output->setBufferMetadataFrom(newest.get());
        output->transformation = reference ? reference : messages.front()->getTransformation();
        if(output->transformation) size = output->transformation->getSize();
        if(!sourceMasks.empty() && sourceMasks.size() != messages.size()) throw std::invalid_argument("ImgDetectionsFilter source mask count mismatch");
        for(const auto& mask : sourceMasks) {
            if(!mask) continue;
            if(mask->getType() != ImgFrame::Type::GRAY8 || mask->getWidth() != size.first || mask->getHeight() != size.second || !output->transformation
               || !mask->getTransformation().isEqualTransformation(*output->transformation))
                throw std::invalid_argument("ImgDetectionsFilter source masks must be GRAY8 frames matching the output transformation and dimensions");
            const auto dataSize = mask->getData().size();
            const std::size_t offset = mask->fb.p1Offset, stride = mask->getStride();
            if(stride < size.first || stride == 0 || offset > dataSize || size.second > (dataSize - offset) / stride)
                throw std::invalid_argument("ImgDetectionsFilter source mask payload is too small");
        }
    }
    void remap(Candidate candidate, const ImgTransformation& source, bool pixels) {
        auto& detection = candidate.detection;
        const bool identity = source.isEqualTransformation(*reference);
        std::vector<RotatedRect> boxes;
        bool seamFragment = false;
        if(!identity) {
            const auto model = reference->getDistortionModel();
            const bool panorama = model == CameraModel::Cylindrical || model == CameraModel::Equirectangular;
            // Panorama projections curve straight box edges; corners alone can underestimate the bounds and shift their center.
            const std::size_t edgeSamples = panorama ? 16 : 1;
            const auto corners = candidate.box.getPoints();
            std::vector<std::array<float, 2>> points;
            points.reserve(corners.size() * edgeSamples);
            for(std::size_t edge = 0; edge < corners.size(); ++edge) {
                const auto a = corners[edge], b = corners[(edge + 1) % corners.size()];
                for(std::size_t sample = 0; sample < edgeSamples; ++sample) {
                    const float t = static_cast<float>(sample) / edgeSamples;
                    const auto mapped = source.remapPointTo(*reference, Point2f(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), false));
                    if(!std::isfinite(mapped.x) || !std::isfinite(mapped.y)) return;
                    points.push_back({mapped.x, mapped.y});
                }
            }
            // The bent edges make the minimum-area rectangle tilt by several degrees even for a level camera, most near the image
            // border, and a normalized RotatedRect stays a rectangle only at angle 0. A panorama keeps the mean camera Y axis
            // vertical, so its boxes are the axis-aligned bounds of the sampled edges; perspective references keep the box
            // orientation, which an in-plane camera roll maps exactly.
            const auto intrinsics = reference->getIntrinsicMatrix();
            if(panorama && intrinsics[0][1] == 0 && intrinsics[1][0] == 0 && intrinsics[2][0] == 0 && intrinsics[2][1] == 0) {
                // Longitude wraps at +/- pi. Unwrap before taking bounds, then split at that join, which need not be the canvas edges.
                const float period = 2 * std::acos(-1.0f) * std::abs(intrinsics[0][0] / intrinsics[2][2]);
                if(!std::isfinite(period) || period <= 0) return;
                const float center = intrinsics[0][2] / intrinsics[2][2];
                const float left = center - period / 2, right = center + period / 2;
                const float anchor = points.front()[0];
                float previous = anchor, unwrapped = anchor;
                for(auto& point : points) {
                    const float x = point[0];
                    unwrapped += std::remainder(x - previous, period);
                    previous = x;
                    point[0] = unwrapped;
                }
                const float winding = unwrapped - anchor + std::remainder(anchor - previous, period);
                candidate.box = axisAlignedBounds(points);
                if(std::abs(winding) > period / 2) {
                    // A footprint enclosing a pole spans all longitudes and reaches the pole, which no edge sample does. Source rays
                    // lie in one hemisphere, so the middle of the sampled latitudes is on the side of the enclosed pole.
                    const float equator = intrinsics[1][2] / intrinsics[2][2];
                    const bool above = candidate.box.center.y < equator;
                    // Cylindrical poles lie at infinity, so the box extends to the canvas edge instead.
                    float pole = above ? 0.0f : static_cast<float>(size.second);
                    if(model == CameraModel::Equirectangular)
                        pole = equator + (above ? -1.0f : 1.0f) * std::abs(intrinsics[1][1] / intrinsics[2][2]) * std::acos(-1.0f) / 2;
                    const auto outer = candidate.box.getOuterRect();
                    const float minY = std::min(outer[1], pole), maxY = std::max(outer[3], pole);
                    candidate.box = RotatedRect(Point2f(center, (minY + maxY) / 2, false), Size2f(period, maxY - minY, false), 0);
                } else {
                    const auto outer = candidate.box.getOuterRect();
                    if(outer[0] < left || outer[2] > right) {
                        seamFragment = true;
                        for(const float shift : {-period, 0.0f, period}) {
                            const float minX = std::max(left, outer[0] + shift), maxX = std::min(right, outer[2] + shift);
                            if(maxX > minX)
                                boxes.emplace_back(
                                    Point2f((minX + maxX) / 2, candidate.box.center.y, false), Size2f(maxX - minX, candidate.box.size.height, false), 0);
                        }
                    }
                }
            } else {
                candidate.box = panorama ? axisAlignedBounds(points) : standardize(getOuterRotatedRect(points));
            }
            candidate.box.center.hasNormalized = candidate.box.size.hasNormalized = true;
            candidate.box.center.normalized = candidate.box.size.normalized = false;
        }
        if(detection.keypoints) {
            for(auto& point : detection.keypoints->keypoints) {
                const auto mapped = source.remapPointTo(*reference, Point2f(point.imageCoordinates.x, point.imageCoordinates.y, true));
                if(!std::isfinite(mapped.x) || !std::isfinite(mapped.y))
                    point.confidence = 0;
                else {
                    point.imageCoordinates.x = mapped.x;
                    point.imageCoordinates.y = mapped.y;
                }
            }
        }
        if(boxes.empty()) boxes.push_back(candidate.box);
        for(const auto& box : boxes) {
            if(intersectionArea(box, RotatedRect(Rect(0, 0, size.first, size.second, false))) <= 0) continue;
            auto fragment = candidate;
            fragment.box = box;
            // Preserve exact normalized coordinates when the input already has standard form.
            if(!identity || pixels || !detection.boundingBox || box.angle != detection.boundingBox->angle) writeBox(fragment);
            fragment.detection.boundingBox->center.hasNormalized = fragment.detection.boundingBox->size.hasNormalized = true;
            fragment.detection.boundingBox->center.normalized = fragment.detection.boundingBox->size.normalized = true;
            if(seamFragment) {
                // Keep each member's original partition when its output box is later averaged.
                fragment.maskMembers.front().xBounds = {fragment.detection.xmin, fragment.detection.xmax};
                if(fragment.detection.keypoints) {
                    for(auto& point : fragment.detection.keypoints->keypoints)
                        if(point.imageCoordinates.x < fragment.detection.xmin || point.imageCoordinates.x > fragment.detection.xmax) point.confidence = 0;
                }
            }
            candidates.push_back(std::move(fragment));
        }
    }
    void collect() {
        for(std::size_t key = 0; key < messages.size(); ++key) {
            const auto& message = messages[key];
            for(std::size_t index = 0; index < message->detections.size(); ++index) {
                Candidate candidate{message->detections[index], {}, key, {{key, index}}};
                const auto& detection = candidate.detection;
                const auto contains = [&detection](const auto& labels) { return std::find(labels.begin(), labels.end(), detection.label) != labels.end(); };
                if(config.labelsToKeep && !contains(*config.labelsToKeep)) continue;
                if(config.labelsToReject && contains(*config.labelsToReject)) continue;
                if((config.minConfidence != 0 && detection.confidence < config.minConfidence)
                   || (config.maxConfidence != 1 && detection.confidence > config.maxConfidence))
                    continue;
                if(reference || geometry) {
                    auto box = detection.getBoundingBox();
                    const bool pixels = detection.boundingBox && box.center.hasNormalized && !box.center.normalized;
                    // Unmarked ImgDetections coordinates are normalized, including values outside [0,1].
                    box.center = {box.center.x, box.center.y, !pixels};
                    box.size = {box.size.width, box.size.height, !pixels};
                    const auto sourceSize = message->transformation->getSize();
                    candidate.box = standardize(box.denormalize(sourceSize.first, sourceSize.second));
                    if(reference) {
                        remap(std::move(candidate), *message->transformation, pixels);
                        continue;
                    }
                }
                candidates.push_back(std::move(candidate));
            }
        }
        for(auto& candidate : candidates) {
            if(sourceMasks.empty() || !sourceMasks[candidate.key]) continue;
            candidate.visibility = 0;
            const auto& box = candidate.box;
            const auto outer = box.getOuterRect();
            if(box.size.width <= 0 || box.size.height <= 0 || !std::all_of(outer.begin(), outer.end(), [](float value) { return std::isfinite(value); }))
                continue;
            const auto& mask = sourceMasks[candidate.key];
            const auto data = mask->getData();
            const auto xBegin = static_cast<std::size_t>(std::clamp(std::floor(static_cast<double>(outer[0])), 0.0, static_cast<double>(size.first)));
            const auto yBegin = static_cast<std::size_t>(std::clamp(std::floor(static_cast<double>(outer[1])), 0.0, static_cast<double>(size.second)));
            const auto xEnd = static_cast<std::size_t>(std::clamp(std::ceil(static_cast<double>(outer[2])), 0.0, static_cast<double>(size.first)));
            const auto yEnd = static_cast<std::size_t>(std::clamp(std::ceil(static_cast<double>(outer[3])), 0.0, static_cast<double>(size.second)));
            const double radians = static_cast<double>(box.angle) * std::acos(-1.0) / 180.0;
            const double cosine = std::cos(radians), sine = std::sin(radians);
            const std::size_t stride = mask->getStride();
            std::size_t visiblePixels = 0;
            // A seam can hide the center while leaving much of the object visible. Check the whole rotated footprint.
            for(auto y = yBegin; y < yEnd; ++y) {
                const double dy = static_cast<double>(y) + 0.5 - static_cast<double>(box.center.y);
                const auto row = mask->fb.p1Offset + y * stride;
                for(auto x = xBegin; x < xEnd; ++x) {
                    const double dx = static_cast<double>(x) + 0.5 - static_cast<double>(box.center.x);
                    visiblePixels += data[row + x] != 0 && std::abs(dx * cosine + dy * sine) <= static_cast<double>(box.size.width) / 2
                                     && std::abs(dy * cosine - dx * sine) <= static_cast<double>(box.size.height) / 2;
                }
            }
            candidate.visibility = std::min(1.0, visiblePixels / (static_cast<double>(box.size.width) * static_cast<double>(box.size.height)));
        }
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const auto& candidate) { return candidate.visibility == 0; }),
                         candidates.end());
        ranked.resize(candidates.size());
        std::iota(ranked.begin(), ranked.end(), 0);
        std::stable_sort(
            ranked.begin(), ranked.end(), [this](auto a, auto b) { return candidates[a].detection.confidence > candidates[b].detection.confidence; });
        removed.assign(candidates.size(), false);
    }
    void average(const std::vector<std::size_t>& members) {
        auto& leader = candidates[members.front()];
        const bool equalWeights = std::all_of(members.begin(), members.end(), [this](auto i) { return candidates[i].detection.confidence == 0; });
        double totalWeight = 0;
        for(const auto index : members)
            totalWeight += static_cast<double>(equalWeights ? 1.0f : candidates[index].detection.confidence) * candidates[index].visibility;
        RotatedRect mean(Point2f(0, 0, false), Size2f(0, 0, false), 0);
        for(const auto index : members) {
            auto box = candidates[index].box;
            while(box.angle - leader.box.angle > 45) {
                box.angle -= 90;
                std::swap(box.size.width, box.size.height);
            }
            while(box.angle - leader.box.angle < -45) {
                box.angle += 90;
                std::swap(box.size.width, box.size.height);
            }
            const auto& candidate = candidates[index];
            const float weight =
                static_cast<float>(static_cast<double>(equalWeights ? 1.0f : candidate.detection.confidence) * candidate.visibility / totalWeight);
            mean.center.x += weight * box.center.x;
            mean.center.y += weight * box.center.y;
            mean.size.width += weight * box.size.width;
            mean.size.height += weight * box.size.height;
            mean.angle += weight * box.angle;
            if(index != members.front())
                leader.maskMembers.insert(leader.maskMembers.end(), candidates[index].maskMembers.begin(), candidates[index].maskMembers.end());
        }
        leader.box = standardize(mean);
        writeBox(leader);
    }
    void suppress() {
        if(messages.size() == 1 || config.overlapMode == ImgDetectionsFilterConfig::OverlapMode::OFF) return;
        auto leaders = ranked;
        // Prefer the camera contributing more of the object; ranked remains confidence-based for output sorting/count limits.
        std::stable_sort(leaders.begin(), leaders.end(), [this](auto a, auto b) { return candidates[a].visibility > candidates[b].visibility; });
        std::vector<bool> grouped(candidates.size(), false);
        for(const auto leaderIndex : leaders) {
            if(grouped[leaderIndex]) continue;
            grouped[leaderIndex] = true;
            const auto& leader = candidates[leaderIndex];
            std::vector<std::size_t> members{leaderIndex};
            for(std::size_t key = 0; key < messages.size(); ++key) {
                if(key == leader.key) continue;
                std::optional<std::size_t> best;
                float bestIou = config.overlapIouThreshold;
                for(const auto index : ranked) {
                    const auto& candidate = candidates[index];
                    if(grouped[index] || candidate.key != key || candidate.detection.label != leader.detection.label) continue;
                    const float intersection = intersectionArea(leader.box, candidate.box);
                    const float unionArea =
                        leader.box.size.width * leader.box.size.height + candidate.box.size.width * candidate.box.size.height - intersection;
                    const float iou = unionArea > 0 ? intersection / unionArea : 0;
                    if(iou > bestIou) {
                        best = index;
                        bestIou = iou;
                    }
                }
                if(best) {
                    grouped[*best] = true;
                    removed[*best] = true;
                    members.push_back(*best);
                }
            }
            if(config.overlapMode == ImgDetectionsFilterConfig::OverlapMode::AVERAGE && members.size() > 1) average(members);
        }
    }
    void select() {
        for(std::size_t index = 0; index < candidates.size(); ++index) {
            if(removed[index]) continue;
            const auto& box = candidates[index].box;
            if(geometry) {
                const float area = box.size.width * box.size.height;
                if(area < config.minArea || area > config.maxArea || box.size.width < config.minWidth || box.size.width > config.maxWidth
                   || box.size.height < config.minHeight || box.size.height > config.maxHeight)
                    continue;
                if(config.regionOfInterest) {
                    const auto& roi = *config.regionOfInterest;
                    const auto corners = box.getPoints();
                    if(!std::all_of(corners.begin(), corners.end(), [&roi](const auto& p) {
                           return p.x >= roi.x && p.y >= roi.y && p.x <= roi.x + roi.width && p.y <= roi.y + roi.height;
                       }))
                        continue;
                }
            }
            kept.push_back(index);
        }
        if(config.maxDetections && kept.size() > *config.maxDetections) {
            std::vector<std::size_t> best;
            for(const auto index : ranked)
                if(std::find(kept.begin(), kept.end(), index) != kept.end()) best.push_back(index);
            best.resize(*config.maxDetections);
            kept.erase(std::remove_if(kept.begin(), kept.end(), [&best](auto i) { return std::find(best.begin(), best.end(), i) == best.end(); }), kept.end());
        }
        if(config.sortByConfidence)
            std::stable_sort(
                kept.begin(), kept.end(), [this](auto a, auto b) { return candidates[a].detection.confidence > candidates[b].detection.confidence; });
        output->detections.reserve(kept.size());
        for(const auto index : kept) output->detections.push_back(candidates[index].detection);
    }
    void mergeMask(std::vector<std::uint8_t>& mask,
                   std::size_t width,
                   std::size_t height,
                   std::size_t key,
                   const std::array<std::vector<MaskTarget>, 256>& indexMap,
                   const std::vector<std::uint8_t>& sourceMask) {
        const auto& message = messages[key];
        const auto sourceWidth = message->getSegmentationMaskWidth(), sourceHeight = message->getSegmentationMaskHeight();
        auto& cache = maskRemaps[key];
        if(reference) {
            // Compare exact values: approximate equality can miss a change across a source-pixel boundary.
            std::array<std::vector<std::uint8_t>, 2> transformations{utility::serialize(*reference), utility::serialize(*message->transformation)};
            const std::array<std::size_t, 4> dimensions{width, height, sourceWidth, sourceHeight};
            if(cache.dimensions != dimensions || cache.transformations != transformations) {
                if(sourceHeight != 0 && sourceWidth > std::numeric_limits<std::size_t>::max() / sourceHeight)
                    throw std::overflow_error("ImgDetectionsFilter source mask size overflow");
                std::vector<std::size_t> indices(mask.size(), std::numeric_limits<std::size_t>::max());
                for(std::size_t y = 0; y < height; ++y) {
                    for(std::size_t x = 0; x < width; ++x) {
                        const auto point = reference->remapPointTo(*message->transformation, Point2f((x + 0.5f) / width, (y + 0.5f) / height, true));
                        if(!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0 || point.y < 0 || point.x >= 1 || point.y >= 1) continue;
                        const auto sx = static_cast<std::size_t>(point.x * sourceWidth), sy = static_cast<std::size_t>(point.y * sourceHeight);
                        if(sx < sourceWidth && sy < sourceHeight) indices[y * width + x] = sy * sourceWidth + sx;
                    }
                }
                cache = {std::move(transformations), dimensions, std::move(indices)};
            }
        }
        for(std::size_t y = 0; y < height; ++y) {
            for(std::size_t x = 0; x < width; ++x) {
                if(!reference && (x >= sourceWidth || y >= sourceHeight)) continue;
                const auto sourceIndex = reference ? cache.sourceIndices[y * width + x] : y * sourceWidth + x;
                if(sourceIndex >= sourceMask.size()) continue;
                for(const auto& target : indexMap[sourceMask[sourceIndex]]) {
                    const float px = (x + 0.5f) / width;
                    if(px < target.xBounds[0] || px > target.xBounds[1]) continue;
                    const auto index = target.index;
                    const auto& detection = output->detections[index];
                    auto& previous = mask[y * width + x];
                    if(previous == 255 || detection.confidence > output->detections[previous].confidence
                       || (detection.confidence == output->detections[previous].confidence && index < previous))
                        previous = index;
                }
            }
        }
    }
    void buildMask() {
        std::vector<std::array<std::vector<MaskTarget>, 256>> indexMaps(messages.size());
        for(std::size_t index = 0; index < kept.size() && index < 255; ++index)
            for(const auto& member : candidates[kept[index]].maskMembers)
                if(member.index < 255) indexMaps[member.key][member.index].push_back({static_cast<std::uint8_t>(index), member.xBounds});
        std::vector<std::uint8_t> mask;
        std::size_t width = 0, height = 0;
        for(std::size_t key = 0; key < messages.size(); ++key) {
            const auto& message = messages[key];
            const auto sourceMask = message->getMaskData();
            if(!sourceMask) continue;
            if(mask.empty()) {
                width = reference ? size.first : message->getSegmentationMaskWidth();
                height = reference ? size.second : message->getSegmentationMaskHeight();
                if(height != 0 && width > std::numeric_limits<std::size_t>::max() / height) throw std::overflow_error("ImgDetectionsFilter mask size overflow");
                mask.assign(width * height, 255);
            }
            mergeMask(mask, width, height, key, indexMaps[key], *sourceMask);
        }
        if(width > 0 && height > 0) output->setSegmentationMask(mask, width, height);
    }

   public:
    DetectionRound(const std::vector<std::shared_ptr<ImgDetections>>& messages,
                   const ImgDetectionsFilterConfig& config,
                   const std::optional<ImgTransformation>& reference,
                   const std::vector<std::shared_ptr<ImgFrame>>& sourceMasks,
                   std::vector<MaskRemapCache>& maskRemaps)
        : messages(messages),
          config(config),
          reference(reference),
          sourceMasks(sourceMasks),
          maskRemaps(maskRemaps),
          geometry(config.hasGeometryFilters() || std::any_of(sourceMasks.begin(), sourceMasks.end(), [](const auto& mask) { return mask != nullptr; })) {}
    std::shared_ptr<ImgDetections> process() {
        prepare();
        collect();
        suppress();
        select();
        buildMask();
        return output;
    }
};
}  // namespace

std::shared_ptr<ImgDetections> filterDetectionRound(const std::vector<std::shared_ptr<ImgDetections>>& messages,
                                                    const ImgDetectionsFilterConfig& config,
                                                    const std::optional<ImgTransformation>& reference,
                                                    const std::vector<std::shared_ptr<ImgFrame>>& sourceMasks,
                                                    std::vector<MaskRemapCache>* maskRemaps) {
    std::vector<MaskRemapCache> localCache;
    auto& cache = maskRemaps ? *maskRemaps : localCache;
    cache.resize(messages.size());
    try {
        return DetectionRound(messages, config, reference, sourceMasks, cache).process();
    } catch(const std::invalid_argument& ex) {
        // ThreadedNode catches runtime_error when stopping a failed processing thread.
        throw std::runtime_error(ex.what());
    }
}
}  // namespace impl
}  // namespace dai
