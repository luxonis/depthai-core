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
    std::array<float, 2> extent{};
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
    bool upright = false;
    float period = 0, left = 0, right = 0;
    std::vector<Candidate> candidates, kept;
    std::vector<std::size_t> ranked;
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
    bool insideCanvas(const RotatedRect& box) const {
        return intersectionArea(box, RotatedRect(Rect(0, 0, size.first, size.second, false))) > 0;
    }
    // Only an unwrapped upright panorama box can cross the longitude join.
    bool crossesJoin(const RotatedRect& box) const {
        if(period <= 0 || box.angle != 0) return false;
        const auto outer = box.getOuterRect();
        return outer[0] < left || outer[2] > right;
    }
    // The parts of a box in the longitude range, left to right, each with the period shift that moves the box onto it.
    std::vector<std::pair<float, RotatedRect>> pieces(const RotatedRect& box) const {
        if(!crossesJoin(box)) return {{0.0f, box}};
        const auto outer = box.getOuterRect();
        std::vector<std::pair<float, RotatedRect>> parts;
        for(const float shift : {-period, 0.0f, period}) {
            const float minX = std::max(left, outer[0] + shift), maxX = std::min(right, outer[2] + shift);
            if(maxX > minX)
                parts.emplace_back(shift, RotatedRect(Point2f((minX + maxX) / 2, box.center.y, false), Size2f(maxX - minX, box.size.height, false), 0));
        }
        if(parts.size() > 1) {
            const float widest = std::max_element(parts.begin(), parts.end(), [](const auto& a, const auto& b) {
                                     return a.second.size.width < b.second.size.width;
                                 })->second.size.width;
            const float minimum = std::min(1.0f, widest);
            parts.erase(std::remove_if(parts.begin(), parts.end(), [minimum](const auto& part) { return part.second.size.width < minimum; }), parts.end());
        }
        return parts;
    }
    // Intersection area, also across the longitude join for unwrapped panorama boxes, which are narrower than one period.
    float overlap(const RotatedRect& lhs, RotatedRect rhs) const {
        if(period <= 0) return intersectionArea(lhs, rhs);
        rhs.center.x -= period * std::round((rhs.center.x - lhs.center.x) / period);
        float area = 0;
        for(const float shift : {-period, 0.0f, period}) {
            auto shifted = rhs;
            shifted.center.x += shift;
            area += intersectionArea(lhs, shifted);
        }
        return area;
    }
    // Source mask pixels that are set and whose centers lie inside a box in output pixels.
    std::size_t visiblePixels(const RotatedRect& box, const ImgFrame& mask) const {
        const auto outer = box.getOuterRect();
        if(box.size.width <= 0 || box.size.height <= 0 || !std::all_of(outer.begin(), outer.end(), [](float value) { return std::isfinite(value); })) return 0;
        const auto data = mask.getData();
        const auto xBegin = static_cast<std::size_t>(std::clamp(std::floor(static_cast<double>(outer[0])), 0.0, static_cast<double>(size.first)));
        const auto yBegin = static_cast<std::size_t>(std::clamp(std::floor(static_cast<double>(outer[1])), 0.0, static_cast<double>(size.second)));
        const auto xEnd = static_cast<std::size_t>(std::clamp(std::ceil(static_cast<double>(outer[2])), 0.0, static_cast<double>(size.first)));
        const auto yEnd = static_cast<std::size_t>(std::clamp(std::ceil(static_cast<double>(outer[3])), 0.0, static_cast<double>(size.second)));
        const double radians = static_cast<double>(box.angle) * std::acos(-1.0) / 180.0;
        const double cosine = std::cos(radians), sine = std::sin(radians);
        const std::size_t stride = mask.getStride();
        std::size_t visible = 0;
        // A seam can hide the center while leaving much of the object visible. Check the whole rotated footprint.
        for(auto y = yBegin; y < yEnd; ++y) {
            const double dy = static_cast<double>(y) + 0.5 - static_cast<double>(box.center.y);
            const auto row = mask.fb.p1Offset + y * stride;
            for(auto x = xBegin; x < xEnd; ++x) {
                const double dx = static_cast<double>(x) + 0.5 - static_cast<double>(box.center.x);
                visible += data[row + x] != 0 && std::abs(dx * cosine + dy * sine) <= static_cast<double>(box.size.width) / 2
                           && std::abs(dy * cosine - dx * sine) <= static_cast<double>(box.size.height) / 2;
            }
        }
        return visible;
    }
    // Output detections for a candidate: the candidate itself, or one fragment on each side of the longitude join.
    std::vector<Candidate> split(Candidate candidate) const {
        std::vector<Candidate> fragments;
        if(!crossesJoin(candidate.box)) {
            fragments.push_back(std::move(candidate));
            return fragments;
        }
        for(const auto& [shift, box] : pieces(candidate.box)) {
            if(!insideCanvas(box)) continue;
            // Keep a fragment that a contributing camera sees: one without a source mask, or one whose mask shows a pixel of it.
            if(!sourceMasks.empty() && std::none_of(candidate.maskMembers.begin(), candidate.maskMembers.end(), [this, &box](const auto& member) {
                   return !sourceMasks[member.key] || visiblePixels(box, *sourceMasks[member.key]) > 0;
               }))
                continue;
            auto fragment = candidate;
            fragment.box = box;
            writeBox(fragment);
            fragment.detection.boundingBox->center.hasNormalized = fragment.detection.boundingBox->size.hasNormalized = true;
            fragment.detection.boundingBox->center.normalized = fragment.detection.boundingBox->size.normalized = true;
            // Each member paints only its own pixels on this side of the join.
            fragment.maskMembers.clear();
            for(auto member : candidate.maskMembers) {
                const float minX = std::max(left, member.extent[0] + shift), maxX = std::min(right, member.extent[1] + shift);
                if(maxX <= minX) continue;
                member.xBounds = {minX / size.first, maxX / size.first};
                fragment.maskMembers.push_back(member);
            }
            if(fragment.detection.keypoints) {
                for(auto& point : fragment.detection.keypoints->keypoints)
                    if(point.imageCoordinates.x < fragment.detection.xmin || point.imageCoordinates.x > fragment.detection.xmax) point.confidence = 0;
            }
            fragments.push_back(std::move(fragment));
        }
        return fragments;
    }
    void checkTransformation(const ImgDetections& message) const {
        const auto source = message.getTransformation();
        if(!source || !source->isValid()) throw std::runtime_error("ImgDetectionsFilter requires a valid input ImgTransformation");
        if(!reference || source->isEqualTransformation(*reference)) return;
        const auto model = source->getDistortionModel();
        if(model == CameraModel::Equirectangular || model == CameraModel::Cylindrical)
            throw std::invalid_argument("ImgDetectionsFilter v1 cannot remap a panorama input to a different transformation");
        // Check compatibility even for empty messages; AUTO uses the existing identity-rotation remap rule.
        const auto fromExtrinsics = source->getExtrinsics(), toExtrinsics = reference->getExtrinsics();
        if(!fromExtrinsics.hasCompatibleCoordinateSystem(toExtrinsics)
           || (fromExtrinsics.toCameraSocket != CameraBoardSocket::AUTO && toExtrinsics.toCameraSocket != CameraBoardSocket::AUTO))
            source->getRotationMatrixTo(*reference);
    }
    void findJoin(const ImgTransformation& target) {
        const auto model = target.getDistortionModel();
        const auto intrinsics = target.getIntrinsicMatrix();
        upright = (model == CameraModel::Cylindrical || model == CameraModel::Equirectangular) && intrinsics[0][1] == 0 && intrinsics[1][0] == 0
                  && intrinsics[2][0] == 0 && intrinsics[2][1] == 0;
        const float wrap = 2 * std::acos(-1.0f) * std::abs(intrinsics[0][0] / intrinsics[2][2]);
        if(!upright || !std::isfinite(wrap) || wrap <= 0) return;
        period = wrap;
        const float center = intrinsics[0][2] / intrinsics[2][2];
        left = center - period / 2;
        right = center + period / 2;
    }
    void prepare() {
        if(messages.empty()) throw std::invalid_argument("ImgDetectionsFilter requires at least one linked input");
        if(reference || geometry)
            for(const auto& message : messages) checkTransformation(*message);
        // The first message wins timestamp ties.
        const auto newest =
            std::max_element(messages.begin(), messages.end(), [](const auto& a, const auto& b) { return a->getTimestamp() < b->getTimestamp(); });
        output->setBufferMetadataFrom(newest->get());
        output->transformation = reference ? reference : messages.front()->getTransformation();
        if(output->transformation) size = output->transformation->getSize();
        if(reference) findJoin(*reference);
        checkSourceMasks();
    }
    void checkSourceMasks() const {
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
    // Box edges mapped to the reference, or nothing when a sample is not projectable.
    static std::optional<std::vector<std::array<float, 2>>> sampleEdges(const RotatedRect& box,
                                                                        const ImgTransformation& source,
                                                                        const ImgTransformation& target,
                                                                        std::size_t edgeSamples) {
        const auto corners = box.getPoints();
        std::vector<std::array<float, 2>> points;
        points.reserve(corners.size() * edgeSamples);
        for(std::size_t edge = 0; edge < corners.size(); ++edge) {
            const auto a = corners[edge], b = corners[(edge + 1) % corners.size()];
            for(std::size_t sample = 0; sample < edgeSamples; ++sample) {
                const float t = static_cast<float>(sample) / edgeSamples;
                const auto mapped = source.remapPointTo(target, Point2f(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), false));
                if(!std::isfinite(mapped.x) || !std::isfinite(mapped.y)) return std::nullopt;
                points.push_back({mapped.x, mapped.y});
            }
        }
        return points;
    }
    // Longitude wraps at +/- pi. Unwrap before taking bounds; select() splits at that join, which need not be the canvas edges.
    RotatedRect unwrappedBounds(std::vector<std::array<float, 2>> points, const ImgTransformation& target) const {
        const float anchor = points.front()[0];
        float previous = anchor, unwrapped = anchor;
        for(auto& point : points) {
            const float x = point[0];
            unwrapped += std::remainder(x - previous, period);
            previous = x;
            point[0] = unwrapped;
        }
        const float winding = unwrapped - anchor + std::remainder(anchor - previous, period);
        const auto box = axisAlignedBounds(points);
        return std::abs(winding) > period / 2 ? enclosePole(box, target) : box;
    }
    // A footprint enclosing a pole spans all longitudes and reaches the pole, which no edge sample does. Source rays
    // lie in one hemisphere, so the middle of the sampled latitudes is on the side of the enclosed pole.
    RotatedRect enclosePole(const RotatedRect& samples, const ImgTransformation& target) const {
        const auto intrinsics = target.getIntrinsicMatrix();
        const float equator = intrinsics[1][2] / intrinsics[2][2];
        const bool above = samples.center.y < equator;
        // Cylindrical poles lie at infinity, so the box extends to the canvas edge instead.
        float pole = above ? 0.0f : static_cast<float>(size.second);
        if(target.getDistortionModel() == CameraModel::Equirectangular)
            pole = equator + (above ? -1.0f : 1.0f) * std::abs(intrinsics[1][1] / intrinsics[2][2]) * std::acos(-1.0f) / 2;
        const auto outer = samples.getOuterRect();
        const float minY = std::min(outer[1], pole), maxY = std::max(outer[3], pole);
        // The center x keeps the box exactly inside [left, right], so it is never split.
        const float center = intrinsics[0][2] / intrinsics[2][2];
        return {Point2f(center, (minY + maxY) / 2, false), Size2f(period, maxY - minY, false), 0};
    }
    // Remap keypoints in place; an unprojectable keypoint keeps its slot with zero confidence.
    static void remapKeypoints(ImgDetection& detection, const ImgTransformation& source, const ImgTransformation& target) {
        if(!detection.keypoints) return;
        for(auto& point : detection.keypoints->keypoints) {
            const auto mapped = source.remapPointTo(target, Point2f(point.imageCoordinates.x, point.imageCoordinates.y, true));
            if(!std::isfinite(mapped.x) || !std::isfinite(mapped.y)) {
                point.confidence = 0;
                continue;
            }
            point.imageCoordinates.x = mapped.x;
            point.imageCoordinates.y = mapped.y;
        }
    }
    void remap(Candidate candidate, const ImgTransformation& source, bool pixels) {
        auto& detection = candidate.detection;
        const auto& target = *reference;
        const bool identity = source.isEqualTransformation(target);
        if(!identity) {
            if(upright && period <= 0) return;
            const auto model = target.getDistortionModel();
            const bool panorama = model == CameraModel::Cylindrical || model == CameraModel::Equirectangular;
            // Panorama projections curve straight box edges; corners alone can underestimate the bounds and shift their center.
            const auto points = sampleEdges(candidate.box, source, target, panorama ? 16 : 1);
            if(!points) return;
            // The bent edges make the minimum-area rectangle tilt by several degrees even for a level camera, most near the image
            // border, and a normalized RotatedRect stays a rectangle only at angle 0. A panorama keeps the mean camera Y axis
            // vertical, so its boxes are the axis-aligned bounds of the sampled edges; perspective references keep the box
            // orientation, which an in-plane camera roll maps exactly.
            if(upright)
                candidate.box = unwrappedBounds(*points, target);
            else
                candidate.box = panorama ? axisAlignedBounds(*points) : standardize(getOuterRotatedRect(*points));
            candidate.box.center.hasNormalized = candidate.box.size.hasNormalized = true;
            candidate.box.center.normalized = candidate.box.size.normalized = false;
        }
        remapKeypoints(detection, source, target);
        const auto parts = pieces(candidate.box);
        if(std::none_of(parts.begin(), parts.end(), [this](const auto& part) { return insideCanvas(part.second); })) return;
        // Preserve exact normalized coordinates when the input already has standard form.
        if(!identity || pixels || !detection.boundingBox || candidate.box.angle != detection.boundingBox->angle) writeBox(candidate);
        detection.boundingBox->center.hasNormalized = detection.boundingBox->size.hasNormalized = true;
        detection.boundingBox->center.normalized = detection.boundingBox->size.normalized = true;
        const auto outer = candidate.box.getOuterRect();
        candidate.maskMembers.front().extent = {outer[0], outer[2]};
        candidates.push_back(std::move(candidate));
    }
    bool rejectedByLabelOrConfidence(const ImgDetection& detection) const {
        const auto contains = [&detection](const auto& labels) { return std::find(labels.begin(), labels.end(), detection.label) != labels.end(); };
        if(config.labelsToKeep && !contains(*config.labelsToKeep)) return true;
        if(config.labelsToReject && contains(*config.labelsToReject)) return true;
        return (config.minConfidence != 0 && detection.confidence < config.minConfidence)
               || (config.maxConfidence != 1 && detection.confidence > config.maxConfidence);
    }
    // Visible fraction of each box in its camera's source mask; boxes with no visible pixel are removed.
    void measureVisibility() {
        for(auto& candidate : candidates) {
            if(sourceMasks.empty() || !sourceMasks[candidate.key]) continue;
            const auto& box = candidate.box;
            std::size_t visible = 0;
            for(const auto& part : pieces(box)) visible += visiblePixels(part.second, *sourceMasks[candidate.key]);
            candidate.visibility = visible == 0 ? 0.0 : std::min(1.0, visible / (static_cast<double>(box.size.width) * static_cast<double>(box.size.height)));
        }
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const auto& candidate) { return candidate.visibility == 0; }),
                         candidates.end());
    }
    void collect() {
        for(std::size_t key = 0; key < messages.size(); ++key) {
            const auto& message = messages[key];
            for(std::size_t index = 0; index < message->detections.size(); ++index) {
                Candidate candidate{message->detections[index], {}, key, {{key, index}}};
                const auto& detection = candidate.detection;
                if(rejectedByLabelOrConfidence(detection)) continue;
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
        measureVisibility();
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
            // Unwrapped panorama boxes of one object can differ by a whole longitude period.
            const float shift = period > 0 ? -period * std::round((box.center.x - leader.box.center.x) / period) : 0.0f;
            box.center.x += shift;
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
            if(index == members.front()) continue;
            for(auto member : candidates[index].maskMembers) {
                member.extent[0] += shift;
                member.extent[1] += shift;
                leader.maskMembers.push_back(member);
            }
        }
        leader.box = standardize(mean);
        writeBox(leader);
    }
    // The ungrouped duplicate of a leader from one key: the same label with the highest IoU above the threshold, then confidence.
    std::optional<std::size_t> bestDuplicate(const Candidate& leader, std::size_t key, const std::vector<bool>& grouped) const {
        std::optional<std::size_t> best;
        float bestIou = config.overlapIouThreshold;
        for(const auto index : ranked) {
            const auto& candidate = candidates[index];
            if(grouped[index] || candidate.key != key || candidate.detection.label != leader.detection.label) continue;
            const float intersection = overlap(leader.box, candidate.box);
            const float unionArea = leader.box.size.width * leader.box.size.height + candidate.box.size.width * candidate.box.size.height - intersection;
            const float iou = unionArea > 0 ? intersection / unionArea : 0;
            if(iou > bestIou) {
                best = index;
                bestIou = iou;
            }
        }
        return best;
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
            std::vector<std::size_t> members{leaderIndex};
            for(std::size_t key = 0; key < messages.size(); ++key) {
                if(key == candidates[leaderIndex].key) continue;
                const auto best = bestDuplicate(candidates[leaderIndex], key, grouped);
                if(!best) continue;
                grouped[*best] = true;
                removed[*best] = true;
                members.push_back(*best);
            }
            if(config.overlapMode == ImgDetectionsFilterConfig::OverlapMode::AVERAGE && members.size() > 1) average(members);
        }
    }
    bool outsideSizeLimits(const RotatedRect& box) const {
        const float area = box.size.width * box.size.height;
        return area < config.minArea || area > config.maxArea || box.size.width < config.minWidth || box.size.width > config.maxWidth
               || box.size.height < config.minHeight || box.size.height > config.maxHeight;
    }
    static bool insideRegionOfInterest(const std::vector<Candidate>& fragments, const Rect& roi) {
        const auto inside = [&roi](const auto& p) { return p.x >= roi.x && p.y >= roi.y && p.x <= roi.x + roi.width && p.y <= roi.y + roi.height; };
        return std::all_of(fragments.begin(), fragments.end(), [&inside](const auto& fragment) {
            const auto corners = fragment.box.getPoints();
            return std::all_of(corners.begin(), corners.end(), inside);
        });
    }
    // Keep the best ranked output detections. The count limits output detections, so it can keep one fragment of a split object.
    void limitCount(const std::vector<std::size_t>& keptRanks, std::size_t count) {
        std::vector<std::size_t> order(kept.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&keptRanks](auto a, auto b) { return keptRanks[a] < keptRanks[b]; });
        std::vector<bool> best(kept.size(), false);
        for(std::size_t i = 0; i < count; ++i) best[order[i]] = true;
        std::vector<Candidate> limited;
        for(std::size_t i = 0; i < kept.size(); ++i)
            if(best[i]) limited.push_back(std::move(kept[i]));
        kept = std::move(limited);
    }
    void select() {
        std::vector<std::size_t> rank(candidates.size()), keptRanks;
        for(std::size_t i = 0; i < ranked.size(); ++i) rank[ranked[i]] = i;
        for(std::size_t index = 0; index < candidates.size(); ++index) {
            // Size limits apply to the whole object, also when the longitude join splits it.
            if(removed[index] || (geometry && outsideSizeLimits(candidates[index].box))) continue;
            auto fragments = split(std::move(candidates[index]));
            if(geometry && config.regionOfInterest && !insideRegionOfInterest(fragments, *config.regionOfInterest)) continue;
            for(auto& fragment : fragments) {
                kept.push_back(std::move(fragment));
                keptRanks.push_back(rank[index]);
            }
        }
        if(config.maxDetections && kept.size() > *config.maxDetections) limitCount(keptRanks, *config.maxDetections);
        if(config.sortByConfidence)
            std::stable_sort(kept.begin(), kept.end(), [](const auto& a, const auto& b) { return a.detection.confidence > b.detection.confidence; });
        output->detections.reserve(kept.size());
        for(const auto& candidate : kept) output->detections.push_back(candidate.detection);
    }
    void mergeMask(std::vector<std::uint8_t>& mask,
                   std::size_t width,
                   std::size_t height,
                   std::size_t key,
                   const std::array<std::vector<MaskTarget>, 256>& indexMap,
                   const std::vector<std::uint8_t>& sourceMask) {
        const auto& message = messages[key];
        const auto sourceWidth = message->getSegmentationMaskWidth(), sourceHeight = message->getSegmentationMaskHeight();
        if(reference) updateMaskRemap(key, width, height, *reference);
        const auto& cache = maskRemaps[key];
        for(std::size_t y = 0; y < height; ++y) {
            for(std::size_t x = 0; x < width; ++x) {
                if(!reference && (x >= sourceWidth || y >= sourceHeight)) continue;
                const auto sourceIndex = reference ? cache.sourceIndices[y * width + x] : y * sourceWidth + x;
                if(sourceIndex >= sourceMask.size()) continue;
                paint(mask[y * width + x], (x + 0.5f) / width, indexMap[sourceMask[sourceIndex]]);
            }
        }
    }
    // Rebuild the source mask pixel of each output pixel when the transformations or dimensions change.
    void updateMaskRemap(std::size_t key, std::size_t width, std::size_t height, const ImgTransformation& target) {
        const auto& message = messages[key];
        const auto sourceWidth = message->getSegmentationMaskWidth();
        const auto sourceHeight = message->getSegmentationMaskHeight();
        auto& cache = maskRemaps[key];
        // Compare exact values: approximate equality can miss a change across a source-pixel boundary.
        std::array<std::vector<std::uint8_t>, 2> transformations{utility::serialize(target), utility::serialize(*message->transformation)};
        const std::array<std::size_t, 4> dimensions{width, height, sourceWidth, sourceHeight};
        if(cache.dimensions == dimensions && cache.transformations == transformations) return;
        if(sourceHeight != 0 && sourceWidth > std::numeric_limits<std::size_t>::max() / sourceHeight)
            throw std::overflow_error("ImgDetectionsFilter source mask size overflow");
        std::vector<std::size_t> indices(width * height, std::numeric_limits<std::size_t>::max());
        for(std::size_t y = 0; y < height; ++y) {
            for(std::size_t x = 0; x < width; ++x) {
                const auto point = target.remapPointTo(*message->transformation, Point2f((x + 0.5f) / width, (y + 0.5f) / height, true));
                if(!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0 || point.y < 0 || point.x >= 1 || point.y >= 1) continue;
                const auto sx = static_cast<std::size_t>(point.x * sourceWidth), sy = static_cast<std::size_t>(point.y * sourceHeight);
                if(sx < sourceWidth && sy < sourceHeight) indices[y * width + x] = sy * sourceWidth + sx;
            }
        }
        cache = {std::move(transformations), dimensions, std::move(indices)};
    }
    // Give an output pixel to the target with the highest confidence, then the lowest index, whose x bounds contain it.
    void paint(std::uint8_t& pixel, float px, const std::vector<MaskTarget>& targets) const {
        for(const auto& target : targets) {
            if(px < target.xBounds[0] || px > target.xBounds[1]) continue;
            const auto index = target.index;
            const auto& detection = output->detections[index];
            if(pixel == 255 || detection.confidence > output->detections[pixel].confidence
               || (detection.confidence == output->detections[pixel].confidence && index < pixel))
                pixel = index;
        }
    }
    void buildMask() {
        std::vector<std::array<std::vector<MaskTarget>, 256>> indexMaps(messages.size());
        for(std::size_t index = 0; index < kept.size() && index < 255; ++index)
            for(const auto& member : kept[index].maskMembers)
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
