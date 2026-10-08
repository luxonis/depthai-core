#include "ImgDetectionsFilterImpl.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "depthai/utility/ImageManipImpl.hpp"

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

struct Candidate {
    ImgDetection detection;
    RotatedRect box;
    std::size_t key;
    std::vector<std::pair<std::size_t, std::size_t>> maskMembers;
};

class DetectionRound {
    const std::vector<std::shared_ptr<ImgDetections>>& messages;
    const ImgDetectionsFilterConfig& config;
    const std::optional<ImgTransformation>& reference;
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
    }
    bool remap(Candidate& candidate, const ImgTransformation& source, bool pixels) const {
        auto& detection = candidate.detection;
        const bool identity = source.isEqualTransformation(*reference);
        if(!identity) {
            std::vector<std::array<float, 2>> corners;
            corners.reserve(4);
            for(const auto& corner : candidate.box.getPoints()) {
                const auto mapped = source.remapPointTo(*reference, corner);
                if(!std::isfinite(mapped.x) || !std::isfinite(mapped.y)) return false;
                corners.push_back({mapped.x, mapped.y});
            }
            candidate.box = standardize(getOuterRotatedRect(corners));
            candidate.box.center.hasNormalized = candidate.box.size.hasNormalized = true;
            candidate.box.center.normalized = candidate.box.size.normalized = false;
        }
        if(intersectionArea(candidate.box, RotatedRect(Rect(0, 0, size.first, size.second, false))) <= 0) return false;
        // Preserve exact normalized coordinates when the input already has standard form.
        if(!identity || pixels || !detection.boundingBox || candidate.box.angle != detection.boundingBox->angle) writeBox(candidate);
        detection.boundingBox->center.hasNormalized = detection.boundingBox->size.hasNormalized = true;
        detection.boundingBox->center.normalized = detection.boundingBox->size.normalized = true;
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
        return true;
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
                    if(reference && !remap(candidate, *message->transformation, pixels)) continue;
                }
                candidates.push_back(std::move(candidate));
            }
        }
        ranked.resize(candidates.size());
        std::iota(ranked.begin(), ranked.end(), 0);
        std::stable_sort(
            ranked.begin(), ranked.end(), [this](auto a, auto b) { return candidates[a].detection.confidence > candidates[b].detection.confidence; });
        removed.assign(candidates.size(), false);
    }
    void average(const std::vector<std::size_t>& members) {
        auto& leader = candidates[members.front()];
        float totalWeight = 0;
        for(const auto index : members) totalWeight += candidates[index].detection.confidence;
        const bool equalWeights = std::all_of(members.begin(), members.end(), [this](auto i) { return candidates[i].detection.confidence == 0; });
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
            const float weight = equalWeights ? 1.0f / members.size() : candidates[index].detection.confidence / totalWeight;
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
        std::vector<bool> grouped(candidates.size(), false);
        for(const auto leaderIndex : ranked) {
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
                   const std::array<std::uint8_t, 256>& indexMap,
                   const std::vector<std::uint8_t>& sourceMask) const {
        const auto& message = messages[key];
        const auto sourceWidth = message->getSegmentationMaskWidth(), sourceHeight = message->getSegmentationMaskHeight();
        for(std::size_t y = 0; y < height; ++y) {
            for(std::size_t x = 0; x < width; ++x) {
                std::size_t sx = x, sy = y;
                if(reference) {
                    const auto point = reference->remapPointTo(*message->transformation, Point2f((x + 0.5f) / width, (y + 0.5f) / height, true));
                    if(!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0 || point.y < 0 || point.x >= 1 || point.y >= 1) continue;
                    sx = static_cast<std::size_t>(point.x * sourceWidth);
                    sy = static_cast<std::size_t>(point.y * sourceHeight);
                }
                if(sx >= sourceWidth || sy >= sourceHeight || sy * sourceWidth + sx >= sourceMask.size()) continue;
                const auto index = indexMap[sourceMask[sy * sourceWidth + sx]];
                auto& previous = mask[y * width + x];
                if(index != 255
                   && (previous == 255 || output->detections[index].confidence > output->detections[previous].confidence
                       || (output->detections[index].confidence == output->detections[previous].confidence && index < previous)))
                    previous = index;
            }
        }
    }
    void buildMask() {
        std::vector<std::array<std::uint8_t, 256>> indexMaps(messages.size());
        for(auto& map : indexMaps) map.fill(255);
        for(std::size_t index = 0; index < kept.size() && index < 255; ++index)
            for(const auto& member : candidates[kept[index]].maskMembers)
                if(member.second < 255) indexMaps[member.first][member.second] = static_cast<std::uint8_t>(index);
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
                   const std::optional<ImgTransformation>& reference)
        : messages(messages), config(config), reference(reference), geometry(config.hasGeometryFilters()) {}
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
                                                    const std::optional<ImgTransformation>& reference) {
    return DetectionRound(messages, config, reference).process();
}
}  // namespace impl
}  // namespace dai
