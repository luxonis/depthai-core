#include "depthai/device/MultiDeviceCalibrationHandler.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <fstream>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

#include "depthai/utility/matrixOps.hpp"

namespace dai {

namespace {

constexpr LengthUnit INTERNAL_UNIT = LengthUnit::CENTIMETER;

std::array<std::array<float, 4>, 4> identity4x4() {
    return {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
}

/// Inverse of a rigid transformation, i.e. [R|t]^-1 == [R^T | -R^T t].
std::array<std::array<float, 4>, 4> invertRigid(const std::array<std::array<float, 4>, 4>& matrix) {
    std::array<std::array<float, 4>, 4> inverse = identity4x4();
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            inverse[i][j] = matrix[j][i];
        }
    }
    for(int i = 0; i < 3; ++i) {
        inverse[i][3] = -(inverse[i][0] * matrix[0][3] + inverse[i][1] * matrix[1][3] + inverse[i][2] * matrix[2][3]);
    }
    return inverse;
}

std::array<std::array<float, 4>, 4> scaleTranslation(std::array<std::array<float, 4>, 4> matrix, LengthUnit targetUnit) {
    const float scale = getDistanceUnitScale(targetUnit, INTERNAL_UNIT);
    for(int i = 0; i < 3; ++i) {
        matrix[i][3] *= scale;
    }
    return matrix;
}

}  // namespace

MultiDeviceCalibrationHandler::MultiDeviceCalibrationHandler(MultiDeviceCalibrationData data, bool validate) : data(std::move(data)) {
    rebuild(validate);
}

MultiDeviceCalibrationHandler::MultiDeviceCalibrationHandler(const std::filesystem::path& jsonPath) {
    std::ifstream file(jsonPath);
    if(!file.is_open()) {
        throw std::runtime_error(fmt::format("Multi-device calibration file '{}' could not be opened.", jsonPath.string()));
    }
    nlohmann::json json;
    file >> json;
    data = json.get<MultiDeviceCalibrationData>();
    rebuild(true);
}

MultiDeviceCalibrationHandler MultiDeviceCalibrationHandler::fromJson(const nlohmann::json& json, bool validate) {
    return MultiDeviceCalibrationHandler(json.get<MultiDeviceCalibrationData>(), validate);
}

const MultiDeviceCalibrationData& MultiDeviceCalibrationHandler::getData() const {
    return data;
}

nlohmann::json MultiDeviceCalibrationHandler::toJson() const {
    return nlohmann::json(data);
}

bool MultiDeviceCalibrationHandler::toJsonFile(const std::filesystem::path& jsonPath) const {
    std::ofstream file(jsonPath);
    if(!file.is_open()) {
        return false;
    }
    file << toJson().dump(4);
    return file.good();
}

void MultiDeviceCalibrationHandler::rebuild(bool validate) {
    frames.clear();
    componentRoots.clear();

    // Adjacency: frame -> (neighbour, transformation from frame to neighbour)
    std::map<CoordinateFrame, std::vector<std::pair<CoordinateFrame, std::array<std::array<float, 4>, 4>>>> adjacency;
    std::set<std::pair<CoordinateFrame, CoordinateFrame>> seenEdges;
    for(const auto& edge : data.edges) {
        if(edge.from == edge.to) {
            throw std::runtime_error(fmt::format("Multi-device calibration edge from {} to itself is not allowed.", toString(edge.from)));
        }
        if(validate) {
            const auto key = std::minmax(edge.from, edge.to);
            if(!seenEdges.insert({key.first, key.second}).second) {
                throw std::runtime_error(
                    fmt::format("Multi-device calibration contains a duplicate edge between {} and {}.", toString(edge.from), toString(edge.to)));
            }
        }
        const auto transform = edge.transform.getTransformationMatrix(false, INTERNAL_UNIT);
        adjacency[edge.from].emplace_back(edge.to, transform);
        adjacency[edge.to].emplace_back(edge.from, invertRigid(transform));
    }

    // Frames are ordered, so the first unvisited frame of a component is its deterministic root.
    for(const auto& [frame, neighbours] : adjacency) {
        (void)neighbours;
        if(frames.count(frame) > 0) continue;

        const size_t component = componentRoots.size();
        componentRoots.push_back(frame);
        frames[frame] = FrameInfo{component, identity4x4()};

        std::queue<CoordinateFrame> pending;
        pending.push(frame);
        std::map<CoordinateFrame, CoordinateFrame> parent;
        while(!pending.empty()) {
            const auto current = pending.front();
            pending.pop();
            const auto toRoot = frames.at(current).toRoot;
            for(const auto& [neighbour, currentToNeighbour] : adjacency.at(current)) {
                const auto parentIt = parent.find(current);
                if(parentIt != parent.end() && parentIt->second == neighbour) continue;
                if(frames.count(neighbour) > 0) {
                    if(validate) {
                        throw std::runtime_error(fmt::format("Multi-device calibration graph must be a forest, but a cycle was found through {} and {}.",
                                                             toString(current),
                                                             toString(neighbour)));
                    }
                    continue;
                }
                // T_root<-neighbour == T_root<-current * T_current<-neighbour
                frames[neighbour] = FrameInfo{component, matrix::matMul(toRoot, invertRigid(currentToNeighbour))};
                parent[neighbour] = current;
                pending.push(neighbour);
            }
        }
    }

    ++revision;
}

std::vector<std::string> MultiDeviceCalibrationHandler::getDeviceIds() const {
    std::set<std::string> ids;
    for(const auto& [frame, info] : frames) {
        (void)info;
        ids.insert(frame.deviceId);
    }
    return {ids.begin(), ids.end()};
}

std::vector<CoordinateFrame> MultiDeviceCalibrationHandler::getFrames() const {
    std::vector<CoordinateFrame> result;
    result.reserve(frames.size());
    for(const auto& [frame, info] : frames) {
        (void)info;
        result.push_back(frame);
    }
    return result;
}

bool MultiDeviceCalibrationHandler::empty() const {
    return data.edges.empty();
}

bool MultiDeviceCalibrationHandler::canTransform(const CoordinateFrame& from, const CoordinateFrame& to) const {
    if(from == to) return true;
    const auto fromIt = frames.find(from);
    const auto toIt = frames.find(to);
    if(fromIt == frames.end() || toIt == frames.end()) return false;
    return fromIt->second.component == toIt->second.component;
}

CoordinateFrame MultiDeviceCalibrationHandler::getComponentRoot(const CoordinateFrame& frame) const {
    const auto it = frames.find(frame);
    if(it == frames.end()) {
        throw std::out_of_range(fmt::format("Frame {} is not part of the multi-device calibration.", toString(frame)));
    }
    return componentRoots.at(it->second.component);
}

std::vector<std::vector<CoordinateFrame>> MultiDeviceCalibrationHandler::getComponents() const {
    std::vector<std::vector<CoordinateFrame>> components(componentRoots.size());
    for(size_t i = 0; i < componentRoots.size(); ++i) {
        components[i].push_back(componentRoots[i]);
    }
    for(const auto& [frame, info] : frames) {
        if(frame == componentRoots.at(info.component)) continue;
        components[info.component].push_back(frame);
    }
    return components;
}

uint64_t MultiDeviceCalibrationHandler::getRevision() const {
    return revision;
}

std::array<std::array<float, 4>, 4> MultiDeviceCalibrationHandler::getTransform(const CoordinateFrame& from, const CoordinateFrame& to, LengthUnit unit) const {
    if(from == to) {
        return identity4x4();
    }
    const auto fromIt = frames.find(from);
    const auto toIt = frames.find(to);
    if(fromIt == frames.end() || toIt == frames.end()) {
        throw std::runtime_error(fmt::format("Cannot transform from {} to {}: {} is not part of the multi-device calibration.",
                                             toString(from),
                                             toString(to),
                                             toString(fromIt == frames.end() ? from : to)));
    }
    if(fromIt->second.component != toIt->second.component) {
        throw std::runtime_error(
            fmt::format("No transformation path between {} and {} - they belong to different connected components of the multi-device "
                        "calibration (rooted at {} and {}).",
                        toString(from),
                        toString(to),
                        toString(componentRoots.at(fromIt->second.component)),
                        toString(componentRoots.at(toIt->second.component))));
    }
    // T_to<-from == inv(T_root<-to) * T_root<-from
    return scaleTranslation(matrix::matMul(invertRigid(toIt->second.toRoot), fromIt->second.toRoot), unit);
}

void MultiDeviceCalibrationHandler::setEdge(const RigEdge& edge) {
    auto updated = data;
    updated.edges.erase(std::remove_if(updated.edges.begin(),
                                       updated.edges.end(),
                                       [&edge](const RigEdge& existing) {
                                           return (existing.from == edge.from && existing.to == edge.to)
                                                  || (existing.from == edge.to && existing.to == edge.from);
                                       }),
                        updated.edges.end());
    updated.edges.push_back(edge);
    updated.edges.back().transform.setReferenceFrame(edge.to);

    // Validate on a copy first, so a rejected edge leaves the handler untouched.
    MultiDeviceCalibrationHandler validated(updated, true);
    data = std::move(updated);
    frames = std::move(validated.frames);
    componentRoots = std::move(validated.componentRoots);
    ++revision;
}

bool MultiDeviceCalibrationHandler::removeEdge(const CoordinateFrame& from, const CoordinateFrame& to) {
    const auto removed = std::remove_if(data.edges.begin(), data.edges.end(), [&from, &to](const RigEdge& existing) {
        return (existing.from == from && existing.to == to) || (existing.from == to && existing.to == from);
    });
    if(removed == data.edges.end()) {
        return false;
    }
    data.edges.erase(removed, data.edges.end());
    rebuild(true);
    return true;
}

void MultiDeviceCalibrationHandler::resolveAliases(const std::map<std::string, std::string>& aliasToDeviceId) {
    std::map<std::string, std::string> aliases;
    for(const auto& [alias, deviceId] : data.aliases) {
        aliases[alias] = deviceId;
    }
    for(const auto& [alias, deviceId] : aliasToDeviceId) {
        aliases[alias] = deviceId;
    }
    if(aliases.empty()) return;

    const auto resolve = [&aliases](CoordinateFrame& frame) {
        const auto it = aliases.find(frame.deviceId);
        if(it != aliases.end()) {
            frame.deviceId = it->second;
        }
    };
    for(auto& edge : data.edges) {
        resolve(edge.from);
        resolve(edge.to);
        edge.transform.setReferenceFrame(edge.to);
    }
    rebuild(true);
}

void MultiDeviceCalibrationHandler::reexpress(Extrinsics& extrinsics, const CoordinateFrame& targetReference) const {
    const auto currentReference = extrinsics.getReferenceFrame();
    if(currentReference == targetReference) {
        return;
    }
    if(currentReference.isUnknown()) {
        throw std::runtime_error("Cannot re-express extrinsics with an unknown reference frame. Set the reference frame first.");
    }
    // T_target<-source == T_target<-current * T_current<-source
    const auto currentToTarget = getTransform(currentReference, targetReference, extrinsics.lengthUnit);
    const auto sourceToCurrent = extrinsics.getTransformationMatrix(false, extrinsics.lengthUnit);
    extrinsics.setTransformationMatrix(matrix::matMul(currentToTarget, sourceToCurrent), extrinsics.lengthUnit);
    extrinsics.setReferenceFrame(targetReference);
}

void MultiDeviceCalibrationHandler::reexpress(ImgTransformation& transformation, const CoordinateFrame& targetReference) const {
    auto extrinsics = transformation.getExtrinsics();
    reexpress(extrinsics, targetReference);
    transformation.setExtrinsics(extrinsics);
}

}  // namespace dai
