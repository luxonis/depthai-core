#include "depthai/device/MultiDeviceCalibrationHandler.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

#include "depthai/utility/matrixOps.hpp"

namespace dai {
namespace {

using Transform = std::array<std::array<float, 4>, 4>;

struct CoordinateKey {
    std::string deviceId;
    CameraBoardSocket socket = CameraBoardSocket::AUTO;

    bool operator<(const CoordinateKey& other) const {
        if(deviceId != other.deviceId) {
            return deviceId < other.deviceId;
        }
        return static_cast<int32_t>(socket) < static_cast<int32_t>(other.socket);
    }

    bool operator==(const CoordinateKey& other) const {
        return deviceId == other.deviceId && socket == other.socket;
    }
};

struct Adjacency {
    CoordinateKey neighbour;
    Transform currentToNeighbour;
};

struct BuiltGraph {
    std::map<CoordinateKey, std::vector<Adjacency>> adjacency;
    std::map<std::string, CameraBoardSocket> deviceSockets;
    std::map<CoordinateKey, Extrinsics> bridges;
};

bool isConcreteSocket(CameraBoardSocket socket) {
    return socket != CameraBoardSocket::AUTO && static_cast<int32_t>(socket) >= static_cast<int32_t>(CameraBoardSocket::CAM_A)
           && static_cast<int32_t>(socket) <= static_cast<int32_t>(CameraBoardSocket::CBA);
}

bool isSupportedLengthUnit(LengthUnit unit) {
    switch(unit) {
        case LengthUnit::METER:
        case LengthUnit::CENTIMETER:
        case LengthUnit::MILLIMETER:
        case LengthUnit::INCH:
        case LengthUnit::FOOT:
            return true;
        case LengthUnit::CUSTOM:
        default:
            return false;
    }
}

bool isFiniteTranslation(const Point3f& translation) {
    return std::isfinite(translation.x) && std::isfinite(translation.y) && std::isfinite(translation.z);
}

Extrinsics normalizeExtrinsics(const Extrinsics& input) {
    if(input.toDeviceId.empty()) {
        throw std::invalid_argument("Multi-device extrinsics destination device ID cannot be empty.");
    }
    if(!isConcreteSocket(input.toCameraSocket)) {
        throw std::invalid_argument("Multi-device extrinsics destination socket must be concrete.");
    }
    if(!isSupportedLengthUnit(input.lengthUnit)) {
        throw std::invalid_argument("Multi-device extrinsics length unit is not supported.");
    }
    try {
        matrix::validateRotationMatrix3x3(input.rotationMatrix);
    } catch(const std::runtime_error&) {
        throw std::invalid_argument("Multi-device extrinsics rotation matrix must be a finite proper 3x3 rotation.");
    }
    if(!isFiniteTranslation(input.translation)) {
        throw std::invalid_argument("Multi-device extrinsics translation must be finite.");
    }

    const auto translation = input.getTranslationVector(false, LengthUnit::METER);
    Extrinsics normalized(input.rotationMatrix, Point3f(translation[0], translation[1], translation[2]), input.toCameraSocket, LengthUnit::METER);
    normalized.toDeviceId = input.toDeviceId;
    return normalized;
}

bool coordinateLess(const CoordinateKey& lhs, const CoordinateKey& rhs) {
    return lhs < rhs;
}

Transform identityTransform() {
    return {{{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}}};
}

Extrinsics bridgeFromTransform(const Transform& transform, const CoordinateKey& target) {
    auto mutableTransform = transform;
    Extrinsics result(mutableTransform, target.socket, LengthUnit::METER);
    result.toDeviceId = target.deviceId;
    return result;
}

std::vector<MultiDeviceExtrinsics> normalizeAndSortGraph(std::vector<MultiDeviceExtrinsics> input) {
    std::set<std::pair<CoordinateKey, CoordinateKey>> seenEdges;
    std::map<std::string, CameraBoardSocket> deviceSockets;

    for(auto& edge : input) {
        if(edge.fromDeviceId.empty()) {
            throw std::invalid_argument("Multi-device extrinsics source device ID cannot be empty.");
        }
        if(!isConcreteSocket(edge.fromSocket)) {
            throw std::invalid_argument("Multi-device extrinsics source socket must be concrete.");
        }

        edge.extrinsics = normalizeExtrinsics(edge.extrinsics);
        if(edge.fromDeviceId == edge.extrinsics.toDeviceId) {
            throw std::invalid_argument("Multi-device extrinsics edges must connect different devices.");
        }

        const CoordinateKey source{edge.fromDeviceId, edge.fromSocket};
        const CoordinateKey destination{edge.extrinsics.toDeviceId, edge.extrinsics.toCameraSocket};
        const auto ordered = coordinateLess(source, destination) ? std::make_pair(source, destination) : std::make_pair(destination, source);
        if(!seenEdges.insert(ordered).second) {
            throw std::invalid_argument("Duplicate or reverse-duplicate multi-device extrinsics edge.");
        }

        const auto registerSocket = [&deviceSockets](const CoordinateKey& coordinate) {
            const auto [it, inserted] = deviceSockets.emplace(coordinate.deviceId, coordinate.socket);
            if(!inserted && it->second != coordinate.socket) {
                throw std::invalid_argument("A device cannot use multiple local calibration-origin sockets in the multi-device graph.");
            }
        };
        registerSocket(source);
        registerSocket(destination);
    }

    std::sort(input.begin(), input.end(), [](const MultiDeviceExtrinsics& lhs, const MultiDeviceExtrinsics& rhs) {
        if(lhs.fromDeviceId != rhs.fromDeviceId) {
            return lhs.fromDeviceId < rhs.fromDeviceId;
        }
        if(lhs.fromSocket != rhs.fromSocket) {
            return static_cast<int32_t>(lhs.fromSocket) < static_cast<int32_t>(rhs.fromSocket);
        }
        if(lhs.extrinsics.toDeviceId != rhs.extrinsics.toDeviceId) {
            return lhs.extrinsics.toDeviceId < rhs.extrinsics.toDeviceId;
        }
        return static_cast<int32_t>(lhs.extrinsics.toCameraSocket) < static_cast<int32_t>(rhs.extrinsics.toCameraSocket);
    });

    return input;
}

BuiltGraph buildGraph(const std::vector<MultiDeviceExtrinsics>& graph) {
    BuiltGraph built;

    for(const auto& edge : graph) {
        const CoordinateKey source{edge.fromDeviceId, edge.fromSocket};
        const CoordinateKey destination{edge.extrinsics.toDeviceId, edge.extrinsics.toCameraSocket};
        const auto forward = edge.extrinsics.getTransformationMatrix(false, LengthUnit::METER);
        const auto reverse = matrix::invertSe3Matrix4x4(forward);

        built.adjacency[source].push_back({destination, forward});
        built.adjacency[destination].push_back({source, reverse});
        built.deviceSockets[source.deviceId] = source.socket;
        built.deviceSockets[destination.deviceId] = destination.socket;
    }

    std::set<CoordinateKey> visited;
    for(const auto& [start, unused] : built.adjacency) {
        static_cast<void>(unused);
        if(visited.count(start) != 0) {
            continue;
        }

        std::map<CoordinateKey, Transform> toRoot;
        std::map<CoordinateKey, std::optional<CoordinateKey>> parent;
        std::vector<CoordinateKey> stack{start};
        toRoot[start] = identityTransform();
        parent[start] = std::nullopt;
        visited.insert(start);

        CoordinateKey root = start;
        while(!stack.empty()) {
            const auto current = stack.back();
            stack.pop_back();
            if(current < root) {
                root = current;
            }

            for(const auto& adjacency : built.adjacency.at(current)) {
                if(visited.count(adjacency.neighbour) != 0) {
                    if(!parent.at(current).has_value() || !(adjacency.neighbour == parent.at(current).value())) {
                        throw std::invalid_argument("Multi-device calibration graph must be a forest; a cycle was detected.");
                    }
                    continue;
                }
                toRoot[adjacency.neighbour] = matrix::matMul(toRoot.at(current), matrix::invertSe3Matrix4x4(adjacency.currentToNeighbour));
                parent[adjacency.neighbour] = current;
                visited.insert(adjacency.neighbour);
                stack.push_back(adjacency.neighbour);
            }
        }

        // The first traversal starts at the first map key, which is already
        // the lowest key in this component. Keep the explicit comparison above
        // as a guard for future changes to the traversal container.
        for(const auto& [coordinate, transform] : toRoot) {
            built.bridges[coordinate] = bridgeFromTransform(transform, root);
        }
    }

    return built;
}

}  // namespace

struct MultiDeviceCalibrationHandler::ResolvedGraph {
    BuiltGraph built;
};

MultiDeviceCalibrationHandler::MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics> graph) : graph(normalizeAndSortGraph(std::move(graph))) {
    getResolvedGraph();
}

MultiDeviceCalibrationHandler::MultiDeviceCalibrationHandler(std::filesystem::path calibrationDataPath) {
    std::ifstream stream(calibrationDataPath);
    if(!stream.is_open()) {
        throw std::runtime_error("Multi-device calibration file does not exist at the provided path.");
    }
    nlohmann::json json;
    stream >> json;
    *this = fromJson(json);
}

MultiDeviceCalibrationHandler MultiDeviceCalibrationHandler::fromJson(const nlohmann::json& calibrationDataJson) {
    return MultiDeviceCalibrationHandler(calibrationDataJson.at("graph").get<std::vector<MultiDeviceExtrinsics>>());
}

nlohmann::json MultiDeviceCalibrationHandler::toJson() const {
    return nlohmann::json(*this);
}

bool MultiDeviceCalibrationHandler::toJsonFile(std::filesystem::path destPath) const {
    std::ofstream stream(destPath);
    if(!stream.is_open()) return false;
    stream << std::setw(4) << toJson() << std::endl;
    return stream.good();
}

std::shared_ptr<const MultiDeviceCalibrationHandler::ResolvedGraph> MultiDeviceCalibrationHandler::getResolvedGraph() const {
    if(const auto cached = std::atomic_load(&resolvedGraph)) {
        return cached;
    }

    auto resolved = std::make_shared<ResolvedGraph>();
    resolved->built = buildGraph(normalizeAndSortGraph(graph));
    std::atomic_store(&resolvedGraph, std::shared_ptr<const ResolvedGraph>(resolved));
    return std::atomic_load(&resolvedGraph);
}

std::optional<CameraBoardSocket> MultiDeviceCalibrationHandler::getDeviceSocket(const std::string& deviceId) const {
    const auto resolved = getResolvedGraph();
    const auto it = resolved->built.deviceSockets.find(deviceId);
    if(it == resolved->built.deviceSockets.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<Extrinsics> MultiDeviceCalibrationHandler::getExtrinsicsToOrigin(const std::string& deviceId, CameraBoardSocket localOriginSocket) const {
    const auto resolved = getResolvedGraph();
    const auto deviceIt = resolved->built.deviceSockets.find(deviceId);
    if(deviceIt == resolved->built.deviceSockets.end()) {
        return std::nullopt;
    }
    if(deviceIt->second != localOriginSocket) {
        throw std::invalid_argument("Multi-device calibration local origin socket does not match the graph for device '" + deviceId + "'.");
    }

    const CoordinateKey coordinate{deviceId, localOriginSocket};
    return resolved->built.bridges.at(coordinate);
}

}  // namespace dai
