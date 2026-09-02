#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * A directed cross-device calibration edge.
 *
 * The source coordinate system is identified by fromDeviceId/fromSocket. The
 * destination coordinate system is identified by extrinsics.toDeviceId and
 * extrinsics.toCameraSocket.
 */
struct MultiDeviceExtrinsics {
    std::string fromDeviceId;
    CameraBoardSocket fromSocket = CameraBoardSocket::AUTO;
    Extrinsics extrinsics;
};

DEPTHAI_SERIALIZE_EXT(MultiDeviceExtrinsics, fromDeviceId, fromSocket, extrinsics);

/**
 * Resolves cross-device calibration edges into a common origin for each
 * connected component.
 *
 * The input graph is a forest. Every edge endpoint must be a device's local
 * calibration origin. The component origin is selected automatically as the
 * lowest (device ID, socket) coordinate-system pair.
 */
class MultiDeviceCalibrationHandler {
   public:
    MultiDeviceCalibrationHandler() = default;

    explicit MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics> graph);

    /** Construct and validate a handler from a JSON file. */
    explicit MultiDeviceCalibrationHandler(std::filesystem::path calibrationDataPath);

    /** Construct and validate a handler from its JSON representation. */
    static MultiDeviceCalibrationHandler fromJson(const nlohmann::json& calibrationDataJson);

    /** Return the handler's JSON representation with translations in centimeters. */
    nlohmann::json toJson() const;

    /** Write the handler's centimeter-normalized JSON representation to a file. */
    bool toJsonFile(std::filesystem::path destPath) const;

    /**
     * Get the local calibration-origin socket used by a device in the graph.
     *
     * @return The socket when the device participates in the graph, or
     * std::nullopt when it does not.
     */
    std::optional<CameraBoardSocket> getDeviceSocket(const std::string& deviceId) const;

    /**
     * Get the transform from a device's local calibration origin to its
     * connected component's automatically selected origin.
     *
     * @return A meter-normalized transform when the device participates in the
     * graph, or std::nullopt when it does not.
     * @throws std::runtime_error when the graph is invalid or the supplied
     * local socket does not match the graph.
     */
    std::optional<Extrinsics> getExtrinsicsToOrigin(const std::string& deviceId, CameraBoardSocket localOriginSocket) const;

   private:
    struct ResolvedGraph;

    std::shared_ptr<const ResolvedGraph> getResolvedGraph() const;

    std::vector<MultiDeviceExtrinsics> graph;
    mutable std::shared_ptr<const ResolvedGraph> resolvedGraph;

    friend void to_json(nlohmann::json& json, const MultiDeviceCalibrationHandler& handler);

    friend void from_json(const nlohmann::json& json, MultiDeviceCalibrationHandler& handler) {
        handler = MultiDeviceCalibrationHandler(json.at("graph").get<std::vector<MultiDeviceExtrinsics>>());
    }

    DEPTHAI_DISPLAY(MultiDeviceCalibrationHandler)
    NOP_STRUCTURE(MultiDeviceCalibrationHandler, graph);
};

}  // namespace dai
