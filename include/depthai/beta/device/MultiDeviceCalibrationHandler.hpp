#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace beta {

/**
 * Facts about a device's local calibration that
 * MultiDeviceCalibrationHandler::validateLocalOrigin checks against the graph.
 *
 * The rule for the local origin depends on the platform whose frame sources
 * publish the extrinsics: RVC4 and the host use the extrinsics chain root, RVC2
 * frame sources publish extrinsics to the lowest-id socket. describeLocalCalibration
 * in LocalCalibrationOrigin.hpp applies the right rule for a platform and is
 * what the host and both firmwares call.
 */
struct LocalCalibrationOrigin {
    /** False when the device has no local calibration data at all. */
    bool available = false;
    /** True when the local calibration holds the given socket. */
    std::function<bool(CameraBoardSocket)> hasSocket;
    /** Socket that the local calibration uses as the extrinsics origin of the given socket. */
    std::function<CameraBoardSocket(CameraBoardSocket)> originOf;
    /** Human-readable name of the rule behind originOf, quoted in mismatch errors (optional). */
    const char* originRule = nullptr;
};

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

    /** Return the validated, meter-normalized calibration edges. */
    const std::vector<MultiDeviceExtrinsics>& getGraph() const;

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
     * @throws std::invalid_argument when the graph is invalid or the supplied
     * local socket does not match the graph.
     */
    std::optional<Extrinsics> getExtrinsicsToOrigin(const std::string& deviceId, CameraBoardSocket localOriginSocket) const;

    /**
     * Verify that a device's local calibration origin matches the socket the
     * graph uses for it. A device absent from the graph always passes.
     *
     * @param deviceId The device's stable identifier
     * @param local The device's local calibration as the platform sees it
     * @throws std::runtime_error naming both the expected and the discovered
     * identity when the local calibration is missing, lacks the graph socket or
     * resolves to a different origin.
     */
    void validateLocalOrigin(const std::string& deviceId, const LocalCalibrationOrigin& local) const;

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
};

}  // namespace beta
}  // namespace dai
