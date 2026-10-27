#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * ArUco / AprilTag dictionaries supported by the calibration marker detector.
 */
enum class ArucoDictionary : int32_t {
    DICT_4X4_50 = 0,
    DICT_4X4_100,
    DICT_4X4_250,
    DICT_4X4_1000,
    DICT_5X5_50,
    DICT_5X5_100,
    DICT_5X5_250,
    DICT_5X5_1000,
    DICT_6X6_50,
    DICT_6X6_100,
    DICT_6X6_250,
    DICT_6X6_1000,
    DICT_7X7_50,
    DICT_7X7_100,
    DICT_7X7_250,
    DICT_7X7_1000,
    DICT_ARUCO_ORIGINAL,
    DICT_APRILTAG_16h5,
    DICT_APRILTAG_25h9,
    DICT_APRILTAG_36h10,
    DICT_APRILTAG_36h11,
};

std::string toString(ArucoDictionary dictionary);
ArucoDictionary arucoDictionaryFromString(const std::string& name);

/**
 * Metric reference for dynamic calibration given through ArUco markers placed in the scene.
 *
 * Image correspondences cannot observe how long the baseline between two cameras is, only its direction and the
 * relative rotation. Known distances between marker centers, and from marker centers to camera sockets, provide
 * that missing scale. The marker size does not matter, only the centers are used; a marker needs to be seen by
 * at least two of the calibrated cameras in the same synchronized frame set to count.
 *
 * Distances are stored in meters. The JSON form accepted by fromJson() is:
 *
 *     {
 *       "dictionary": "DICT_4X4_50",            // optional, default DICT_4X4_50
 *       "lengthUnit": "m",                      // optional: "m", "cm", "mm" or "in" for every distance below
 *       "markers": [37, 43],                    // optional: markers that belong to the scene (others are ignored)
 *       "markerDistances": [ {"markers": [37, 43], "distance": 0.84} ],
 *       "socketDistances": [ {"marker": 37, "socket": "CAM_A", "distance": 1.68} ]
 *     }
 *
 * Either list may be empty; at least one distance is required for the constraints to have an effect.
 */
struct ArucoMarkerConstraints {
    /// Distance between the centers of two markers.
    struct MarkerDistance {
        int32_t markerA = -1;
        int32_t markerB = -1;
        float distanceMeters = 0.0f;
        DEPTHAI_SERIALIZE(MarkerDistance, markerA, markerB, distanceMeters);
    };

    /// Distance between the center of a marker and the optical center of a camera socket. The socket may be one
    /// of the calibrated cameras or any other calibrated socket of the device.
    struct MarkerSocketDistance {
        int32_t marker = -1;
        CameraBoardSocket socket = CameraBoardSocket::AUTO;
        float distanceMeters = 0.0f;
        DEPTHAI_SERIALIZE(MarkerSocketDistance, marker, socket, distanceMeters);
    };

    ArucoDictionary dictionary = ArucoDictionary::DICT_4X4_50;
    /// Markers that belong to the scene. Empty means every marker that a distance refers to.
    std::vector<int32_t> markerIds;
    std::vector<MarkerDistance> markerDistances;
    std::vector<MarkerSocketDistance> socketDistances;
    /// How many synchronized frame sets with marker observations are kept for the calibration (oldest dropped).
    uint32_t maxObservationSets = 50;

    /// Parses the JSON form documented above. Throws std::invalid_argument on malformed input.
    static ArucoMarkerConstraints fromJson(const nlohmann::json& json);
    static ArucoMarkerConstraints fromJsonString(const std::string& json);
    static ArucoMarkerConstraints fromFile(const std::filesystem::path& path);
    /// The JSON form documented above, distances in meters.
    nlohmann::json toJson() const;

    /// True when no distance is given.
    bool empty() const;
    /// markerIds together with every marker a distance refers to, sorted, without duplicates.
    std::vector<int32_t> referencedMarkerIds() const;
    /// Sockets referred to by socketDistances, sorted, without duplicates.
    std::vector<CameraBoardSocket> referencedSockets() const;
    /// Throws std::invalid_argument when a distance is not positive, refers to the same marker twice, or names
    /// a marker outside markerIds (when markerIds is given) or the AUTO socket.
    void validate() const;

    DEPTHAI_SERIALIZE(ArucoMarkerConstraints, dictionary, markerIds, markerDistances, socketDistances, maxObservationSets);
};

/**
 * Marker centers found in one synchronized set of frames: pixel coordinates per marker id and camera socket.
 */
struct ArucoMarkerObservationSet {
    std::map<int32_t, std::map<CameraBoardSocket, std::array<float, 2>>> centers;
    DEPTHAI_SERIALIZE(ArucoMarkerObservationSet, centers);
};

}  // namespace dai
