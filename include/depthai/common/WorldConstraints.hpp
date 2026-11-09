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
 * Marker families accepted by the calibration. Marker IDs identify printed targets across camera views.
 */
enum class MarkerType : int32_t {
    ARUCO = 0,    ///< ArUco / AprilTag markers of one dictionary; the reference point is the marker center.
    LUXONIS = 1,  ///< Luxonis markers, IDs 0-5; the reference point is the marker center.
};
std::string toString(MarkerType type);
MarkerType markerTypeFromString(const std::string& name);

/**
 * Metric reference for dynamic calibration given through fiducial markers placed in the scene.
 *
 * Known distances between marker centers, distances from marker centers to camera optical centers, and printed
 * marker sizes refine camera orientations after image-based calibration. This refinement preserves camera centers
 * and baseline lengths. A marker needs to be seen by at least two calibrated cameras in the same synchronized
 * frame set to count, and needs to be at least 25 px across in each image (ArUco: shortest side of the square,
 * Luxonis: smallest ring diameter). One constraint set describes markers of one type. Measurements are fitted together and may
 * not be met exactly. Set the constraints before building the pipeline.
 *
 * Distances are stored in meters. The JSON form accepted by fromJson() is:
 *
 *     {
 *       "markerType": "aruco",                  // optional, default aruco; "luxonis" for Luxonis Markers
 *       "arucoDictionary": "DICT_4X4_50",       // optional, default DICT_4X4_50 ("dictionary" is accepted too); ARUCO only
 *       "lengthUnit": "m",                      // optional: "m", "cm", "mm" or "in" for every distance below
 *       "markers": [37, 43],                    // optional: markers that belong to the scene (others are ignored)
 *       "markerDistances": [ {"markers": [37, 43], "distance": 0.84} ],
 *       "socketDistances": [ {"marker": 37, "socket": "CAM_A", "distance": 1.68} ],
 *       "markerSizes": [ {"marker": 37, "size": 0.10} ],
 *       "maxObservationSets": 50                // optional: synchronized frame sets with markers kept, default 50
 *     }
 *
 * The size of an ArUco marker is the length of a side of its black square, the distance between two neighbouring
 * corners; the size of a Luxonis Marker is the outer diameter of its lens ring. Any of the three lists may be empty;
 * at least one entry is required for the constraints to have an effect. Names are matched regardless of case; the
 * dictionary may omit its "DICT_" prefix, a socket may be given as "A" or its number, "ft" is a lengthUnit too, and
 * a marker distance may name its ends as {"markerA": 37, "markerB": 43, "distance": 0.84}.
 */
struct WorldConstraints {
    /// Distance between the reference points of two markers.
    struct MarkerDistance {
        int32_t markerA = -1;
        int32_t markerB = -1;
        float distanceMeters = 0.0f;
        DEPTHAI_SERIALIZE(MarkerDistance, markerA, markerB, distanceMeters);
    };

    /// Distance between the reference point of a marker and the optical center of a camera socket. The socket may
    /// be one of the calibrated cameras or any other calibrated socket of the device.
    struct MarkerSocketDistance {
        int32_t marker = -1;
        CameraBoardSocket socket = CameraBoardSocket::AUTO;
        float distanceMeters = 0.0f;
        DEPTHAI_SERIALIZE(MarkerSocketDistance, marker, socket, distanceMeters);
    };

    /// Printed size of a marker: ARUCO the length of a side of the black square, LUXONIS the outer diameter of the ring.
    struct MarkerSize {
        int32_t marker = -1;
        float sizeMeters = 0.0f;
        DEPTHAI_SERIALIZE(MarkerSize, marker, sizeMeters);
    };

    MarkerType markerType = MarkerType::ARUCO;
    /// ARUCO: the dictionary the markers were printed from. Ignored by other marker types.
    ArucoDictionary arucoDictionary = ArucoDictionary::DICT_4X4_50;
    /// Markers that belong to the scene. Empty means every marker that a distance refers to.
    std::vector<int32_t> markerIds;
    std::vector<MarkerDistance> markerDistances;
    std::vector<MarkerSocketDistance> socketDistances;
    std::vector<MarkerSize> markerSizes;
    /// How many synchronized frame sets with marker observations are kept for the calibration (oldest dropped).
    uint32_t maxObservationSets = 50;

    /// Parses the JSON form documented above. Throws std::invalid_argument on malformed input (also for JSON that
    /// does not parse or has a wrong type where a value is expected).
    static WorldConstraints fromJson(const nlohmann::json& json);
    static WorldConstraints fromJsonString(const std::string& json);
    static WorldConstraints fromFile(const std::filesystem::path& path);
    /// The JSON form documented above, distances in meters.
    nlohmann::json toJson() const;

    /// True when neither a distance nor a marker size is given.
    bool empty() const;
    /// markerIds together with every marker a distance or a size refers to, sorted, without duplicates.
    std::vector<int32_t> referencedMarkerIds() const;
    /// Sockets referred to by socketDistances, sorted, without duplicates.
    std::vector<CameraBoardSocket> referencedSockets() const;
    /// Throws std::invalid_argument when a distance or size is not positive, a distance refers to the same marker
    /// twice, a marker has two sizes, an entry names a marker outside markerIds (when markerIds is given) or the
    /// AUTO socket, maxObservationSets is 0, or markerType / arucoDictionary is not one of the enumerators.
    void validate() const;

    DEPTHAI_SERIALIZE(WorldConstraints, markerType, arucoDictionary, markerIds, markerDistances, socketDistances, markerSizes, maxObservationSets);

   private:
    static WorldConstraints parseJson(const nlohmann::json& json);
};

/**
 * Image of a marker's shape in one camera, in pixels: what a known marker size is measured against.
 */
struct MarkerOutline {
    /// ARUCO: the 4 corners, clockwise from the marker's top-left corner.
    std::vector<std::array<float, 2>> corners;
    /// LUXONIS: points on the outer edge of the lens ring.
    std::vector<std::array<float, 2>> outerEdge;
    /// LUXONIS: points on the inner edge of the lens ring.
    std::vector<std::array<float, 2>> innerEdge;
    DEPTHAI_SERIALIZE(MarkerOutline, corners, outerEdge, innerEdge);
};

/**
 * Markers found in one synchronized set of frames, per marker id and camera socket: reference points in pixels and,
 * for the markers with a known size, their outlines.
 */
struct MarkerObservationSet {
    /// Reference point of each marker in each camera that saw it, in pixels.
    std::map<int32_t, std::map<CameraBoardSocket, std::array<float, 2>>> centers;
    /// Outline of each marker with a configured size in each camera that saw it.
    std::map<int32_t, std::map<CameraBoardSocket, MarkerOutline>> outlines;
    DEPTHAI_SERIALIZE(MarkerObservationSet, centers, outlines);
};

}  // namespace dai
