#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
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

/// Name of the enumerator, such as "DICT_4X4_50"; "UNKNOWN" for a value outside the enumeration.
std::string toString(ArucoDictionary dictionary);
/// The dictionary named like the enumerator, matched regardless of case and with or without the "DICT_" prefix.
/// Throws std::invalid_argument for a name that is not a dictionary.
ArucoDictionary arucoDictionaryFromString(const std::string& name);

/**
 * Marker families accepted by the calibration. Marker IDs identify printed targets across camera views.
 */
enum class MarkerType : int32_t {
    ARUCO = 0,    ///< ArUco / AprilTag markers of one dictionary; the reference point is the marker center.
    LUXONIS = 1,  ///< Luxonis markers, IDs 0..kLuxonisMarkerIdMax; the reference point is the marker center.
};
/// Largest id a Luxonis Marker can carry.
constexpr int32_t kLuxonisMarkerIdMax = 5;
/// Name of the enumerator, "ARUCO" or "LUXONIS"; "UNKNOWN" for a value outside the enumeration.
std::string toString(MarkerType type);
/// The marker type named like the enumerator, matched regardless of case. Throws std::invalid_argument for any other name.
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
 * Every distance and size is in meters, in the struct and in the JSON. The constructors of the entries take an
 * optional LengthUnit to convert from. The JSON form accepted by fromJson() is:
 *
 *     {
 *       "markerType": "aruco",
 *       "arucoDictionary": "DICT_4X4_50",
 *       "markerDistances": [ {"markers": [37, 43], "distance": 0.84} ],
 *       "socketDistances": [ {"marker": 37, "socket": "CAM_A", "distance": 1.68} ],
 *       "markerSizes": [ {"marker": 37, "size": 0.10} ]
 *     }
 *
 * "markerType" is optional and defaults to "aruco"; "luxonis" selects Luxonis Markers. "arucoDictionary" is optional,
 * defaults to "DICT_4X4_50" and is allowed with "aruco" only. JSON comments are not accepted.
 *
 * The size of an ArUco marker is the length of a side of its black square, the distance between two neighbouring
 * corners; the size of a Luxonis Marker is the outer diameter of its lens ring. Any of the three lists may be empty;
 * at least one entry is required for the constraints to have an effect. Only the markers named by an entry are
 * looked for; any other marker in the scene is ignored.
 *
 * Names are matched regardless of case. For compatibility, "dictionary" aliases "arucoDictionary", the dictionary
 * may omit "DICT_", sockets accept letters or numbers, and marker distances accept "markerA" / "markerB" in place
 * of "markers". Marker type and dictionary enumerator numbers are also accepted.
 * fromJson() rejects unknown keys (including "lengthUnit"), malformed values, conflicting aliases, and everything
 * validate() rejects. fromJsonString() and fromFile() also reject a key given twice in one object and anything that
 * follows the document. Use the form above for new configuration files.
 */
struct WorldConstraints {
    /// Distance between the reference points of two markers, in meters.
    struct MarkerDistance {
        MarkerDistance() = default;
        /// distance is given in unit (meters by default) and stored in meters.
        MarkerDistance(int32_t markerA, int32_t markerB, float distance, LengthUnit unit = LengthUnit::METER);
        int32_t markerA = -1;
        int32_t markerB = -1;
        float distance = 0.0f;
        DEPTHAI_SERIALIZE_DECL(MarkerDistance, markerA, markerB, distance);
    };

    /// Distance between the reference point of a marker and the optical center of a camera socket. The socket may
    /// be one of the calibrated cameras or any other calibrated socket of the device.
    struct MarkerSocketDistance {
        MarkerSocketDistance() = default;
        /// distance is given in unit (meters by default) and stored in meters.
        MarkerSocketDistance(int32_t marker, CameraBoardSocket socket, float distance, LengthUnit unit = LengthUnit::METER);
        int32_t marker = -1;
        CameraBoardSocket socket = CameraBoardSocket::AUTO;
        float distance = 0.0f;
        DEPTHAI_SERIALIZE_DECL(MarkerSocketDistance, marker, socket, distance);
    };

    /// Printed size of a marker: ARUCO the length of a side of the black square, LUXONIS the outer diameter of the ring.
    struct MarkerSize {
        MarkerSize() = default;
        /// size is given in unit (meters by default) and stored in meters.
        MarkerSize(int32_t marker, float size, LengthUnit unit = LengthUnit::METER);
        int32_t marker = -1;
        float size = 0.0f;
        DEPTHAI_SERIALIZE_DECL(MarkerSize, marker, size);
    };

    MarkerType markerType = MarkerType::ARUCO;
    /// ARUCO: the dictionary the markers were printed from. Ignored by other marker types.
    ArucoDictionary arucoDictionary = ArucoDictionary::DICT_4X4_50;
    std::vector<MarkerDistance> markerDistances;
    std::vector<MarkerSocketDistance> socketDistances;
    std::vector<MarkerSize> markerSizes;

    /// Parses the JSON form documented above. Throws std::invalid_argument on malformed input (also for JSON that
    /// does not parse or has a wrong type where a value is expected).
    static WorldConstraints fromJson(const nlohmann::json& json);
    /// Same as fromJson() for a JSON document given as text.
    static WorldConstraints fromJsonString(const std::string& json);
    /// Same as fromJson() for a JSON document read from a file; a file that cannot be opened is a std::invalid_argument too.
    static WorldConstraints fromFile(const std::filesystem::path& path);
    /// The JSON form documented above.
    nlohmann::json toJson() const;

    /// True when neither a distance nor a marker size is given.
    bool empty() const;
    /// Throws std::invalid_argument when a distance or size is not positive and finite, a distance refers to the
    /// same marker twice, a pair of markers, a marker and socket, or a marker size is measured twice, an entry
    /// names a negative marker id or the AUTO socket, markerType / arucoDictionary is not one of the enumerators,
    /// an ARUCO configuration refers to a marker id its dictionary does not contain, or a LUXONIS configuration
    /// refers to a marker id above kLuxonisMarkerIdMax.
    void validate() const;

    DEPTHAI_SERIALIZE_DECL(WorldConstraints, markerType, arucoDictionary, markerDistances, socketDistances, markerSizes);

   private:
    /// fromJson() without the translation of nlohmann exceptions into std::invalid_argument.
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
    DEPTHAI_SERIALIZE_DECL(MarkerOutline, corners, outerEdge, innerEdge);
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
    DEPTHAI_SERIALIZE_DECL(MarkerObservationSet, centers, outlines);
};

}  // namespace dai
