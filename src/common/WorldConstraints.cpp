#include "depthai/common/WorldConstraints.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include "depthai/common/DepthUnit.hpp"
#include "utility/WorldConstraintsUtils.hpp"

namespace dai {

namespace {

/// Every ArucoDictionary enumerator with its name, in enumeration order.
const std::vector<std::pair<ArucoDictionary, const char*>>& dictionaryNames() {
    static const std::vector<std::pair<ArucoDictionary, const char*>> names = {
        {ArucoDictionary::DICT_4X4_50, "DICT_4X4_50"},
        {ArucoDictionary::DICT_4X4_100, "DICT_4X4_100"},
        {ArucoDictionary::DICT_4X4_250, "DICT_4X4_250"},
        {ArucoDictionary::DICT_4X4_1000, "DICT_4X4_1000"},
        {ArucoDictionary::DICT_5X5_50, "DICT_5X5_50"},
        {ArucoDictionary::DICT_5X5_100, "DICT_5X5_100"},
        {ArucoDictionary::DICT_5X5_250, "DICT_5X5_250"},
        {ArucoDictionary::DICT_5X5_1000, "DICT_5X5_1000"},
        {ArucoDictionary::DICT_6X6_50, "DICT_6X6_50"},
        {ArucoDictionary::DICT_6X6_100, "DICT_6X6_100"},
        {ArucoDictionary::DICT_6X6_250, "DICT_6X6_250"},
        {ArucoDictionary::DICT_6X6_1000, "DICT_6X6_1000"},
        {ArucoDictionary::DICT_7X7_50, "DICT_7X7_50"},
        {ArucoDictionary::DICT_7X7_100, "DICT_7X7_100"},
        {ArucoDictionary::DICT_7X7_250, "DICT_7X7_250"},
        {ArucoDictionary::DICT_7X7_1000, "DICT_7X7_1000"},
        {ArucoDictionary::DICT_ARUCO_ORIGINAL, "DICT_ARUCO_ORIGINAL"},
        {ArucoDictionary::DICT_APRILTAG_16h5, "DICT_APRILTAG_16h5"},
        {ArucoDictionary::DICT_APRILTAG_25h9, "DICT_APRILTAG_25h9"},
        {ArucoDictionary::DICT_APRILTAG_36h10, "DICT_APRILTAG_36h10"},
        {ArucoDictionary::DICT_APRILTAG_36h11, "DICT_APRILTAG_36h11"},
    };
    return names;
}

/// Every MarkerType enumerator with its name.
const std::vector<std::pair<MarkerType, const char*>>& markerTypeNames() {
    static const std::vector<std::pair<MarkerType, const char*>> names = {
        {MarkerType::ARUCO, "ARUCO"},
        {MarkerType::LUXONIS, "LUXONIS"},
    };
    return names;
}

/// The text with every ASCII letter in upper case, for case-insensitive name matching.
std::string upper(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
}

/// True for a concrete camera socket (CAM_A..CAM_J), false for AUTO and values outside the enumeration.
bool isKnownSocket(CameraBoardSocket socket) {
    return socket >= CameraBoardSocket::CAM_A && socket <= CameraBoardSocket::CAM_J;
}

/// An integer that fits int32_t. Throws std::invalid_argument for anything else, including integers that would be
/// narrowed (nlohmann::json does not range-check get<T>()).
int32_t int32FromJson(const nlohmann::json& value, const std::string& what) {
    if(!value.is_number_integer()) {
        throw std::invalid_argument("WorldConstraints: " + what + " must be an integer, got " + value.dump());
    }
    const auto wide = value.get<int64_t>();
    if(value.is_number_unsigned() ? value.get<uint64_t>() > static_cast<uint64_t>(std::numeric_limits<int32_t>::max())
                                  : (wide < std::numeric_limits<int32_t>::min() || wide > std::numeric_limits<int32_t>::max())) {
        throw std::invalid_argument("WorldConstraints: " + what + " is out of range: " + value.dump());
    }
    return static_cast<int32_t>(wide);
}

/// A socket given as its number, its name ("CAM_B") or the letter alone ("b"), in any case. Throws std::invalid_argument otherwise.
CameraBoardSocket socketFromJson(const nlohmann::json& value) {
    if(value.is_number_integer()) {
        const auto number = int32FromJson(value, "socket");
        const auto socket = static_cast<CameraBoardSocket>(number);
        if(!isKnownSocket(socket)) {
            throw std::invalid_argument("WorldConstraints: unknown socket " + std::to_string(number));
        }
        return socket;
    }
    if(!value.is_string()) {
        throw std::invalid_argument("WorldConstraints: a socket must be a name such as \"CAM_A\"");
    }
    auto name = upper(value.get<std::string>());
    if(name.size() == 1) {
        name = "CAM_" + name;
    }
    for(int socket = static_cast<int>(CameraBoardSocket::CAM_A); socket <= static_cast<int>(CameraBoardSocket::CAM_J); ++socket) {
        if(toString(static_cast<CameraBoardSocket>(socket)) == name) {
            return static_cast<CameraBoardSocket>(socket);
        }
    }
    throw std::invalid_argument("WorldConstraints: unknown socket \"" + value.get<std::string>() + "\"");
}

/// A marker id, which must be a non-negative integer. Throws std::invalid_argument otherwise.
int32_t markerIdFromJson(const nlohmann::json& value) {
    if(!value.is_number_integer() || (!value.is_number_unsigned() && value.get<int64_t>() < 0)) {
        throw std::invalid_argument("WorldConstraints: marker ids must be non-negative integers, got " + value.dump());
    }
    return int32FromJson(value, "marker id");
}

/// An array of marker ids, each checked by markerIdFromJson(). Throws std::invalid_argument for anything but an array.
std::vector<int32_t> markerIdsFromJson(const nlohmann::json& value) {
    if(!value.is_array()) {
        throw std::invalid_argument("WorldConstraints: expected an array of marker ids, got " + value.dump());
    }
    std::vector<int32_t> ids;
    for(const auto& id : value) {
        ids.push_back(markerIdFromJson(id));
    }
    return ids;
}

/// The "distance" of a distance entry, in meters. Throws std::invalid_argument when it is missing or not a number.
float distanceFromJson(const nlohmann::json& entry) {
    if(!entry.contains("distance") || !entry.at("distance").is_number()) {
        throw std::invalid_argument("WorldConstraints: every distance entry needs a numeric \"distance\"");
    }
    return entry.at("distance").get<float>();
}

}  // namespace

std::string toString(ArucoDictionary dictionary) {
    for(const auto& [value, name] : dictionaryNames()) {
        if(value == dictionary) {
            return name;
        }
    }
    return "UNKNOWN";
}

ArucoDictionary arucoDictionaryFromString(const std::string& name) {
    const auto wanted = upper(name);
    for(const auto& [value, candidate] : dictionaryNames()) {
        // Compare without case: the AprilTag names keep a lowercase "h" (DICT_APRILTAG_36h11).
        const auto candidateUpper = upper(candidate);
        if(wanted == candidateUpper || wanted == candidateUpper.substr(5)) {
            return value;
        }
    }
    throw std::invalid_argument("WorldConstraints: unknown dictionary \"" + name + "\"");
}

std::string toString(MarkerType type) {
    for(const auto& [value, name] : markerTypeNames()) {
        if(value == type) {
            return name;
        }
    }
    return "UNKNOWN";
}

MarkerType markerTypeFromString(const std::string& name) {
    const auto wanted = upper(name);
    for(const auto& [value, candidate] : markerTypeNames()) {
        if(wanted == candidate) {
            return value;
        }
    }
    throw std::invalid_argument("WorldConstraints: unknown markerType \"" + name + "\"");
}

WorldConstraints WorldConstraints::fromJson(const nlohmann::json& json) {
    try {
        return parseJson(json);
    } catch(const nlohmann::json::exception& ex) {
        throw std::invalid_argument(std::string("WorldConstraints: malformed JSON: ") + ex.what());
    }
}

WorldConstraints WorldConstraints::parseJson(const nlohmann::json& json) {
    if(!json.is_object()) {
        throw std::invalid_argument("WorldConstraints: expected a JSON object");
    }
    WorldConstraints constraints;
    if(json.contains("markerType")) {
        const auto& type = json.at("markerType");
        constraints.markerType = type.is_string() ? markerTypeFromString(type.get<std::string>()) : static_cast<MarkerType>(int32FromJson(type, "markerType"));
    }
    for(const char* key : {"arucoDictionary", "dictionary"}) {
        if(!json.contains(key)) {
            continue;
        }
        const auto& dictionary = json.at(key);
        constraints.arucoDictionary = dictionary.is_string() ? arucoDictionaryFromString(dictionary.get<std::string>())
                                                             : static_cast<ArucoDictionary>(int32FromJson(dictionary, "arucoDictionary"));
        break;
    }
    if(json.contains("lengthUnit")) {
        throw std::invalid_argument("WorldConstraints: \"lengthUnit\" is not supported, every distance and size is in meters");
    }
    for(const auto& entry : json.value("markerDistances", nlohmann::json::array())) {
        MarkerDistance distance;
        if(entry.contains("markers")) {
            const auto markers = markerIdsFromJson(entry.at("markers"));
            if(markers.size() != 2) {
                throw std::invalid_argument("WorldConstraints: \"markers\" of a marker distance needs exactly two ids");
            }
            distance.markerA = markers[0];
            distance.markerB = markers[1];
        } else {
            distance.markerA = markerIdFromJson(entry.at("markerA"));
            distance.markerB = markerIdFromJson(entry.at("markerB"));
        }
        distance.distance = distanceFromJson(entry);
        constraints.markerDistances.push_back(distance);
    }
    for(const auto& entry : json.value("socketDistances", nlohmann::json::array())) {
        MarkerSocketDistance distance;
        distance.marker = markerIdFromJson(entry.at("marker"));
        distance.socket = socketFromJson(entry.at("socket"));
        distance.distance = distanceFromJson(entry);
        constraints.socketDistances.push_back(distance);
    }
    for(const auto& entry : json.value("markerSizes", nlohmann::json::array())) {
        if(!entry.contains("size") || !entry.at("size").is_number()) {
            throw std::invalid_argument("WorldConstraints: every marker size entry needs a numeric \"size\"");
        }
        MarkerSize size;
        size.marker = markerIdFromJson(entry.at("marker"));
        size.size = entry.at("size").get<float>();
        constraints.markerSizes.push_back(size);
    }
    constraints.validate();
    return constraints;
}

WorldConstraints WorldConstraints::fromJsonString(const std::string& json) {
    try {
        return fromJson(nlohmann::json::parse(json));
    } catch(const nlohmann::json::exception& ex) {
        throw std::invalid_argument(std::string("WorldConstraints: malformed JSON: ") + ex.what());
    }
}

WorldConstraints WorldConstraints::fromFile(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if(!stream) {
        throw std::invalid_argument("WorldConstraints: cannot open " + path.string());
    }
    nlohmann::json json;
    try {
        stream >> json;
    } catch(const nlohmann::json::exception& ex) {
        throw std::invalid_argument("WorldConstraints: malformed JSON in " + path.string() + ": " + ex.what());
    }
    return fromJson(json);
}

nlohmann::json WorldConstraints::toJson() const {
    nlohmann::json json;
    json["markerType"] = toString(markerType);
    if(markerType == MarkerType::ARUCO) {
        json["arucoDictionary"] = toString(arucoDictionary);
    }
    auto markerDistancesJson = nlohmann::json::array();
    for(const auto& distance : markerDistances) {
        markerDistancesJson.push_back({{"markers", {distance.markerA, distance.markerB}}, {"distance", distance.distance}});
    }
    json["markerDistances"] = markerDistancesJson;
    auto socketDistancesJson = nlohmann::json::array();
    for(const auto& distance : socketDistances) {
        socketDistancesJson.push_back({{"marker", distance.marker}, {"socket", toString(distance.socket)}, {"distance", distance.distance}});
    }
    json["socketDistances"] = socketDistancesJson;
    auto markerSizesJson = nlohmann::json::array();
    for(const auto& size : markerSizes) {
        markerSizesJson.push_back({{"marker", size.marker}, {"size", size.size}});
    }
    json["markerSizes"] = markerSizesJson;
    return json;
}

namespace {
/// A length given in unit, in meters.
float toMeters(float value, LengthUnit unit) {
    return value * getDistanceUnitScale(LengthUnit::METER, unit);
}
}  // namespace

WorldConstraints::MarkerDistance::MarkerDistance(int32_t markerA, int32_t markerB, float distance, LengthUnit unit)
    : markerA(markerA), markerB(markerB), distance(toMeters(distance, unit)) {}

WorldConstraints::MarkerSocketDistance::MarkerSocketDistance(int32_t marker, CameraBoardSocket socket, float distance, LengthUnit unit)
    : marker(marker), socket(socket), distance(toMeters(distance, unit)) {}

WorldConstraints::MarkerSize::MarkerSize(int32_t marker, float size, LengthUnit unit) : marker(marker), size(toMeters(size, unit)) {}

bool WorldConstraints::empty() const {
    return markerDistances.empty() && socketDistances.empty() && markerSizes.empty();
}

void WorldConstraints::validate() const {
    const auto checkDistance = [](float distance, const std::string& what) {
        if(!std::isfinite(distance) || distance <= 0.0f) {
            throw std::invalid_argument("WorldConstraints: " + what + " must be a positive distance");
        }
    };
    for(const auto& distance : markerDistances) {
        const std::string what = "distance between markers " + std::to_string(distance.markerA) + " and " + std::to_string(distance.markerB);
        if(distance.markerA < 0 || distance.markerB < 0) {
            throw std::invalid_argument("WorldConstraints: " + what + " needs non-negative marker ids");
        }
        if(distance.markerA == distance.markerB) {
            throw std::invalid_argument("WorldConstraints: " + what + " refers to the same marker twice");
        }
        checkDistance(distance.distance, what);
    }
    for(const auto& distance : socketDistances) {
        const std::string what = "distance between marker " + std::to_string(distance.marker) + " and " + toString(distance.socket);
        if(distance.marker < 0) {
            throw std::invalid_argument("WorldConstraints: " + what + " needs a non-negative marker id");
        }
        if(!isKnownSocket(distance.socket)) {
            throw std::invalid_argument("WorldConstraints: " + what + " needs a concrete socket (CAM_A..CAM_J)");
        }
        checkDistance(distance.distance, what);
    }
    std::set<int32_t> sizedMarkers;
    for(const auto& size : markerSizes) {
        const std::string what = "size of marker " + std::to_string(size.marker);
        if(size.marker < 0) {
            throw std::invalid_argument("WorldConstraints: " + what + " needs a non-negative marker id");
        }
        if(!sizedMarkers.insert(size.marker).second) {
            throw std::invalid_argument("WorldConstraints: " + what + " is given twice");
        }
        checkDistance(size.size, what);
    }
    if(toString(markerType) == "UNKNOWN") {
        throw std::invalid_argument("WorldConstraints: unknown markerType " + std::to_string(static_cast<int32_t>(markerType)));
    }
    if(toString(arucoDictionary) == "UNKNOWN") {
        throw std::invalid_argument("WorldConstraints: unknown arucoDictionary " + std::to_string(static_cast<int32_t>(arucoDictionary)));
    }
    if(markerType == MarkerType::LUXONIS) {
        for(const auto id : utility::referencedMarkerIds(*this)) {
            if(id > kLuxonisMarkerIdMax) {
                throw std::invalid_argument("WorldConstraints: Luxonis Marker ids are 0.." + std::to_string(kLuxonisMarkerIdMax) + ", got "
                                            + std::to_string(id));
            }
        }
    }
}

namespace utility {

std::vector<int32_t> referencedMarkerIds(const WorldConstraints& constraints) {
    std::set<int32_t> ids;
    for(const auto& distance : constraints.markerDistances) {
        ids.insert(distance.markerA);
        ids.insert(distance.markerB);
    }
    for(const auto& distance : constraints.socketDistances) {
        ids.insert(distance.marker);
    }
    for(const auto& size : constraints.markerSizes) {
        ids.insert(size.marker);
    }
    return {ids.begin(), ids.end()};
}

std::vector<CameraBoardSocket> referencedSockets(const WorldConstraints& constraints) {
    std::set<CameraBoardSocket> sockets;
    for(const auto& distance : constraints.socketDistances) {
        sockets.insert(distance.socket);
    }
    return {sockets.begin(), sockets.end()};
}

}  // namespace utility
}  // namespace dai
