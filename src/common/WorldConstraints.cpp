#include "depthai/common/WorldConstraints.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

#include "depthai/common/DepthUnit.hpp"

namespace dai {

namespace {

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

const std::vector<std::pair<MarkerType, const char*>>& markerTypeNames() {
    static const std::vector<std::pair<MarkerType, const char*>> names = {
        {MarkerType::ARUCO, "ARUCO"},
        {MarkerType::LUXONIS, "LUXONIS"},
    };
    return names;
}

std::string upper(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
}

bool isKnownSocket(CameraBoardSocket socket) {
    return socket >= CameraBoardSocket::CAM_A && socket <= CameraBoardSocket::CAM_J;
}

CameraBoardSocket socketFromJson(const nlohmann::json& value) {
    if(value.is_number_integer()) {
        const auto socket = static_cast<CameraBoardSocket>(value.get<int32_t>());
        if(!isKnownSocket(socket)) {
            throw std::invalid_argument("WorldConstraints: unknown socket " + std::to_string(value.get<int32_t>()));
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

LengthUnit lengthUnitFromJson(const nlohmann::json& json) {
    if(!json.contains("lengthUnit")) {
        return LengthUnit::METER;
    }
    static const std::vector<std::pair<const char*, LengthUnit>> names = {
        {"M", LengthUnit::METER},
        {"METER", LengthUnit::METER},
        {"METERS", LengthUnit::METER},
        {"CM", LengthUnit::CENTIMETER},
        {"CENTIMETER", LengthUnit::CENTIMETER},
        {"CENTIMETERS", LengthUnit::CENTIMETER},
        {"MM", LengthUnit::MILLIMETER},
        {"MILLIMETER", LengthUnit::MILLIMETER},
        {"MILLIMETERS", LengthUnit::MILLIMETER},
        {"IN", LengthUnit::INCH},
        {"INCH", LengthUnit::INCH},
        {"INCHES", LengthUnit::INCH},
        {"FT", LengthUnit::FOOT},
        {"FOOT", LengthUnit::FOOT},
        {"FEET", LengthUnit::FOOT},
    };
    const auto unit = json.at("lengthUnit").get<std::string>();
    const auto wanted = upper(unit);
    for(const auto& [name, value] : names) {
        if(wanted == name) {
            return value;
        }
    }
    throw std::invalid_argument("WorldConstraints: unknown lengthUnit \"" + unit + "\"");
}

/// Factor that turns a distance given in the document's lengthUnit into meters.
float lengthUnitToMeters(const nlohmann::json& json) {
    return 1.0f / getLengthUnitMultiplier(lengthUnitFromJson(json));
}

int32_t markerIdFromJson(const nlohmann::json& value) {
    if(!value.is_number_integer() || value.get<int64_t>() < 0) {
        throw std::invalid_argument("WorldConstraints: marker ids must be non-negative integers, got " + value.dump());
    }
    return value.get<int32_t>();
}

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

float distanceFromJson(const nlohmann::json& entry, float toMeters) {
    if(!entry.contains("distance") || !entry.at("distance").is_number()) {
        throw std::invalid_argument("WorldConstraints: every distance entry needs a numeric \"distance\"");
    }
    return entry.at("distance").get<float>() * toMeters;
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
        if(wanted == candidate || wanted == std::string(candidate).substr(5)) {
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
        constraints.markerType = type.is_string() ? markerTypeFromString(type.get<std::string>()) : static_cast<MarkerType>(type.get<int32_t>());
    }
    for(const char* key : {"arucoDictionary", "dictionary"}) {
        if(!json.contains(key)) {
            continue;
        }
        const auto& dictionary = json.at(key);
        constraints.arucoDictionary =
            dictionary.is_string() ? arucoDictionaryFromString(dictionary.get<std::string>()) : static_cast<ArucoDictionary>(dictionary.get<int32_t>());
        break;
    }
    const float toMeters = lengthUnitToMeters(json);
    if(json.contains("markers")) {
        constraints.markerIds = markerIdsFromJson(json.at("markers"));
    }
    if(json.contains("maxObservationSets")) {
        constraints.maxObservationSets = json.at("maxObservationSets").get<uint32_t>();
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
        distance.distanceMeters = distanceFromJson(entry, toMeters);
        constraints.markerDistances.push_back(distance);
    }
    for(const auto& entry : json.value("socketDistances", nlohmann::json::array())) {
        MarkerSocketDistance distance;
        distance.marker = markerIdFromJson(entry.at("marker"));
        distance.socket = socketFromJson(entry.at("socket"));
        distance.distanceMeters = distanceFromJson(entry, toMeters);
        constraints.socketDistances.push_back(distance);
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
    json["lengthUnit"] = "m";
    json["markers"] = markerIds;
    json["maxObservationSets"] = maxObservationSets;
    auto markerDistancesJson = nlohmann::json::array();
    for(const auto& distance : markerDistances) {
        markerDistancesJson.push_back({{"markers", {distance.markerA, distance.markerB}}, {"distance", distance.distanceMeters}});
    }
    json["markerDistances"] = markerDistancesJson;
    auto socketDistancesJson = nlohmann::json::array();
    for(const auto& distance : socketDistances) {
        socketDistancesJson.push_back({{"marker", distance.marker}, {"socket", toString(distance.socket)}, {"distance", distance.distanceMeters}});
    }
    json["socketDistances"] = socketDistancesJson;
    return json;
}

bool WorldConstraints::empty() const {
    return markerDistances.empty() && socketDistances.empty();
}

std::vector<int32_t> WorldConstraints::referencedMarkerIds() const {
    std::set<int32_t> ids(markerIds.begin(), markerIds.end());
    for(const auto& distance : markerDistances) {
        ids.insert(distance.markerA);
        ids.insert(distance.markerB);
    }
    for(const auto& distance : socketDistances) {
        ids.insert(distance.marker);
    }
    return {ids.begin(), ids.end()};
}

std::vector<CameraBoardSocket> WorldConstraints::referencedSockets() const {
    std::set<CameraBoardSocket> sockets;
    for(const auto& distance : socketDistances) {
        sockets.insert(distance.socket);
    }
    return {sockets.begin(), sockets.end()};
}

void WorldConstraints::validate() const {
    const auto markerKnown = [&](int32_t id) { return markerIds.empty() || std::find(markerIds.begin(), markerIds.end(), id) != markerIds.end(); };
    const auto checkDistance = [](float distance, const std::string& what) {
        if(!std::isfinite(distance) || distance <= 0.0f) {
            throw std::invalid_argument("WorldConstraints: " + what + " must be a positive distance");
        }
    };
    for(const auto& id : markerIds) {
        if(id < 0) {
            throw std::invalid_argument("WorldConstraints: marker ids must be non-negative");
        }
    }
    for(const auto& distance : markerDistances) {
        const std::string what = "distance between markers " + std::to_string(distance.markerA) + " and " + std::to_string(distance.markerB);
        if(distance.markerA < 0 || distance.markerB < 0) {
            throw std::invalid_argument("WorldConstraints: " + what + " needs non-negative marker ids");
        }
        if(distance.markerA == distance.markerB) {
            throw std::invalid_argument("WorldConstraints: " + what + " refers to the same marker twice");
        }
        if(!markerKnown(distance.markerA) || !markerKnown(distance.markerB)) {
            throw std::invalid_argument("WorldConstraints: " + what + " refers to a marker outside \"markers\"");
        }
        checkDistance(distance.distanceMeters, what);
    }
    for(const auto& distance : socketDistances) {
        const std::string what = "distance between marker " + std::to_string(distance.marker) + " and " + toString(distance.socket);
        if(distance.marker < 0) {
            throw std::invalid_argument("WorldConstraints: " + what + " needs a non-negative marker id");
        }
        if(!isKnownSocket(distance.socket)) {
            throw std::invalid_argument("WorldConstraints: " + what + " needs a concrete socket (CAM_A..CAM_J)");
        }
        if(!markerKnown(distance.marker)) {
            throw std::invalid_argument("WorldConstraints: " + what + " refers to a marker outside \"markers\"");
        }
        checkDistance(distance.distanceMeters, what);
    }
    if(maxObservationSets == 0) {
        throw std::invalid_argument("WorldConstraints: maxObservationSets must be at least 1");
    }
    if(toString(markerType) == "UNKNOWN") {
        throw std::invalid_argument("WorldConstraints: unknown markerType " + std::to_string(static_cast<int32_t>(markerType)));
    }
}

}  // namespace dai
