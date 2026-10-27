#include "depthai/common/ArucoMarkerConstraints.hpp"

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

std::string upper(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
}

CameraBoardSocket socketFromJson(const nlohmann::json& value) {
    if(value.is_number_integer()) {
        return static_cast<CameraBoardSocket>(value.get<int32_t>());
    }
    if(!value.is_string()) {
        throw std::invalid_argument("ArucoMarkerConstraints: a socket must be a name such as \"CAM_A\"");
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
    throw std::invalid_argument("ArucoMarkerConstraints: unknown socket \"" + value.get<std::string>() + "\"");
}

float lengthUnitToMeters(const nlohmann::json& json) {
    if(!json.contains("lengthUnit")) {
        return 1.0f;
    }
    const auto unit = json.at("lengthUnit").get<std::string>();
    const auto lowered = [&] {
        std::string text = unit;
        std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return text;
    }();
    if(lowered == "m" || lowered == "meter" || lowered == "meters") return 1.0f;
    if(lowered == "cm" || lowered == "centimeter" || lowered == "centimeters") return 0.01f;
    if(lowered == "mm" || lowered == "millimeter" || lowered == "millimeters") return 0.001f;
    if(lowered == "in" || lowered == "inch" || lowered == "inches") return 1.0f / getLengthUnitMultiplier(LengthUnit::INCH);
    if(lowered == "ft" || lowered == "foot" || lowered == "feet") return 1.0f / getLengthUnitMultiplier(LengthUnit::FOOT);
    throw std::invalid_argument("ArucoMarkerConstraints: unknown lengthUnit \"" + unit + "\"");
}

float distanceFromJson(const nlohmann::json& entry, float toMeters) {
    if(!entry.contains("distance") || !entry.at("distance").is_number()) {
        throw std::invalid_argument("ArucoMarkerConstraints: every distance entry needs a numeric \"distance\"");
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
    throw std::invalid_argument("ArucoMarkerConstraints: unknown dictionary \"" + name + "\"");
}

ArucoMarkerConstraints ArucoMarkerConstraints::fromJson(const nlohmann::json& json) {
    if(!json.is_object()) {
        throw std::invalid_argument("ArucoMarkerConstraints: expected a JSON object");
    }
    ArucoMarkerConstraints constraints;
    if(json.contains("dictionary")) {
        const auto& dictionary = json.at("dictionary");
        constraints.dictionary =
            dictionary.is_string() ? arucoDictionaryFromString(dictionary.get<std::string>()) : static_cast<ArucoDictionary>(dictionary.get<int32_t>());
    }
    const float toMeters = lengthUnitToMeters(json);
    if(json.contains("markers")) {
        constraints.markerIds = json.at("markers").get<std::vector<int32_t>>();
    }
    if(json.contains("maxObservationSets")) {
        constraints.maxObservationSets = json.at("maxObservationSets").get<uint32_t>();
    }
    for(const auto& entry : json.value("markerDistances", nlohmann::json::array())) {
        MarkerDistance distance;
        if(entry.contains("markers")) {
            const auto markers = entry.at("markers").get<std::vector<int32_t>>();
            if(markers.size() != 2) {
                throw std::invalid_argument("ArucoMarkerConstraints: \"markers\" of a marker distance needs exactly two ids");
            }
            distance.markerA = markers[0];
            distance.markerB = markers[1];
        } else {
            distance.markerA = entry.at("markerA").get<int32_t>();
            distance.markerB = entry.at("markerB").get<int32_t>();
        }
        distance.distanceMeters = distanceFromJson(entry, toMeters);
        constraints.markerDistances.push_back(distance);
    }
    for(const auto& entry : json.value("socketDistances", nlohmann::json::array())) {
        MarkerSocketDistance distance;
        distance.marker = entry.at("marker").get<int32_t>();
        distance.socket = socketFromJson(entry.at("socket"));
        distance.distanceMeters = distanceFromJson(entry, toMeters);
        constraints.socketDistances.push_back(distance);
    }
    constraints.validate();
    return constraints;
}

ArucoMarkerConstraints ArucoMarkerConstraints::fromJsonString(const std::string& json) {
    return fromJson(nlohmann::json::parse(json));
}

ArucoMarkerConstraints ArucoMarkerConstraints::fromFile(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if(!stream) {
        throw std::invalid_argument("ArucoMarkerConstraints: cannot open " + path.string());
    }
    nlohmann::json json;
    stream >> json;
    return fromJson(json);
}

nlohmann::json ArucoMarkerConstraints::toJson() const {
    nlohmann::json json;
    json["dictionary"] = toString(dictionary);
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

bool ArucoMarkerConstraints::empty() const {
    return markerDistances.empty() && socketDistances.empty();
}

std::vector<int32_t> ArucoMarkerConstraints::referencedMarkerIds() const {
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

std::vector<CameraBoardSocket> ArucoMarkerConstraints::referencedSockets() const {
    std::set<CameraBoardSocket> sockets;
    for(const auto& distance : socketDistances) {
        sockets.insert(distance.socket);
    }
    return {sockets.begin(), sockets.end()};
}

void ArucoMarkerConstraints::validate() const {
    const auto markerKnown = [&](int32_t id) { return markerIds.empty() || std::find(markerIds.begin(), markerIds.end(), id) != markerIds.end(); };
    const auto checkDistance = [](float distance, const std::string& what) {
        if(!std::isfinite(distance) || distance <= 0.0f) {
            throw std::invalid_argument("ArucoMarkerConstraints: " + what + " must be a positive distance");
        }
    };
    for(const auto& id : markerIds) {
        if(id < 0) {
            throw std::invalid_argument("ArucoMarkerConstraints: marker ids must be non-negative");
        }
    }
    for(const auto& distance : markerDistances) {
        const std::string what = "distance between markers " + std::to_string(distance.markerA) + " and " + std::to_string(distance.markerB);
        if(distance.markerA == distance.markerB) {
            throw std::invalid_argument("ArucoMarkerConstraints: " + what + " refers to the same marker twice");
        }
        if(!markerKnown(distance.markerA) || !markerKnown(distance.markerB)) {
            throw std::invalid_argument("ArucoMarkerConstraints: " + what + " refers to a marker outside \"markers\"");
        }
        checkDistance(distance.distanceMeters, what);
    }
    for(const auto& distance : socketDistances) {
        const std::string what = "distance between marker " + std::to_string(distance.marker) + " and " + toString(distance.socket);
        if(distance.socket == CameraBoardSocket::AUTO) {
            throw std::invalid_argument("ArucoMarkerConstraints: " + what + " needs a concrete socket");
        }
        if(!markerKnown(distance.marker)) {
            throw std::invalid_argument("ArucoMarkerConstraints: " + what + " refers to a marker outside \"markers\"");
        }
        checkDistance(distance.distanceMeters, what);
    }
    if(maxObservationSets == 0) {
        throw std::invalid_argument("ArucoMarkerConstraints: maxObservationSets must be at least 1");
    }
}

}  // namespace dai
