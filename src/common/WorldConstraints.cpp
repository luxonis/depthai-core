#include "depthai/common/WorldConstraints.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

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

/// Number of marker ids in a dictionary, 0 for a value outside the enumeration.
int32_t dictionaryMarkerCount(ArucoDictionary dictionary) {
    switch(dictionary) {
        case ArucoDictionary::DICT_4X4_50:
        case ArucoDictionary::DICT_5X5_50:
        case ArucoDictionary::DICT_6X6_50:
        case ArucoDictionary::DICT_7X7_50:
            return 50;
        case ArucoDictionary::DICT_4X4_100:
        case ArucoDictionary::DICT_5X5_100:
        case ArucoDictionary::DICT_6X6_100:
        case ArucoDictionary::DICT_7X7_100:
            return 100;
        case ArucoDictionary::DICT_4X4_250:
        case ArucoDictionary::DICT_5X5_250:
        case ArucoDictionary::DICT_6X6_250:
        case ArucoDictionary::DICT_7X7_250:
            return 250;
        case ArucoDictionary::DICT_4X4_1000:
        case ArucoDictionary::DICT_5X5_1000:
        case ArucoDictionary::DICT_6X6_1000:
        case ArucoDictionary::DICT_7X7_1000:
            return 1000;
        case ArucoDictionary::DICT_ARUCO_ORIGINAL:
            return 1024;
        case ArucoDictionary::DICT_APRILTAG_16h5:
            return 30;
        case ArucoDictionary::DICT_APRILTAG_25h9:
            return 35;
        case ArucoDictionary::DICT_APRILTAG_36h10:
            return 2320;
        case ArucoDictionary::DICT_APRILTAG_36h11:
            return 587;
    }
    return 0;
}

/// Throws std::invalid_argument unless value is an object whose keys are all among allowed. what names the object in the message.
void requireObjectWithKeys(const nlohmann::json& value, const std::string& what, const std::vector<std::string>& allowed) {
    if(!value.is_object()) {
        throw std::invalid_argument("WorldConstraints: " + what + " must be a JSON object, got " + value.dump());
    }
    for(const auto& item : value.items()) {
        if(std::find(allowed.begin(), allowed.end(), item.key()) != allowed.end()) {
            continue;
        }
        std::string accepted;
        for(const auto& key : allowed) {
            accepted += (accepted.empty() ? "\"" : ", \"") + key + "\"";
        }
        throw std::invalid_argument("WorldConstraints: unknown key \"" + item.key() + "\" in " + what + ", the accepted keys are " + accepted);
    }
}

/// The member key of entry, which must be present. what names the entry in the message.
const nlohmann::json& requiredMember(const nlohmann::json& entry, const char* key, const std::string& what) {
    const auto it = entry.find(key);
    if(it == entry.end()) {
        throw std::invalid_argument("WorldConstraints: " + what + " needs \"" + key + "\", got " + entry.dump());
    }
    return *it;
}

/// The entries of the list named key; an absent list is empty. Throws std::invalid_argument for anything but an array.
std::vector<nlohmann::json> listFromJson(const nlohmann::json& json, const char* key) {
    const auto it = json.find(key);
    if(it == json.end()) {
        return {};
    }
    if(!it->is_array()) {
        throw std::invalid_argument(std::string("WorldConstraints: \"") + key + "\" must be an array of entries, got " + it->dump());
    }
    return {it->begin(), it->end()};
}

/// A socket given by its name, letter or enumerator number. Throws std::invalid_argument otherwise.
CameraBoardSocket socketFromJson(const nlohmann::json& value) {
    if(value.is_number_integer()) {
        const auto socket = static_cast<CameraBoardSocket>(int32FromJson(value, "socket"));
        if(isKnownSocket(socket)) return socket;
    } else if(value.is_string()) {
        auto name = upper(value.get<std::string>());
        if(name.size() == 1) name = "CAM_" + name;
        for(int socket = static_cast<int>(CameraBoardSocket::CAM_A); socket <= static_cast<int>(CameraBoardSocket::CAM_J); ++socket) {
            if(toString(static_cast<CameraBoardSocket>(socket)) == name) return static_cast<CameraBoardSocket>(socket);
        }
    }
    throw std::invalid_argument("WorldConstraints: a socket must name a concrete socket (CAM_A..CAM_J), got " + value.dump());
}

/// A marker id, which must be a non-negative integer. Throws std::invalid_argument otherwise.
int32_t markerIdFromJson(const nlohmann::json& value) {
    if(!value.is_number_integer() || (!value.is_number_unsigned() && value.get<int64_t>() < 0)) {
        throw std::invalid_argument("WorldConstraints: marker ids must be non-negative integers, got " + value.dump());
    }
    return int32FromJson(value, "marker id");
}

/// A length in meters given as a JSON number. Throws std::invalid_argument for anything else and for a number
/// outside the range of float; validate() checks the sign.
float lengthFromJson(const nlohmann::json& value, const std::string& what) {
    if(!value.is_number()) {
        throw std::invalid_argument("WorldConstraints: " + what + " must be a number of meters, got " + value.dump());
    }
    const auto length = value.get<double>();
    if(!std::isfinite(length) || std::abs(length) > static_cast<double>(std::numeric_limits<float>::max())) {
        throw std::invalid_argument("WorldConstraints: " + what + " is out of range: " + value.dump());
    }
    return static_cast<float>(length);
}

/// Parses text holding exactly one JSON document. A key given twice in the same object is an error: nlohmann would keep
/// the last value and drop the other one silently.
template <typename Input>
nlohmann::json parseDocument(Input&& input) {
    std::vector<std::set<std::string>> keysOfOpenObjects;
    const nlohmann::json::parser_callback_t rejectRepeatedKeys = [&keysOfOpenObjects](int, nlohmann::json::parse_event_t event, nlohmann::json& parsed) {
        if(event == nlohmann::json::parse_event_t::object_start) {
            keysOfOpenObjects.emplace_back();
        } else if(event == nlohmann::json::parse_event_t::object_end) {
            keysOfOpenObjects.pop_back();
        } else if(event == nlohmann::json::parse_event_t::key && !keysOfOpenObjects.back().insert(parsed.get<std::string>()).second) {
            throw std::invalid_argument("WorldConstraints: the key " + parsed.dump() + " is given twice in the same object");
        }
        return true;
    };
    return nlohmann::json::parse(std::forward<Input>(input), rejectRepeatedKeys);
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
    for(const auto& [value, candidate] : markerTypeNames()) {
        if(upper(name) == upper(candidate)) {
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
    // Reject typos while preserving the input forms accepted by earlier SDK versions.
    if(json.is_object() && json.contains("lengthUnit")) {
        throw std::invalid_argument("WorldConstraints: \"lengthUnit\" is not supported, every distance and size is in meters");
    }
    requireObjectWithKeys(json, "the constraints", {"markerType", "arucoDictionary", "dictionary", "markerDistances", "socketDistances", "markerSizes"});
    WorldConstraints constraints;
    if(json.contains("markerType")) {
        const auto& type = json.at("markerType");
        constraints.markerType = type.is_string() ? markerTypeFromString(type.get<std::string>()) : static_cast<MarkerType>(int32FromJson(type, "markerType"));
    }
    std::optional<ArucoDictionary> dictionary;
    for(const char* key : {"arucoDictionary", "dictionary"}) {
        if(!json.contains(key)) continue;
        if(constraints.markerType != MarkerType::ARUCO) {
            throw std::invalid_argument("WorldConstraints: arucoDictionary applies to markerType aruco only");
        }
        const auto& value = json.at(key);
        const auto parsed = value.is_string() ? arucoDictionaryFromString(value.get<std::string>()) : static_cast<ArucoDictionary>(int32FromJson(value, key));
        if(dictionary && *dictionary != parsed) {
            throw std::invalid_argument("WorldConstraints: dictionary and arucoDictionary disagree");
        }
        dictionary = parsed;
    }
    if(dictionary) constraints.arucoDictionary = *dictionary;
    for(const auto& entry : listFromJson(json, "markerDistances")) {
        const std::string what = "a \"markerDistances\" entry";
        requireObjectWithKeys(entry, what, {"markers", "markerA", "markerB", "distance"});
        MarkerDistance distance;
        if(entry.contains("markers")) {
            const auto& markers = entry.at("markers");
            if(!markers.is_array() || markers.size() != 2) {
                throw std::invalid_argument("WorldConstraints: markers must be an array of two marker ids, got " + markers.dump());
            }
            distance.markerA = markerIdFromJson(markers[0]);
            distance.markerB = markerIdFromJson(markers[1]);
            if((entry.contains("markerA") && markerIdFromJson(entry.at("markerA")) != distance.markerA)
               || (entry.contains("markerB") && markerIdFromJson(entry.at("markerB")) != distance.markerB)) {
                throw std::invalid_argument("WorldConstraints: markers and markerA / markerB disagree");
            }
        } else {
            distance.markerA = markerIdFromJson(requiredMember(entry, "markerA", what));
            distance.markerB = markerIdFromJson(requiredMember(entry, "markerB", what));
        }
        distance.distance = lengthFromJson(requiredMember(entry, "distance", what), "distance of " + what);
        constraints.markerDistances.push_back(distance);
    }
    for(const auto& entry : listFromJson(json, "socketDistances")) {
        const std::string what = "a \"socketDistances\" entry";
        requireObjectWithKeys(entry, what, {"marker", "socket", "distance"});
        MarkerSocketDistance distance;
        distance.marker = markerIdFromJson(requiredMember(entry, "marker", what));
        distance.socket = socketFromJson(requiredMember(entry, "socket", what));
        distance.distance = lengthFromJson(requiredMember(entry, "distance", what), "\"distance\" of " + what);
        constraints.socketDistances.push_back(distance);
    }
    for(const auto& entry : listFromJson(json, "markerSizes")) {
        const std::string what = "a \"markerSizes\" entry";
        requireObjectWithKeys(entry, what, {"marker", "size"});
        MarkerSize size;
        size.marker = markerIdFromJson(requiredMember(entry, "marker", what));
        size.size = lengthFromJson(requiredMember(entry, "size", what), "\"size\" of " + what);
        constraints.markerSizes.push_back(size);
    }
    constraints.validate();
    return constraints;
}

WorldConstraints WorldConstraints::fromJsonString(const std::string& json) {
    try {
        return fromJson(parseDocument(json));
    } catch(const nlohmann::json::exception& ex) {
        throw std::invalid_argument(std::string("WorldConstraints: malformed JSON: ") + ex.what());
    }
}

WorldConstraints WorldConstraints::fromFile(const std::filesystem::path& path) {
    std::error_code error;
    if(!std::filesystem::is_regular_file(path, error)) {
        throw std::invalid_argument("WorldConstraints: " + path.string() + " is not a file");
    }
    std::ifstream stream(path);
    if(!stream) {
        throw std::invalid_argument("WorldConstraints: cannot open " + path.string());
    }
    nlohmann::json json;
    try {
        json = parseDocument(stream);
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
            throw std::invalid_argument("WorldConstraints: " + what + " must be a positive, finite number of meters, got " + std::to_string(distance));
        }
    };
    std::set<std::pair<int32_t, int32_t>> measuredPairs;
    for(const auto& distance : markerDistances) {
        const std::string what = "distance between markers " + std::to_string(distance.markerA) + " and " + std::to_string(distance.markerB);
        if(!measuredPairs.insert(std::minmax(distance.markerA, distance.markerB)).second) {
            throw std::invalid_argument("WorldConstraints: " + what + " is given twice");
        }
        if(distance.markerA < 0 || distance.markerB < 0) {
            throw std::invalid_argument("WorldConstraints: " + what + " needs non-negative marker ids");
        }
        if(distance.markerA == distance.markerB) {
            throw std::invalid_argument("WorldConstraints: " + what + " refers to the same marker twice");
        }
        checkDistance(distance.distance, what);
    }
    std::set<std::pair<int32_t, CameraBoardSocket>> measuredSockets;
    for(const auto& distance : socketDistances) {
        const std::string what = "distance between marker " + std::to_string(distance.marker) + " and " + toString(distance.socket);
        if(!measuredSockets.insert({distance.marker, distance.socket}).second) {
            throw std::invalid_argument("WorldConstraints: " + what + " is given twice");
        }
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
    if(markerType == MarkerType::ARUCO) {
        const auto count = dictionaryMarkerCount(arucoDictionary);
        for(const auto id : utility::referencedMarkerIds(*this)) {
            if(id >= count) {
                throw std::invalid_argument("WorldConstraints: " + toString(arucoDictionary) + " has marker ids 0.." + std::to_string(count - 1) + ", got "
                                            + std::to_string(id));
            }
        }
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
