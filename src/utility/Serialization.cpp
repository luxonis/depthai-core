#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace utility {

std::string jsonDisplay(const nlohmann::json& json, int level, int indent) {
    std::stringstream ss;
    auto doIndent = [&]() {
        for(int i = 0; i < level * indent; ++i) ss << " ";
    };
    ss << '\n';
    switch(json.type()) {
        case nlohmann::detail::value_t::null:
            doIndent();
            ss << "null";
            break;
        case nlohmann::detail::value_t::string:
            doIndent();
            ss << json.get<std::string>();
            break;
        case nlohmann::detail::value_t::boolean:
            doIndent();
            ss << (json.get<bool>() ? "true" : "false");
            break;
        case nlohmann::detail::value_t::number_integer:
            doIndent();
            ss << json.get<int>();
            break;
        case nlohmann::detail::value_t::number_unsigned:
            doIndent();
            ss << json.get<unsigned int>();
            break;
        case nlohmann::detail::value_t::number_float:
            doIndent();
            ss << json.get<float>();
            break;
        case nlohmann::detail::value_t::object:
            for(const auto& [k, v] : json.items()) {
                doIndent();
                ss << k << jsonDisplay(v, level + 1, indent) << '\n';
            }
            break;
        case nlohmann::detail::value_t::array: {
            auto elements = json.get<std::vector<nlohmann::json>>();
            doIndent();
            ss << '[';
            for(const auto& el : elements) {
                ss << jsonDisplay(el, level + 1, indent);
            }
            doIndent();
            ss << ']';
            break;
        }
        case nlohmann::detail::value_t::binary:
        case nlohmann::detail::value_t::discarded:
            break;
    }
    if(level == 0) ss << '\n';
    return ss.str();
}

}  // namespace utility
}  // namespace dai
