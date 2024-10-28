#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace utility {

std::string jsonDisplay(const nlohmann::json& json, int level, int indent) {
    std::stringstream ss;
    auto doIndent = [&]() {
        for(int i = 0; i < level * indent; ++i) ss << " ";
    };
    switch(json.type()) {
        case nlohmann::detail::value_t::null:
            ss << "null";
            break;
        case nlohmann::detail::value_t::string:
            ss << json.get<std::string>();
            break;
        case nlohmann::detail::value_t::boolean:
            ss << (json.get<bool>() ? "true" : "false");
            break;
        case nlohmann::detail::value_t::number_integer:
            ss << json.get<int>();
            break;
        case nlohmann::detail::value_t::number_unsigned:
            ss << json.get<unsigned int>();
            break;
        case nlohmann::detail::value_t::number_float:
            ss << json.get<float>();
            break;
        case nlohmann::detail::value_t::object:
            for(const auto& [k, v] : json.items()) {
                std::string key = k;
                auto sep = key.find_last_of("::");
                if(sep != std::string::npos) key = key.substr(sep + 1);
                ss << '\n';
                doIndent();
                ss << key << ": " << jsonDisplay(v, level + 1, indent);
            }
            break;
        case nlohmann::detail::value_t::array: {
            auto elements = json.get<std::vector<nlohmann::json>>();
            if(elements.empty())
                ss << "[ ]";
            else {
                ss << '[';
                for(const auto& el : elements) {
                    ss << jsonDisplay(el, level + 1, indent) << ", ";
                }
                ss << '\n';
                doIndent();
                ss << ']';
            }
            break;
        }
        case nlohmann::detail::value_t::binary:
        case nlohmann::detail::value_t::discarded:
            ss << "invalid";
            break;
    }
    if(level == 0) ss << '\n';
    return ss.str();
}

}  // namespace utility
}  // namespace dai
