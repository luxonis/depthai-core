#pragma once

#include <string>

namespace dai {
namespace gate {

inline std::string toUpperCase(const std::string& input) {
    std::string result = input;

    for(char& c : result) {
        c = std::toupper(c);
    }

    return result;
}

inline std::string toLowerCase(const std::string& input) {
    std::string result = input;

    for(char& c : result) {
        c = std::tolower(c);
    }

    return result;
}

}  // namespace gate
}  // namespace dai
