#include "Environment.hpp"

#include <spdlog/details/os.h>
#include <spdlog/spdlog.h>

#include <mutex>
#include <unordered_map>

// project
#include <utility/Logging.hpp>

namespace dai {
namespace utility {

std::vector<std::string> splitList(const std::string& list, const std::string& delimiter) {
    std::vector<std::string> result;
    if(list.empty()) {
        return result;  // Return an empty vector if the input string is empty
    }
    size_t pos = 0;
    size_t end = 0;
    while((end = list.find(delimiter, pos)) != std::string::npos) {
        result.push_back(list.substr(pos, end - pos));
        pos = end + delimiter.size();
    }
    result.push_back(list.substr(pos));

    // Strip down whitespace from each element
    for(auto& elem : result) {
        size_t start = 0;
        while(start < elem.size() && std::isspace(elem[start])) {
            start++;
        }
        size_t end = elem.size();
        while(end > 0 && std::isspace(elem[end - 1])) {
            end--;
        }
        elem = elem.substr(start, end - start);
    }
    return result;
}

}  // namespace utility
}  // namespace dai
