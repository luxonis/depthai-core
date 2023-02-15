#include "Environment.hpp"

#include <spdlog/details/os.h>
#include <spdlog/spdlog.h>

#include <mutex>
#include <unordered_map>

#include <sstream>

namespace dai {
namespace utility {

static std::mutex mtx;
static std::unordered_map<std::string, std::string> map;

std::string getEnv(const std::string& var) {
    std::unique_lock<std::mutex> lock(mtx);

    if(map.count(var) > 0) {
        return map.at(var);
    }

    auto env = std::getenv(var.c_str());
    std::string value;
    if(env) {
        value = std::string(env);
    } else {
        value = "";
    }
    map[var] = value;

    // Log if env variable is set
    if(!value.empty()) {
        spdlog::debug("Environment '{}' set to '{}'", var, value);
    }

    return value;
}

// inplace convert to lowercase
inline std::string &to_lower_(std::string &str)
{
    std::transform(
        str.begin(), str.end(), str.begin(), [](char ch) { return static_cast<char>((ch >= 'A' && ch <= 'Z') ? ch + ('a' - 'A') : ch); });
    return str;
}

// inplace trim spaces
inline std::string &trim_(std::string &str)
{
    const char *spaces = " \n\r\t";
    str.erase(str.find_last_not_of(spaces) + 1);
    str.erase(0, str.find_first_not_of(spaces));
    return str;
}

// return (name,value) trimmed pair from given "name=value" string.
// return empty string on missing parts
// "key=val" => ("key", "val")
// " key  =  val " => ("key", "val")
// "key=" => ("key", "")
// "val" => ("", "val")

inline std::pair<std::string, std::string> extract_kv_(char sep, const std::string &str)
{
    auto n = str.find(sep);
    std::string k, v;
    if (n == std::string::npos)
    {
        v = str;
    }
    else
    {
        k = str.substr(0, n);
        v = str.substr(n + 1);
    }
    return std::make_pair(trim_(k), trim_(v));
}

// return vector of key/value pairs from sequence of "K1=V1,K2=V2,.."
// "a=AAA,b=BBB,c=CCC,.." => {("a","AAA"),("b","BBB"),("c", "CCC"),...}
inline std::unordered_map<std::string, std::string> extract_key_vals_(const std::string &str)
{
    std::string token;
    std::istringstream token_stream(str);
    std::unordered_map<std::string, std::string> rv{};
    while (std::getline(token_stream, token, ','))
    {
        if (token.empty())
        {
            continue;
        }
        auto kv = extract_kv_('=', token);
        rv[kv.first] = kv.second;
    }
    return rv;
}

void spdlogLoadLevels(const std::string &input)
{
    if (input.empty() || input.size() > 512)
    {
        return;
    }

    auto key_vals = extract_key_vals_(input);
    std::unordered_map<std::string, spdlog::level::level_enum> levels;
    spdlog::level::level_enum global_level = spdlog::level::info;
    bool global_level_found = false;

    for (auto &name_level : key_vals)
    {
        auto &logger_name = name_level.first;
        auto level_name = to_lower_(name_level.second);
        auto level = spdlog::level::from_str(level_name);
        // ignore unrecognized level names
        if (level == spdlog::level::off && level_name != "off")
        {
            continue;
        }
        if (logger_name.empty()) // no logger name indicate global level
        {
            global_level_found = true;
            global_level = level;
        }
        else
        {
            levels[logger_name] = level;
        }
    }

    // spdlog::details::registry::instance().set_levels(std::move(levels), global_level_found ? &global_level : nullptr);
    spdlog::details::registry::instance().set_level(global_level);
}

}  // namespace utility
}  // namespace dai
