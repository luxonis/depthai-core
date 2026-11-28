#pragma once

#include <fmt/std.h>
#include <spdlog/details/os.h>
#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#ifdef _WIN32
    #include <windows.h>
#endif

#include "Logging.hpp"

namespace {
template <typename T>
std::mutex& getEnvTypeMutex() {
    static std::mutex mtx;
    return mtx;
}

template <typename T>
std::unordered_map<std::string, T>& getEnvTypeCache() {
    static std::unordered_map<std::string, T> cache;
    return cache;
}

template <typename T>
std::string vec2str(const std::vector<T>& vec) {
    std::ostringstream oss;
    oss << "[";
    for(size_t i = 0; i < vec.size(); ++i) {
        oss << "'" << vec[i] << "'";
    }
    oss << "]";
    return oss.str();
}
}  // namespace

namespace dai {
namespace utility {

template <typename T>
T getEnvAs(const std::string& var, T defaultValue, spdlog::logger& logger, bool cache = true) {
    std::mutex& mutex = getEnvTypeMutex<T>();
    std::unordered_map<std::string, T>& varCache = getEnvTypeCache<T>();
    std::unique_lock<std::mutex> lock(mutex);

    if(cache && varCache.count(var) > 0) {
        return varCache[var];
    }

    T returnValue;

    std::string value = spdlog::details::os::getenv(var.c_str());

    bool useDefault = value.empty();

    if(useDefault) {
        returnValue = defaultValue;
        logger.info("Environment variable {} is not set. Using default value: '{}'", var, defaultValue);
    }

    if(!useDefault) {
        try {
            if constexpr(std::is_same_v<T, std::string>) {
                returnValue = value;
            }

            else if constexpr(std::is_same_v<T, std::filesystem::path>) {
                returnValue = std::filesystem::path(value);
            }

            else if constexpr(std::is_same_v<T, bool>) {
                std::transform(value.begin(), value.end(), value.begin(), ::tolower);
                if(value == "1" || value == "true" || value == "t" || value == "on") {
                    returnValue = true;
                } else if(value == "0" || value == "false" || value == "f" || value == "off") {
                    returnValue = false;
                } else {
                    std::ostringstream message;
                    message << "Failed to convert environment variable " << var << " from '" << value << "' to type " << typeid(T).name();
                    message << ". Possible values are '1', 'true', 't', 'on', '0', 'false', 'f', 'off'";
                    throw std::runtime_error(message.str());
                }
            }

            else {
                std::istringstream iss(value);
                T result;
                iss >> result;

                if(iss.fail()) {
                    throw std::runtime_error("Failed to convert environment variable " + var + " from '" + value + "' to type " + typeid(T).name());
                }

                returnValue = result;
            }

        } catch(const std::runtime_error& e) {
            logger.error("{}. Using default value instead: '{}'", e.what(), defaultValue);
            returnValue = defaultValue;
        }
    }

    if(cache) {
        varCache[var] = returnValue;
    }
    return returnValue;
}

template <typename T>
T getEnvAsChecked(const std::string& var, T defaultValue, const std::vector<T>& possibleValues, spdlog::logger& logger, bool cache = true) {
    bool isDefaultValueValid = std::find(possibleValues.begin(), possibleValues.end(), defaultValue) != possibleValues.end();
    if(!isDefaultValueValid) {
        std::ostringstream message;
        message << "Default value '" << defaultValue << "' is not part of the possible values: " << vec2str(possibleValues);
        throw std::runtime_error(message.str());
    }

    T value = getEnvAs<T>(var, defaultValue, logger);
    bool isValid = std::find(possibleValues.begin(), possibleValues.end(), value) != possibleValues.end();

    if(!isValid) {
        std::ostringstream message;
        message << "Given value '" << value << "' is not part of the possible values: " << vec2str(possibleValues);
        message << ". Using default value instead: '" << defaultValue << "'";

        logger.error(message.str());

        if(cache) {
            std::unique_lock<std::mutex> lock(getEnvTypeMutex<T>());
            std::unordered_map<std::string, T>& varCache = getEnvTypeCache<T>();
            varCache[var] = defaultValue;
        }

        value = defaultValue;
    }
    return value;
}

template <typename T>
T getEnvAs(const std::string& var, T defaultValue, bool cache = true) {
    return getEnvAs<T>(var, defaultValue, Logging::getInstance().logger, cache);
}

template <typename T>
T getEnvAsChecked(const std::string& var, T defaultValue, const std::vector<T>& possibleValues, bool cache = true) {
    return getEnvAsChecked<T>(var, defaultValue, possibleValues, Logging::getInstance().logger, cache);
}

inline bool isEnvSet(const std::string& var) {
    return !spdlog::details::os::getenv(var.c_str()).empty();
}

inline void setEnv(const std::string& var, const std::string& value, bool overwrite = true) {
    if(isEnvSet(var) && !overwrite) {
        return;
    }

#ifdef _WIN32
    SetEnvironmentVariableA(var.c_str(), value.c_str());
    _putenv_s(var.c_str(), value.c_str());
#else
    setenv(var.c_str(), value.c_str(), 1);
#endif
}

inline void unsetEnv(const std::string& var) {
#ifdef _WIN32
    SetEnvironmentVariableA(var.c_str(), nullptr);
    _putenv_s(var.c_str(), "");
#else
    unsetenv(var.c_str());
#endif
}

std::vector<std::string> splitList(const std::string& list, const std::string& delimiter);

}  // namespace utility
}  // namespace dai
