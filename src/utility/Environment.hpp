#pragma once

#include <spdlog/details/os.h>
#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

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

/**
 * @brief Get environment variable as a specific type
 * @tparam T The type to convert the environment variable to
 * @param var The name of the environment variable
 * @param defaultValue The default value to return if the environment variable is not set
 * @param logger The logger to use for logging
 * @return The environment variable value as the specified type
 */
template <typename T>
T getEnvAs(const std::string& var, T defaultValue, spdlog::logger& logger) {
    // Lock to make sure there is no collision between threads
    std::mutex& mutex = getEnvTypeMutex<T>();
    std::unordered_map<std::string, T>& cache = getEnvTypeCache<T>();
    std::unique_lock<std::mutex> lock(mutex);

    // Return immediately if cached
    if(cache.count(var) > 0) {
        return cache[var];
    }

    // Get environment variable
    std::string value = spdlog::details::os::getenv(var.c_str());

    // Check if we should use the default
    bool useDefault = value.empty();

    // If unspecified, return the default value
    if(useDefault) {
        cache[var] = defaultValue;
        logger.info("Environment variable {} is not set. Using default value: '{}'", var, defaultValue);
    }

    // If specified, convert to the corresponding data type
    if(!useDefault) {
        // Convert to corresponding data type
        try {
            // string
            if constexpr(std::is_same_v<T, std::string>) {
                cache[var] = value;
            }

            // bool
            else if constexpr(std::is_same_v<T, bool>) {
                if(value == "1" || value == "true" || value == "TRUE" || value == "True") {
                    cache[var] = true;
                } else if(value == "0" || value == "false" || value == "FALSE" || value == "False") {
                    cache[var] = false;
                } else {
                    std::ostringstream message;
                    message << "Failed to convert environment variable " << var << " from '" << value << "' to type " << typeid(T).name();
                    message << ". Possible values are '1', 'true', 'TRUE', 'True', '0', 'false', 'FALSE', 'False'";
                    throw std::runtime_error(message.str());
                }
            }

            // anything else
            else {
                std::istringstream iss(value);
                T result;
                iss >> result;

                // Check for failure
                if(iss.fail()) {
                    throw std::runtime_error("Failed to convert environment variable " + var + " from '" + value + "' to type " + typeid(T).name());
                }

                cache[var] = result;
            }

        } catch(const std::runtime_error& e) {
            logger.error("{}. Using default value instead: '{}'", e.what(), defaultValue);
            cache[var] = defaultValue;
        }
    }

    // variable is guaranteed to be in cache by now
    return cache[var];
}

/**
 * @brief Get environment variable as a specific type and check if it is a valid value
 * @tparam T The type to convert the environment variable to
 * @param var The name of the environment variable
 * @param defaultValue The default value to return if the environment variable is not set
 * @param possibleValues The list of possible values for the environment variable
 * @param logger The logger to use for logging
 * @return The environment variable value as the specified type
 */
template <typename T>
T getEnvAsChecked(const std::string& var, T defaultValue, const std::vector<T>& possibleValues, spdlog::logger& logger) {
    // Check that default value is part of possible values
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

        // Log the error
        logger.error(message.str());

        // Override the value in the cache
        std::unique_lock<std::mutex> lock(getEnvTypeMutex<T>());
        std::unordered_map<std::string, T>& cache = getEnvTypeCache<T>();
        cache[var] = defaultValue;

        // use default value
        value = defaultValue;
    }
    return value;
}

/**
 * @brief Get environment variable as a specific type
 * @tparam T The type to convert the environment variable to
 * @param var The name of the environment variable
 * @param defaultValue The default value to return if the environment variable is not set
 * @return The environment variable value as the specified type
 */
template <typename T>
T getEnvAs(const std::string& var, T defaultValue) {
    return getEnvAs<T>(var, defaultValue, Logging::getInstance().logger);
}

/**
 * @brief Get environment variable as a specific type and check if it is a valid value
 * @tparam T The type to convert the environment variable to
 * @param var The name of the environment variable
 * @param defaultValue The default value to return if the environment variable is not set
 * @param possibleValues The list of possible values for the environment variable
 * @return The environment variable value as the specified type
 */
template <typename T>
T getEnvAsChecked(const std::string& var, T defaultValue, const std::vector<T>& possibleValues) {
    return getEnvAsChecked<T>(var, defaultValue, possibleValues, Logging::getInstance().logger);
}

std::vector<std::string> splitList(const std::string& list, const std::string& delimiter);

}  // namespace utility
}  // namespace dai
