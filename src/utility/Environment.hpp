#pragma once

#include <spdlog/details/os.h>
#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#include "Logging.hpp"

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
    static std::mutex mtx;
    static std::unordered_map<std::string, T> cache;
    std::unique_lock<std::mutex> lock(mtx);

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
                cache[var] = value == "1" || value == "true" || value == "TRUE" || value == "True";
            }

            // anything else
            else {
                std::istringstream iss(value);
                T result;
                iss >> result;

                // Check for failure
                if(iss.fail()) {
                    throw std::runtime_error("Failed to convert environment variable " + var);
                }

                cache[var] = result;
            }

        } catch(const std::exception& e) {
            logger.error("Failed to convert environment variable {}: {}. Using default value: {}", var, e.what(), defaultValue);
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
    T value = getEnvAs<T>(var, defaultValue, logger);
    bool isValid = std::find(possibleValues.begin(), possibleValues.end(), value) != possibleValues.end();

    if(!isValid) {
        std::ostringstream message;
        message << "Given value '" << value << "' is not part of the possible values: [";
        for(int i = 0; i < possibleValues.size(); ++i) {
            message << possibleValues[i];
            if(i != possibleValues.size() - 1) {
                message << ", ";
            }
        }
        message << "]";

        // Log the error
        logger.error(message.str());

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
