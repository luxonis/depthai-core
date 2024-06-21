#pragma once

#include <yaml-cpp/yaml.h>

namespace dai {
namespace utility {

/**
 * @brief Load yaml file
 *
 * @param path: Path to yaml file
 * @return YAML::Node: Parsed yaml file as node
 */
inline YAML::Node loadYaml(const std::string& path) {
    return YAML::LoadFile(path);
}

/**
 * @brief Check if file is yaml file
 *
 * @param path: Path to file
 * @return bool: True if file is yaml file
 */
inline bool isYamlFile(const std::string& path) {
    std::string extension = path.substr(path.find_last_of(".") + 1);
    return extension == "yaml" || extension == "yml";
}

/**
 * @brief Get value from yaml node. If key not found, return default value
 *
 * @tparam T: Type of value to get
 * @param node: YAML node
 * @param key: Key to get value from
 * @param defaultValue: Default value if key not found
 * @return T: Value from yaml node
 */
template <typename T>
T yamlGet(const YAML::Node& node, const std::string& key, const T& defaultValue) {
    if(node[key]) {
        return node[key].as<T>();
    }
    return defaultValue;
}

/**
 * @brief Get value from yaml node. If key not found, throw runtime error
 *
 * @tparam T: Type of value to get
 * @param node: YAML node
 * @param key: Key to get value from
 * @return T: Value from yaml node
 */
template <typename T>
T yamlGet(const YAML::Node& node, const std::string& key) {
    if(node[key]) {
        return node[key].as<T>();
    }
    std::string errorMessage = "Key not found in yaml: " + key;
    throw std::runtime_error(errorMessage);
}

}  // namespace utility
}  // namespace dai