#pragma once

#include <yaml-cpp/yaml.h>

#include <fstream>

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
 * @brief Save yaml node to file
 *
 * @param node: YAML node to save
 * @param path: Path to save yaml file
 */
inline void saveYaml(const YAML::Node& node, const std::string& path) {
    std::ofstream fout(path);
    fout << node;
    fout.close();
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
    throw std::runtime_error("Key not found in yaml: " + key);
}

}  // namespace utility
}  // namespace dai