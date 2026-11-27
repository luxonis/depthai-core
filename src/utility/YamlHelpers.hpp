#pragma once

#include <fmt/format.h>
#include <fmt/std.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>

namespace dai {
namespace utility {

namespace fs = std::filesystem;

inline YAML::Node loadYaml(const fs::path& path) {
    std::ifstream file(path);
    if(!file) {
        throw std::runtime_error(fmt::format("Failed to open yaml file: {}", path));
    }
    return YAML::Load(file);
}

inline void saveYaml(const YAML::Node& node, const fs::path& path) {
    std::ofstream fout(path);
    fout << node;
    fout.close();
}

inline bool isYamlFile(const fs::path& path) {
    auto extension = path.extension();
    return extension == ".yaml" || extension == ".yml";
}

template <typename T>
T yamlGet(const YAML::Node& node, const std::string& key, const T& defaultValue) {
    if(node[key]) {
        return node[key].as<T>();
    }
    return defaultValue;
}

template <typename T>
T yamlGet(const YAML::Node& node, const std::string& key) {
    if(node[key]) {
        return node[key].as<T>();
    }
    throw std::runtime_error("Key not found in yaml: " + key);
}

}  // namespace utility
}  // namespace dai
