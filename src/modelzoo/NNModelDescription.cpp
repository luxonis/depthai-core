#include "depthai/modelzoo/NNModelDescription.hpp"
#include <iostream>

namespace dai {

NNModelDescription NNModelDescription::fromYaml(const std::string& yamlPath) {

    // Parse yaml file
    auto yamlNode = utility::loadYaml(yamlPath);

    // Load required parameters - throws if key not found
    auto name = utility::yamlGet<std::string>(yamlNode, "name");
    auto platform = utility::yamlGet<std::string>(yamlNode, "platform");

    // Load optional parameters - uses default value if key not found
    auto version = utility::yamlGet<std::string>(yamlNode, "version", "defaultVersion");

    return NNModelDescription(name, version, platform);
}

NNModelDescription NNModelDescription::fromParameters(const std::string& name, const std::string& version, const std::string& platform) {
    return NNModelDescription(name, version, platform);
}



}  // namespace dai