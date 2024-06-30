#include "depthai/modelzoo/NNModelDescription.hpp"

#include <iostream>

#include <yaml-cpp/yaml.h>
#include "../utility/YamlHelpers.hpp"

namespace dai {

NNModelDescription NNModelDescription::fromYamlFile(const std::string& yamlPath) {
    // Make sure the file exists
    if(!std::filesystem::exists(yamlPath)) throw std::runtime_error("File does not exist: " + yamlPath);

    // Parse yaml file
    auto yamlNode = utility::loadYaml(yamlPath);

    // Load REQUIRED parameters - throws if key not found
    auto modelSlug = utility::yamlGet<std::string>(yamlNode, "model_slug");
    auto platform = utility::yamlGet<std::string>(yamlNode, "platform");

    // Load OPTIONAL parameters - use default value if key not found
    auto modelVersionSlug = utility::yamlGet<std::string>(yamlNode, "model_version_slug", "");

    return {modelSlug, platform, modelVersionSlug};
}

void NNModelDescription::saveToYamlFile(const std::string& yamlPath) const {
    YAML::Node yamlNode;

    // Write REQUIRED parameters
    yamlNode["model_slug"] = modelSlug;
    yamlNode["platform"] = platform;

    // Write OPTIONAL parameters
    if(!modelVersionSlug.empty()) yamlNode["model_version_slug"] = modelVersionSlug;

    // Write yaml node to file
    utility::saveYaml(yamlNode, yamlPath);
}

std::string NNModelDescription::toString() const {
    std::string out = "NNModelDescription [\n";
    out += "  model_slug: " + modelSlug + "\n";
    out += "  platform: " + platform + "\n";
    out += "  model_version_slug: " + modelVersionSlug + "\n";
    out += "]";
    return out;
}

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription) {
    os << modelDescription.toString();
    return os;
}

}  // namespace dai