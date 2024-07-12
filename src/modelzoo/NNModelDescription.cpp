#include "depthai/modelzoo/NNModelDescription.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>

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
    auto modelInstanceSlug = utility::yamlGet<std::string>(yamlNode, "model_instance_slug", "");
    auto modelInstanceHash = utility::yamlGet<std::string>(yamlNode, "model_instance_hash", "");
    auto optimizationLevel = utility::yamlGet<std::string>(yamlNode, "optimization_level", "");
    auto compressionLevel = utility::yamlGet<std::string>(yamlNode, "compression_level", "");

    return {modelSlug, platform, modelVersionSlug, modelInstanceSlug, modelInstanceHash, optimizationLevel, compressionLevel};
}

void NNModelDescription::saveToYamlFile(const std::string& yamlPath) const {
    YAML::Node yamlNode;

    // Write REQUIRED parameters
    yamlNode["model_slug"] = modelSlug;
    yamlNode["platform"] = platform;

    // Write OPTIONAL parameters
    if(!modelVersionSlug.empty()) yamlNode["model_version_slug"] = modelVersionSlug;
    if(!modelInstanceSlug.empty()) yamlNode["model_instance_slug"] = modelInstanceSlug;
    if(!modelInstanceHash.empty()) yamlNode["model_instance_hash"] = modelInstanceHash;
    if(!optimizationLevel.empty()) yamlNode["optimization_level"] = optimizationLevel;
    if(!compressionLevel.empty()) yamlNode["compression_level"] = compressionLevel;

    // Write yaml node to file
    utility::saveYaml(yamlNode, yamlPath);
}

std::string NNModelDescription::toString() const {
    std::string out = "NNModelDescription [\n";
    out += "  model_slug: " + modelSlug + "\n";
    out += "  platform: " + platform + "\n";
    out += "  model_version_slug: " + modelVersionSlug + "\n";
    out += "  model_instance_slug: " + modelInstanceSlug + "\n";
    out += "  model_instance_hash: " + modelInstanceHash + "\n";
    out += "  optimization_level: " + optimizationLevel + "\n";
    out += "  compression_level: " + compressionLevel + "\n";
    out += "]";
    return out;
}

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription) {
    os << modelDescription.toString();
    return os;
}

}  // namespace dai