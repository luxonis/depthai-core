#include "depthai/modelzoo/NNModelDescription.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <sstream>

#include "../utility/YamlHelpers.hpp"

namespace dai {

std::string SlugComponents::merge() const {
    std::ostringstream oss;
    if(!teamName.empty()) {
        oss << teamName << "/";
    }
    oss << modelSlug;
    if(!modelVariantSlug.empty()) {
        oss << ":" << modelVariantSlug;
    }
    if(!modelRef.empty()) {
        oss << ":" << modelRef;
    }
    return oss.str();
}

SlugComponents SlugComponents::split(const std::string& slug) {
    SlugComponents components;
    std::istringstream iss(slug);
    std::string part;
    int partIndex = 0;

    while(std::getline(iss, part, ':')) {
        if(partIndex == 0) {
            // Check if there's a teamId and modelSlug separated by a '/'
            auto slashPos = part.find('/');
            if(slashPos != std::string::npos) {
                components.teamName = part.substr(0, slashPos);
                components.modelSlug = part.substr(slashPos + 1);
            } else {
                components.modelSlug = part;
            }
        } else if(partIndex == 1) {
            components.modelVariantSlug = part;
        } else if(partIndex == 2) {
            components.modelRef = part;
        }
        partIndex++;
    }

    return components;
}

NNModelDescription NNModelDescription::fromYamlFile(const std::string& yamlPath) {
    // Make sure the file exists
    if(!std::filesystem::exists(yamlPath)) throw std::runtime_error("File does not exist: " + yamlPath);

    // Parse yaml file
    auto yamlNode = utility::loadYaml(yamlPath);

    // Load REQUIRED parameters - throws if key not found
    auto model = utility::yamlGet<std::string>(yamlNode, "model");

    // Load OPTIONAL parameters - use default value if key not found
    auto platform = utility::yamlGet<std::string>(yamlNode, "platform", "");
    auto optimizationLevel = utility::yamlGet<std::string>(yamlNode, "optimization_level", "");
    auto compressionLevel = utility::yamlGet<std::string>(yamlNode, "compression_level", "");
    auto snpeVersion = utility::yamlGet<std::string>(yamlNode, "snpe_version", "");
    auto modelPrecisionType = utility::yamlGet<std::string>(yamlNode, "model_precision_type", "");

    return {model, platform, optimizationLevel, compressionLevel, snpeVersion, modelPrecisionType};
}

void NNModelDescription::saveToYamlFile(const std::string& yamlPath) const {
    YAML::Node yamlNode;

    // Write REQUIRED parameters
    yamlNode["model"] = model;

    // Write OPTIONAL parameters
    if(!platform.empty()) yamlNode["platform"] = platform;
    if(!optimizationLevel.empty()) yamlNode["optimization_level"] = optimizationLevel;
    if(!compressionLevel.empty()) yamlNode["compression_level"] = compressionLevel;
    if(!snpeVersion.empty()) yamlNode["snpe_version"] = snpeVersion;
    if(!modelPrecisionType.empty()) yamlNode["model_precision_type"] = modelPrecisionType;

    // Write yaml node to file
    utility::saveYaml(yamlNode, yamlPath);
}

std::string NNModelDescription::toString() const {
    std::string out = "NNModelDescription [\n";
    out += "  model: " + model + "\n";
    out += "  platform: " + platform + "\n";
    out += "  optimization_level: " + optimizationLevel + "\n";
    out += "  compression_level: " + compressionLevel + "\n";
    out += "  snpe_version: " + snpeVersion + "\n";
    out += "  model_precision_type: " + modelPrecisionType + "\n";
    out += "]";
    return out;
}

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription) {
    os << modelDescription.toString();
    return os;
}

}  // namespace dai
