#include "depthai/modelzoo/NNModelDescription.hpp"

#include <iostream>

namespace dai {

NNModelDescription NNModelDescription::fromYaml(const std::string& yamlPath) {
    // Make sure the file exists
    if(!std::filesystem::exists(yamlPath)) throw std::runtime_error("File does not exist: " + yamlPath);

    // Parse yaml file
    auto yamlNode = utility::loadYaml(yamlPath);

    // Load REQUIRED parameters - throws if key not found
    auto modelSlug = utility::yamlGet<std::string>(yamlNode, "model_slug");
    auto platform = utility::yamlGet<std::string>(yamlNode, "platform");

    // Load OPTIONAL parameters - use default value if key not found
    auto modelInstanceSlug = utility::yamlGet<std::string>(yamlNode, "model_instance_slug", "");

    return NNModelDescription(modelSlug, platform, modelInstanceSlug);
}

NNModelDescription NNModelDescription::fromParameters(const std::string& modelSlug, const std::string& platform, const std::string& modelInstanceSlug) {
    return NNModelDescription(modelSlug, platform, modelInstanceSlug);
}

NNModelDescription NNModelDescription::fromParameters(const std::string& modelSlug, const Platform platform, const std::string& modelInstanceSlug) {
    return fromParameters(modelSlug, platform2string(platform), modelInstanceSlug);
}

std::string NNModelDescription::toString() const {
    std::string out = "NNModelDescription[\n";
    out += "\tmodel_slug: " + modelSlug + "\n";
    out += "\tplatform: " + platform + "\n";
    out += "\tmodel_instance_slug: " + modelInstanceSlug + "\n";
    out += "]";
    return out;
}

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription) {
    os << modelDescription.toString();
    return os;
}

}  // namespace dai