#pragma once

#include <string>

// Yaml parsing
#include <yaml-cpp/yaml.h>
#include "depthai/utility/YamlHelpers.hpp"

#include "depthai/device/Device.hpp"

namespace dai {

class NNModelDescription {
   public:
    /**
     * @brief Initialize NNModelDescription from yaml file
     *
     * @param yamlPath: Path to yaml file
     * @return NNModelDescription
     */
    static NNModelDescription fromYaml(const std::string& yamlPath);

    /**
     * @brief Initialize NNModelDescription in code using parameters
     *
     * @param modelSlug: Model slug
     * @param platform: Model platform
     * @param modelVersionSlug: Model instance slug
     * @return NNModelDescription
     */
    static NNModelDescription fromParameters(const std::string& modelSlug, const std::string& platform, const std::string& modelVersionSlug = "");

    /**
     * @brief Initialize NNModelDescription in code using parameters
     *
     * @param modelSlug: Model slug
     * @param platform: Model platform
     * @param modelVersionSlug: Model instance slug
     * @return NNModelDescription
     */
    static NNModelDescription fromParameters(const std::string& modelSlug, const Platform platform, const std::string& modelVersionSlug = "");

    /**
     * @brief Convert NNModelDescription to string for printing purposes. This can be used for debugging.
     *
     * @return std::string: String representation
     */
    std::string toString() const;

    /** Getters */
    std::string getModelSlug() const {
        return modelSlug;
    }
    std::string getPlatform() const {
        return platform;
    }
    std::string getmodelVersionSlug() const {
        return modelVersionSlug;
    }

    /** Setters */
    void setModelSlug(const std::string& modelSlug) {
        this->modelSlug = modelSlug;
    }
    void setPlatform(const std::string& platform) {
        this->platform = platform;
    }
    void setmodelVersionSlug(const std::string& modelVersionSlug) {
        this->modelVersionSlug = modelVersionSlug;
    }

   private:
    NNModelDescription(const std::string& modelSlug, const std::string& platform = "RVC2", const std::string& modelVersionSlug = "")
        : modelSlug(modelSlug), platform(platform), modelVersionSlug(modelVersionSlug) {}

    std::string modelSlug;
    std::string platform;
    std::string modelVersionSlug;
};

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription);

}  // namespace dai