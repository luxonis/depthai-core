#pragma once

#include <string>

// Yaml parsing
#include <yaml-cpp/yaml.h>

#include "depthai/utility/YamlHelpers.hpp"

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
     * @param name: Model name
     * @param version: Model version
     * @param platform: Model platform
     * @return NNModelDescription
     */
    static NNModelDescription fromParameters(const std::string& name, const std::string& version = "", const std::string& platform = "RVC2");

    /** Getters */
    std::string getName() const {
        return name;
    }
    std::string getVersion() const {
        return version;
    }
    std::string getPlatform() const {
        return platform;
    }

    /** Setters */
    void setName(const std::string& name) {
        this->name = name;
    }
    void setVersion(const std::string& version) {
        this->version = version;
    }
    void setPlatform(const std::string& platform) {
        this->platform = platform;
    }

   private:
    NNModelDescription(const std::string& name, const std::string& version = "", const std::string& platform = "RVC2")
        : name(name), version(version), platform(platform) {}

    std::string name;
    std::string version;
    std::string platform;
};

}  // namespace dai