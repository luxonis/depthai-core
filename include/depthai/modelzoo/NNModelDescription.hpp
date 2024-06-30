#pragma once

#include <filesystem>
#include <string>

#include "depthai/device/Device.hpp"

namespace dai {

struct NNModelDescription {
    /**
     * @brief Initialize NNModelDescription from yaml file
     *
     * @param yamlPath: Path to yaml file
     * @return NNModelDescription
     */
    static NNModelDescription fromYamlFile(const std::string& yamlPath);

    /**
     * @brief Save NNModelDescription to yaml file
     *
     * @param yamlPath: Path to yaml file
     */
    void saveToYamlFile(const std::string& yamlPath) const;

    /**
     * @brief Convert NNModelDescription to string for printing purposes. This can be used for debugging.
     *
     * @return std::string: String representation
     */
    std::string toString() const;

    /** Model slug = REQUIRED parameter*/
    std::string modelSlug = "";

    /** Hardware platform - RVC2, RVC3, RVC4, ... = REQUIRED parameter*/
    std::string platform = "";

    /** Model version slug = OPTIONAL parameter */
    std::string modelVersionSlug = "";
};

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription);

}  // namespace dai