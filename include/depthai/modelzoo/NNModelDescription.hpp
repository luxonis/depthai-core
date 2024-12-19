#pragma once

#include <string>

namespace dai {
struct SlugComponents {
    std::string teamName;
    std::string modelSlug;
    std::string modelVariantSlug;
    std::string modelRef;

    // Merges the fields into a single string
    std::string merge() const;

    // Splits a slug string into components
    static SlugComponents split(const std::string& slug);
};

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

    /** Model slug = REQUIRED parameter */
    std::string model;

    /** Hardware platform - RVC2, RVC3, RVC4, ... = REQUIRED parameter */
    std::string platform;

    /** Optimization level = OPTIONAL parameter */
    std::string optimizationLevel;

    /** Compression level = OPTIONAL parameter */
    std::string compressionLevel;

    /** SNPE version = OPTIONAL parameter */
    std::string snpeVersion;

    /** modelPrecisionType = OPTIONAL parameter */
    std::string modelPrecisionType;
};

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription);

}  // namespace dai