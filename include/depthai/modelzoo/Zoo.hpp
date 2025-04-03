#pragma once

#include <ostream>
#include <string>

namespace dai {
constexpr const char* MODEL_ZOO_HEALTH_ENDPOINT = "https://easyml.cloud.luxonis.com/models/api/v1/health/";
constexpr const char* MODEL_ZOO_DOWNLOAD_ENDPOINT = "https://easyml.cloud.luxonis.com/models/api/v1/models/download";
constexpr const char* MODEL_ZOO_DEFAULT_CACHE_PATH = ".depthai_cached_models";  // hidden cache folder
constexpr const char* MODEL_ZOO_DEFAULT_MODELS_PATH = "depthai_models";         // folder

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
     *        If modelName is a relative path (e.g. ./yolo.yaml), it is used as is.
     *        If modelName is a full path (e.g. /home/user/models/yolo.yaml), it is used as is.
     *        If modelName is a model name (e.g. yolo) or a model yaml file (e.g. yolo.yaml),
     *        the function will use the DEPTHAI_ZOO_MODELS_PATH environment variable and use a path made by combining the DEPTHAI_ZOO_MODELS_PATH environment
     *        variable and the model name to the yaml file. For instance, yolo -> ./depthai_models/yolo.yaml (if DEPTHAI_ZOO_MODELS_PATH is ./depthai_models)
     *
     * @param modelName: model name or yaml file path
     * @return NNModelDescription
     */
    static NNModelDescription fromYamlFile(const std::string& modelName);

    /**
     * @brief Save NNModelDescription to yaml file
     *
     * @param yamlPath: Path to yaml file
     */
    void saveToYamlFile(const std::string& yamlPath) const;

    /**
     * @brief Check if the model description is valid (contains all required fields)
     *
     * @return bool: True if the model description is valid, false otherwise
     */
    bool check() const;

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

/**
 * @brief Get model from model zoo
 *
 * @param modelDescription: Model description
 * @param useCached: Use cached model if present, default is true
 * @param cacheDirectory: Cache directory where the cached models are stored, default is "". If cacheDirectory is set to "", this function checks the
 * DEPTHAI_ZOO_CACHE_PATH environment variable and uses that if set, otherwise the default value stored in MODEL_ZOO_DEFAULT_CACHE_PATH is used.
 * @param apiKey: API key for the model zoo, default is "". If apiKey is set to "", this function checks the DEPTHAI_ZOO_API_KEY environment variable and uses
 * that if set. Otherwise, no API key is used.
 * @return std::string: Path to the model in cache
 */
std::string getModelFromZoo(const NNModelDescription& modelDescription,
                            bool useCached = true,
                            const std::string& cacheDirectory = "",
                            const std::string& apiKey = "");

/**
 * @brief Helper function allowing one to download all models specified in yaml files in the given path and store them in the cache directory
 *
 * @param path: Path to the directory containing yaml files
 * @param cacheDirectory: Cache directory where the cached models are stored, default is "". If cacheDirectory is set to "", this function checks the
 * DEPTHAI_ZOO_CACHE_PATH environment variable and uses that if set, otherwise the default value stored in MODEL_ZOO_DEFAULT_CACHE_PATH is used.
 * @param apiKey: API key for the model zoo, default is "". If apiKey is set to "", this function checks the DEPTHAI_ZOO_API_KEY environment variable and uses
 * that if set. Otherwise, no API key is used.
 */
void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory = "", const std::string& apiKey = "");

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription);

}  // namespace dai