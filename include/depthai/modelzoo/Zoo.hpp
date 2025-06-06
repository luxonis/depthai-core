#pragma once

#include <filesystem>
#include <ostream>
#include <string>

namespace dai {

namespace fs = std::filesystem;

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
     *        the function will use modelsPath if provided or the DEPTHAI_ZOO_MODELS_PATH environment variable and use a path made by combining the modelsPath
     *        and the model name to the yaml file. For instance, yolo -> ./depthai_models/yolo.yaml (if modelsPath or DEPTHAI_ZOO_MODELS_PATH are
     * ./depthai_models)
     *
     * @param modelName: model name or yaml file path (string is implicitly converted to Path)
     * @param modelsPath: Path to the models folder, use environment variable DEPTHAI_ZOO_MODELS_PATH if not provided
     * @return NNModelDescription
     */
    static NNModelDescription fromYamlFile(const fs::path& modelName, const fs::path& modelsPath = "");

    /**
     * @brief Save NNModelDescription to yaml file
     *
     * @param yamlPath: Path to yaml file
     */
    void saveToYamlFile(const fs::path& yamlPath) const;

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

    /** Name of the entry in the global metadata file */
    std::string globalMetadataEntryName;
};

/**
 * @brief Get model from model zoo
 *
 * @param modelDescription: Model description
 * @param useCached: Use cached model if present, default is true
 * @param cacheDirectory: Cache directory where the cached models are stored, default is "". If cacheDirectory is set to "", this function checks the
 * DEPTHAI_ZOO_CACHE_PATH environment variable and uses that if set, otherwise the default value is used (see getDefaultCachePath).
 * @param apiKey: API key for the model zoo, default is "". If apiKey is set to "", this function checks the DEPTHAI_ZOO_API_KEY environment variable and uses
 * that if set. Otherwise, no API key is used.
 * @param progressFormat: Format to use for progress output (possible values: pretty, json, none), default is "pretty"
 * @return std::filesystem::path: Path to the model in cache
 */
fs::path getModelFromZoo(const NNModelDescription& modelDescription,
                         bool useCached = true,
                         const fs::path& cacheDirectory = "",
                         const std::string& apiKey = "",
                         const std::string& progressFormat = "none");

/**
 * @brief Helper function allowing one to download all models specified in yaml files in the given path and store them in the cache directory
 *
 * @param path: Path to the directory containing yaml files
 * @param cacheDirectory: Cache directory where the cached models are stored, default is "". If cacheDirectory is set to "", this function checks the
 * DEPTHAI_ZOO_CACHE_PATH environment variable and uses that if set, otherwise the default is used (see getDefaultCachePath).
 * @param apiKey: API key for the model zoo, default is "". If apiKey is set to "", this function checks the DEPTHAI_ZOO_API_KEY environment variable and uses
 * that if set. Otherwise, no API key is used.
 * @param progressFormat: Format to use for progress output (possible values: pretty, json, none), default is "pretty"
 * @return bool: True if all models were downloaded successfully, false otherwise
 */
bool downloadModelsFromZoo(const fs::path& path,
                           const fs::path& cacheDirectory = "",
                           const std::string& apiKey = "",
                           const std::string& progressFormat = "none");

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription);

namespace modelzoo {

/**
 * @brief Set the health endpoint (for internet check)
 *
 * @param endpoint
 */
void setHealthEndpoint(const std::string& endpoint);

/**
 * @brief Set the download endpoint (for model querying)
 *
 * @param endpoint
 */
void setDownloadEndpoint(const std::string& endpoint);

/**
 * @brief Set the default cache path (where models are cached)
 *
 * @param path
 */
void setDefaultCachePath(const fs::path& path);

/**
 * @brief Set the default models path (where yaml files are stored)
 *
 * @param path
 */
void setDefaultModelsPath(const fs::path& path);

/**
 * @brief Get the health endpoint (for internet check)
 */
std::string getHealthEndpoint();

/**
 * @brief Get the download endpoint (for model querying)
 */
std::string getDownloadEndpoint();

/**
 * @brief Get the default cache path (where models are cached)
 */
fs::path getDefaultCachePath();

/**
 * @brief Get the default models path (where yaml files are stored)
 */
fs::path getDefaultModelsPath();

}  // namespace modelzoo

}  // namespace dai