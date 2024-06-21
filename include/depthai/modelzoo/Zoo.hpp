#pragma once

#include "depthai/modelzoo/NNModelDescription.hpp"

namespace dai {

class ZooManager {
   public:
    /**
     * @brief Construct a new Zoo Manager object
     *
     * @param modelDescription: Model description
     * @param cacheDirectory: Cache directory, default is "."
     */
    ZooManager(const NNModelDescription& modelDescription, const std::string& cacheDirectory = ".")
        : modelDescription(modelDescription), cacheDirectory(cacheDirectory) {}

    /**
     * @brief Compute hash based on name, version and platform
     *
     * @return std::string: Hash value
     */
    std::string computeModelHash() const;

    /**
     * @brief Get name of the folder where model is cached
     *
     * @return std::string: Cache folder name
     */
    std::string getModelCacheFolderName() const;

    /**
     * @brief Get path to the folder where model is cached
     *
     * @param cacheDirectory: Cache directory where the cached models are stores, default is "."
     * @return std::string: Cache folder name
     */
    std::string getModelCacheFolderPath(const std::string& cacheDirectory = ".") const;

    /**
     * @brief Combine two paths
     *
     * @param path1: First path
     * @param path2: Second path
     * @return std::string: Combined path
     */
    std::string combinePaths(const std::string& path1, const std::string& path2) const;

    /**
     * @brief Create cache folder
     */
    void createCacheFolder() const;

    /**
     * @brief Remove cache folder where the model is cached
     */
    void removeModelCacheFolder() const;

    /**
     * @brief Check if model is cached
     *
     * @return bool: True if model is cached
     */
    bool isCached() const;

    /**
     * @brief Check if path exists
     *
     * @param path: Path to check
     * @return bool: True if path exists
     */
    bool checkExists(const std::string& path) const;

    /**
     * @brief Download model from model zoo
     */
    void downloadModel();

    /**
     * @brief Return path to model in cache
     *
     * @return std::string: Path to model
     */
    std::string loadModelFromCache() const;

    /**
     * @brief Return the name of the file that would be downloaded were this url entered into a web browser.
     *
     * @param url: URL
     * @return std::string: Filename
     */
    std::string getFilenameFromUrl(const std::string& url) const;

    /**
     * @brief Get all files in folder
     *
     * @param folder: Folder path
     * @return std::vector<std::string>: List of files in the folder
     */
    std::vector<std::string> getFilesInFolder(const std::string& folder) const;

   private:
    const NNModelDescription& modelDescription;
    const std::string cacheDirectory;
};

/**
 * @brief Get model from model zoo
 *
 * @param modelDescription: Model description
 * @param cacheDirectory: Cache directory where the cached models are stored, default is "."
 * @param useCached: Use cached model if present, default is true
 * @param cacheModel: Whether to store the downloaded model in the cache directory, default is true
 * @return std::string: Path to the model in cache
 */
std::string getModelFromZoo(const NNModelDescription& modelDescription, const std::string& cacheDirectory = ".", bool useCached = true, bool verbose = false);

/**
 * @brief Helper function allowing one to download all models specified in yaml files in the given path and store them in the cache directory
 *
 * @param path: Path to the directory containing yaml files
 * @param cacheDirectory: Cache directory where the cached models are stored, default is "."
 * @param verbose: Print verbose output
 */
void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory = ".", bool verbose = false);
}  // namespace dai