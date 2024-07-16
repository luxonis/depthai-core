#pragma once

#include "depthai/modelzoo/NNModelDescription.hpp"

namespace dai {
constexpr const char* MODEL_ZOO_URL = "https://api.cloud.luxonis.com/graphql";
constexpr const char* MODEL_ZOO_DEFAULT_CACHE_DIRECTORY = ".depthai_cached_models";

/**
 * @brief Get model from model zoo
 *
 * @param modelDescription: Model description
 * @param useCached: Use cached model if present, default is true
 * @param cacheDirectory: Cache directory where the cached models are stored, default is MODEL_ZOO_DEFAULT_CACHE_DIRECTORY
 * @return std::string: Path to the model in cache
 */
std::string getModelFromZoo(const NNModelDescription& modelDescription,
                            bool useCached = true,
                            const std::string& cacheDirectory = MODEL_ZOO_DEFAULT_CACHE_DIRECTORY);

/**
 * @brief Helper function allowing one to download all models specified in yaml files in the given path and store them in the cache directory
 *
 * @param path: Path to the directory containing yaml files
 * @param cacheDirectory: Cache directory where the cached models are stored, default is MODEL_ZOO_DEFAULT_CACHE_DIRECTORY
 * @param verbose: Print verbose output, default is false
 */
void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory = MODEL_ZOO_DEFAULT_CACHE_DIRECTORY);
}  // namespace dai