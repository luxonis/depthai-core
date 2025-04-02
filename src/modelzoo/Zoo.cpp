#include "depthai/modelzoo/Zoo.hpp"

#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>

#include "utility/Environment.hpp"
#include "utility/Logging.hpp"
#include "utility/YamlHelpers.hpp"
#include "utility/sha1.hpp"

#ifdef DEPTHAI_ENABLE_CURL
    #include <cpr/api.h>
    #include <cpr/parameters.h>
    #include <cpr/status_codes.h>
namespace dai {
class ZooManager {
   public:
    /**
     * @brief Construct a new Zoo Manager object
     *
     * @param modelDescription: Model description
     * @param cacheDirectory: Cache directory, default is ".depthai_cached_models"
     */
    explicit ZooManager(NNModelDescription modelDescription, std::string cacheDirectory = "", std::string apiKey = "")
        : modelDescription(std::move(modelDescription)), apiKey(std::move(apiKey)), cacheDirectory(std::move(cacheDirectory)) {
        // If the API is empty override from environment variable, if it exists
        if(this->apiKey.empty()) {
            logger::info("Trying to get API key from environment variable DEPTHAI_HUB_API_KEY");
            auto envApiKey = utility::getEnvAs<std::string>("DEPTHAI_HUB_API_KEY", "");
            if(!envApiKey.empty()) {
                this->apiKey = envApiKey;
                logger::info("API key found in environment variable DEPTHAI_HUB_API_KEY");
            } else {
                logger::info("API key not provided");
            }
        } else {
            logger::info("API key provided");
        }

        // If cache directory is not set, use the environment variable DEPTHAI_ZOO_CACHE_PATH with fallback to MODEL_ZOO_DEFAULT_CACHE_PATH
        if(this->cacheDirectory.empty()) {
            logger::info("Trying to get cache directory from environment variable DEPTHAI_ZOO_CACHE_PATH");
            this->cacheDirectory = utility::getEnvAs<std::string>("DEPTHAI_ZOO_CACHE_PATH", MODEL_ZOO_DEFAULT_CACHE_PATH);
        }
    }

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
     * @param cacheDirectory: Cache directory where the cached models are stores
     * @return std::string: Cache folder name
     */
    std::string getModelCacheFolderPath(const std::string& cacheDirectory) const;

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
    bool isModelCached() const;

    /**
     * @brief Download model from model zoo
     */
    void downloadModel(const nlohmann::json& responseJson);

    /**
     * @brief Return path to model in cache
     *
     * @return std::string: Path to model
     */
    std::string loadModelFromCache() const;

    /**
     * @brief Get path to metadata file
     *
     * @return std::string: Path to metadata file
     */
    std::string getMetadataFilePath() const;

    /**
     * @brief Fetch model download links from Hub
     *
     * @return nlohmann::json: JSON with download links
     */
    nlohmann::json fetchModelDownloadLinks();

    /**
     * @brief Get files in folder
     *
     * @return std::vector<std::string>: Files in folder
     */
    std::vector<std::string> getFilesInFolder(const std::string& folder) const;

    /**
     * @brief Check if internet is available
     *
     * @return bool: True if internet is available
     */
    static bool connectionToZooAvailable();

    /**
     * @brief Get path to yaml file.
     *        If name is a relative path (e.g. ./yolo.yaml), it is returned as is.
     *        If name is a full path (e.g. /home/user/models/yolo.yaml), it is returned as is.
     *        If name is a model name (e.g. yolo) or a model yaml file (e.g. yolo.yaml),
     *        the function will use the DEPTHAI_ZOO_MODELS_PATH environment variable and return a path to the yaml file.
     *        For instance, yolo -> ./depthai_models/yolo.yaml (if DEPTHAI_ZOO_MODELS_PATH is ./depthai_models)
     *
     * @param name: Name of the yaml file
     * @return std::string: Path to yaml file
     */
    static std::string getYamlFilePath(const std::string& name);

   private:
    // Description of the model
    NNModelDescription modelDescription;

    // Private key to access the Hub
    std::string apiKey;

    // Path to directory where to store the cached models
    std::string cacheDirectory;
};

std::string generateErrorMessageHub(const cpr::Response& response) {
    std::string errorMessage;
    errorMessage += "There was an error while sending a request to the Hub\n";
    errorMessage += "HTTP status code: " + std::to_string(response.status_code) + "\n";
    errorMessage += "CPR error code: " + std::to_string(static_cast<int>(response.error.code)) + "\n";
    errorMessage += "Response text: " + response.text + "\n";
    return errorMessage;
}

bool checkIsErrorModelDownload(const cpr::Response& response) {
    bool isError = response.status_code != cpr::status::HTTP_OK;
    return isError;
}

std::string generateErrorMessageModelDownload(const cpr::Response& response) {
    std::string errorMessage;
    errorMessage += "There was an error while downloading the model\n";
    errorMessage += "HTTP status code: " + std::to_string(response.status_code) + "\n";
    errorMessage += "CPR error code: " + std::to_string(static_cast<int>(response.error.code)) + "\n";
    errorMessage += "Response text: " + response.text + "\n";
    return errorMessage;
}

std::string combinePaths(const std::string& path1, const std::string& path2) {
    return std::filesystem::path(path1).append(path2).string();
}

bool checkIsErrorHub(const cpr::Response& response) {
    // Check if response is an HTTP error
    if(response.status_code != cpr::status::HTTP_OK) return true;

    // If there was no HTTP error, check presence of required fields
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    if(!responseJson.contains("hash")) return true;
    if(!responseJson.contains("download_links")) return true;

    // All checks passed - no errors yay
    return false;
}

std::vector<std::string> ZooManager::getFilesInFolder(const std::string& folder) const {
    auto metadata = utility::loadYaml(getMetadataFilePath());
    auto downloadedFiles = utility::yamlGet<std::vector<std::string>>(metadata, "downloaded_files");
    std::vector<std::string> files;
    for(const auto& downloadedFile : downloadedFiles) {
        if(std::filesystem::exists(combinePaths(folder, downloadedFile))) {
            files.push_back(combinePaths(folder, downloadedFile));
        }
    }
    return files;
}

std::string getFilenameFromUrl(const std::string& url) {
    // Example url: https://storage.googleapis.com/luxonis/bla/resnet18_rvc2.blob?a=33a&b=443b&last=elon
    // It's fake ^^^ :) but it's just to show how the function works
    // For this example url, the function would return "resnet18_rvc2.blob"

    // Find query string start
    size_t queryStart = url.find('?');
    bool queryFound = queryStart != std::string::npos;
    if(!queryFound) queryStart = url.size();

    // Cut off query string
    const std::string urlWithoutQuery = url.substr(0, queryStart);

    // Find last slash
    size_t lastSlash = urlWithoutQuery.find_last_of('/');

    // Extract filename
    const std::string filename = urlWithoutQuery.substr(lastSlash + 1);
    return filename;
}

std::string ZooManager::computeModelHash() const {
    SHA1 hasher;
    hasher.update(modelDescription.toString());
    return hasher.final();
}

std::string ZooManager::getModelCacheFolderName() const {
    return computeModelHash();
}

std::string ZooManager::getModelCacheFolderPath(const std::string& cacheDirectory) const {
    return combinePaths(cacheDirectory, getModelCacheFolderName());
}

void ZooManager::createCacheFolder() const {
    std::string cacheFolderName = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::create_directories(cacheFolderName);
}

void ZooManager::removeModelCacheFolder() const {
    std::string cacheFolderName = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::remove_all(cacheFolderName);
}

bool ZooManager::isModelCached() const {
    return std::filesystem::exists(getModelCacheFolderPath(cacheDirectory));
}

nlohmann::json ZooManager::fetchModelDownloadLinks() {
    // Add request parameters
    cpr::Parameters params;

    // Required parameters
    // clang-format off
    std::vector<std::pair<std::string, std::string>> requiredParams = {
        {"slug", modelDescription.model},
        {"platform", modelDescription.platform}
    };
    // clang-format on
    for(const auto& param : requiredParams) {
        params.Add({param.first, param.second});
    }

    // Optional parameters
    // clang-format off
    std::vector<std::pair<std::string, std::string>> optionalParams = {
        {"optimizationLevel", modelDescription.optimizationLevel},
        {"compressionLevel", modelDescription.compressionLevel},
        {"snpeVersion", modelDescription.snpeVersion},
        {"modelPrecisionType", modelDescription.modelPrecisionType}
    };
    // clang-format on
    for(const auto& param : optionalParams) {
        if(!param.second.empty()) {
            params.Add({param.first, param.second});
        }
    }

    // Set the Authorization headers
    cpr::Header headers = {
        {"Content-Type", "application/json"},
    };
    if(!apiKey.empty()) {
        headers["Authorization"] = "Bearer " + apiKey;
    }

    // Send HTTP GET request to REST endpoint
    cpr::Response response = cpr::Get(cpr::Url{MODEL_ZOO_DOWNLOAD_ENDPOINT}, headers, params);
    if(checkIsErrorHub(response)) {
        removeModelCacheFolder();
        throw std::runtime_error(generateErrorMessageHub(response));
    }

    // Extract download links from response
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    return responseJson;
}

void ZooManager::downloadModel(const nlohmann::json& responseJson) {
    // Extract download links from response
    auto downloadLinks = responseJson["download_links"].get<std::vector<std::string>>();
    auto downloadHash = responseJson["hash"].get<std::string>();

    // Metadata
    YAML::Node metadata;
    metadata["hash"] = downloadHash;
    metadata["downloaded_files"] = std::vector<std::string>();

    // Download all files and store them in cache folder
    for(const auto& downloadLink : downloadLinks) {
        cpr::Response downloadResponse = cpr::Get(cpr::Url(downloadLink));
        if(checkIsErrorModelDownload(downloadResponse)) {
            removeModelCacheFolder();
            throw std::runtime_error(generateErrorMessageModelDownload(downloadResponse));
        }

        // Save downloaded file to cache folder
        std::string filename = getFilenameFromUrl(downloadLink);
        std::string filepath = combinePaths(getModelCacheFolderPath(cacheDirectory), filename);
        std::ofstream file(filepath, std::ios::binary);
        file.write(downloadResponse.text.c_str(), downloadResponse.text.size());
        file.close();

        // Add filename to metadata
        metadata["downloaded_files"].push_back(filename);
    }

    // Save metadata to file
    utility::saveYaml(metadata, getMetadataFilePath());
}

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

NNModelDescription NNModelDescription::fromYamlFile(const std::string& modelName) {
    std::string yamlPath = ZooManager::getYamlFilePath(modelName);
    if(!std::filesystem::exists(yamlPath)) {
        throw std::runtime_error("Model file not found: `" + yamlPath + "` | If you meant to use a relative path, prefix with ./ (e.g. `./" + modelName
                                 + "`) | Also, make sure the file exists. Read the documentation for more information.");
    }

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

bool NNModelDescription::check() const {
    return !model.empty() && !platform.empty();
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

std::string ZooManager::loadModelFromCache() const {
    const std::string cacheFolder = getModelCacheFolderPath(cacheDirectory);

    // Make sure the cache folder exists
    if(!std::filesystem::exists(cacheFolder)) throw std::runtime_error("Cache folder " + cacheFolder + " not found.");

    // Find all files in cache folder
    std::vector<std::string> folderFiles = getFilesInFolder(cacheFolder);

    // Make sure there are files in the folder
    if(folderFiles.empty()) throw std::runtime_error("No files found in cache folder " + cacheFolder);

    // Return absolute path to the first file found
    return std::filesystem::absolute(folderFiles[0]).string();
}

std::string getModelFromZoo(const NNModelDescription& modelDescription, bool useCached, const std::string& cacheDirectory, const std::string& apiKey) {
    // Check if model description is valid
    if(!modelDescription.check()) throw std::runtime_error("Invalid model description:\n" + modelDescription.toString());

    // Initialize ZooManager
    ZooManager zooManager(modelDescription, cacheDirectory, apiKey);

    // Check if model is cached
    bool modelIsCached = zooManager.isModelCached();
    bool isMetadataPresent = std::filesystem::exists(zooManager.getMetadataFilePath());
    bool useCachedModel = useCached && modelIsCached && isMetadataPresent;

    bool performInternetCheck = utility::getEnvAs<bool>("DEPTHAI_ZOO_INTERNET_CHECK", true);  // default is true

    // Check if internet is available
    bool internetIsAvailable = performInternetCheck && ZooManager::connectionToZooAvailable();
    nlohmann::json responseJson;

    logger::info(fmt::format("Model is cached: {} | Metadata present: {} | Use cached model: {} | Perform internet check: {} | Internet is available: {} | useCached: {}",
                             modelIsCached,
                             isMetadataPresent,
                             useCachedModel,
                             performInternetCheck,
                             internetIsAvailable,
                             useCached));

    if(internetIsAvailable) {
        responseJson = zooManager.fetchModelDownloadLinks();
    }

    // Use cached model if present and useCached is true
    if(useCachedModel) {
        if(!internetIsAvailable) {
            std::string modelPath = zooManager.loadModelFromCache();
            logger::info("Using cached model located at {}", modelPath);
            return modelPath;
        }

        auto responseHash = responseJson["hash"].get<std::string>();
        auto metadata = utility::loadYaml(zooManager.getMetadataFilePath());
        auto metadataHash = utility::yamlGet<std::string>(metadata, "hash");

        if(responseHash == metadataHash) {
            std::string modelPath = zooManager.loadModelFromCache();
            logger::info("Using cached model located at {}", modelPath);
            return modelPath;
        }

        logger::warn("Cached model hash does not match response hash, downloading anew ...");
    }

    // Remove cached model if present
    if(modelIsCached) {
        logger::info("Removing cached model folder");
        zooManager.removeModelCacheFolder();
    }

    // Model is not cached and internet check is disabled
    if(!performInternetCheck) {
        throw std::runtime_error("Model no available. Please set DEPTHAI_ZOO_INTERNET_CHECK to 1 to download models from the model zoo.");
    }

    // Model is not cached and internet check is enabled but no internet connection is available
    if(performInternetCheck && !internetIsAvailable) {
        throw std::runtime_error("Model not available. No internet connection available. Could not connect to host: " + std::string(MODEL_ZOO_HEALTH_ENDPOINT));
    }

    // Create cache folder
    zooManager.createCacheFolder();

    // Download model
    logger::info("Downloading model from model zoo");
    zooManager.downloadModel(responseJson);

    // Find path to model in cache
    std::string modelPath = zooManager.loadModelFromCache();
    return modelPath;
}

void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory, const std::string& apiKey) {
    logger::info("Downloading models from zoo");
    // Make sure 'path' exists
    if(!std::filesystem::exists(path)) throw std::runtime_error("Path does not exist: " + path);

    // Find all yaml files in 'path'
    std::vector<std::string> yamlFiles;
    for(const auto& entry : std::filesystem::directory_iterator(path)) {
        std::string filePath = entry.path().string();
        if(utility::isYamlFile(filePath)) {
            yamlFiles.push_back(filePath);
        }
    }

    // Download models from yaml files
    for(size_t i = 0; i < yamlFiles.size(); ++i) {
        // Parse yaml file
        const std::string& yamlFile = yamlFiles[i];

        // Download model - ignore the returned model path here == we are only interested in downloading the model
        try {
            auto modelDescription = NNModelDescription::fromYamlFile(yamlFile);
            getModelFromZoo(modelDescription, true, cacheDirectory, apiKey);
            logger::info("Downloaded model [{} / {}]: {}", i + 1, yamlFiles.size(), yamlFile);
        } catch(const std::exception& e) {
            logger::error("Failed to download model [{} / {}]: {}\n{}", i + 1, yamlFiles.size(), yamlFile, e.what());
        }
    }
}

bool ZooManager::connectionToZooAvailable() {
    int timeoutMs = utility::getEnvAs<int>("DEPTHAI_ZOO_INTERNET_CHECK_TIMEOUT", 1000);  // default is 1000ms
    constexpr std::string_view host = MODEL_ZOO_HEALTH_ENDPOINT;

    // Check if internet is available
    bool connected = false;
    try {
        cpr::Response r = cpr::Get(cpr::Url{host}, cpr::Timeout{timeoutMs});
        connected = r.status_code == cpr::status::HTTP_OK;
    } catch(const cpr::Error& e) {
        // pass, we don't care about the error
    }

    return connected;
}

std::string ZooManager::getYamlFilePath(const std::string& name) {
    // If the name does not start with any dot or slash, we treat it as the special
    // case of where we prepend the DEPTHAI_ZOO_MODELS_PATH environment variable first.
    // We check whether the first character is a letter or a number here (model.yaml, model, 3model, ...)
    if(isalnum(name[0])) {
        std::string modelsPath = utility::getEnvAs<std::string>("DEPTHAI_ZOO_MODELS_PATH", MODEL_ZOO_DEFAULT_MODELS_PATH);
        std::string path = combinePaths(modelsPath, name);
        // if path does not contain the yaml or yml extension, add it
        if(!utility::isYamlFile(path)) {
            if(std::filesystem::exists(path + ".yaml")) {
                path += ".yaml";
            } else if(std::filesystem::exists(path + ".yml")) {
                path += ".yml";
            } else {
                throw std::runtime_error("Model file not found: (neither `" + path + ".yaml` nor `" + path
                                         + ".yml` exists) | If you meant to use a relative path, prefix with ./ (e.g. `./" + name
                                         + "`) | Also, make sure the file exists. Read the documentation for more information.");
            }
        }
        return path;
    }

    // We treat the name either as a relative path or an absolute path
    return name;
}

std::string ZooManager::getMetadataFilePath() const {
    return combinePaths(getModelCacheFolderPath(cacheDirectory), "metadata.yaml");
}

}  // namespace dai

#else
namespace dai {
std::string getModelFromZoo(const NNModelDescription& modelDescription, bool useCached, const std::string& cacheDirectory, const std::string& apiKey) {
    (void)modelDescription;
    (void)useCached;
    (void)cacheDirectory;
    (void)apiKey;
    throw std::runtime_error("getModelFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}

void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory, const std::string& apiKey) {
    (void)path;
    (void)cacheDirectory;
    (void)apiKey;
    throw std::runtime_error("downloadModelsFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}
}  // namespace dai
#endif
