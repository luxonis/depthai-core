#include "depthai/modelzoo/Zoo.hpp"

#if defined(_WIN32) && defined(_MSC_VER)
    #include <cwctype>
#else
    #include <cctype>
#endif

#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

#include "utility/Environment.hpp"
#include "utility/Logging.hpp"
#include "utility/YamlHelpers.hpp"
#include "utility/sha1.hpp"

#ifdef DEPTHAI_ENABLE_CURL
    #include <cpr/api.h>
    #include <cpr/cprtypes.h>
    #include <cpr/parameters.h>
    #include <cpr/status_codes.h>
#endif

namespace dai {

static std::string MODEL_ZOO_HEALTH_ENDPOINT = "https://easyml.cloud.luxonis.com/models/api/v1/health/";
static std::string MODEL_ZOO_DOWNLOAD_ENDPOINT = "https://easyml.cloud.luxonis.com/models/api/v1/models/download";
static dai::Path MODEL_ZOO_DEFAULT_CACHE_PATH = ".depthai_cached_models";  // hidden cache folder
static dai::Path MODEL_ZOO_DEFAULT_MODELS_PATH = "depthai_models";         // folder

#ifdef DEPTHAI_ENABLE_CURL
class ZooManager {
   public:
    /**
     * @brief Construct a new Zoo Manager object
     *
     * @param modelDescription: Model description
     * @param cacheDirectory: Cache directory, if not provided, use default value (see getDefaultCachePath)
     */
    explicit ZooManager(NNModelDescription modelDescription, Path cacheDirectory = "", const std::string& apiKey = "")
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

        // If cache directory is not set, use the environment variable DEPTHAI_ZOO_CACHE_PATH with fallback to getDefaultCachePath()
        if(this->cacheDirectory.empty()) {
            logger::info("Trying to get cache directory from environment variable DEPTHAI_ZOO_CACHE_PATH");
            this->cacheDirectory = utility::getEnvAs<Path>("DEPTHAI_ZOO_CACHE_PATH", dai::modelzoo::getDefaultCachePath(), false);
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
     * @return dai::Path: Cache folder name
     */
    Path getModelCacheFolderPath(const Path& cacheDirectory) const;

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
     *
     * @param responseJson: JSON with download links
     * @param cprCallback: Progress callback
     */
    void downloadModel(const nlohmann::json& responseJson, std::unique_ptr<cpr::ProgressCallback> cprCallback);

    /**
     * @brief Return path to model in cache
     *
     * @return dai::Path: Path to model
     */
    Path loadModelFromCache() const;

    /**
     * @brief Get path to metadata file
     *
     * @return dai::Path: Path to metadata file
     */
    Path getMetadataFilePath() const;

    /**
     * @brief Get path to global metadata file
     *
     * @return dai::Path: Path to global metadata file
     */
    Path getGlobalMetadataFilePath() const;

    /**
     * @brief Fetch model download links from Hub
     *
     * @return nlohmann::json: JSON with download links
     */
    nlohmann::json fetchModelDownloadLinks();

    /**
     * @brief Get files in folder
     *
     * @return std::vector<dai::Path>: Files in folder
     */
    std::vector<Path> getFilesInFolder(const Path& folder) const;

    /**
     * @brief Check if internet is available
     *
     * @return bool: True if internet is available
     */
    static bool connectionToZooAvailable();

   private:
    // Description of the model
    NNModelDescription modelDescription;

    // Private key to access the Hub
    std::string apiKey;

    // Path to directory where to store the cached models
    Path cacheDirectory;
};

#endif

/**
 * @brief Get path to yaml file.
 *        If name is a relative path (e.g. ./yolo.yaml), it is returned as is.
 *        If name is a full path (e.g. /home/user/models/yolo.yaml), it is returned as is.
 *        If name is a model name (e.g. yolo) or a model yaml file (e.g. yolo.yaml),
 *        the function will use modelsPath if provided or the DEPTHAI_ZOO_MODELS_PATH environment variable and return a path to the yaml file.
 *        For instance, yolo -> ./depthai_models/yolo.yaml (if modelsPath or DEPTHAI_ZOO_MODELS_PATH are ./depthai_models)
 *
 * @param name: Name of the yaml file (implicitly converted to Path)
 * @param modelsPath: Path to the models folder, use environment variable DEPTHAI_ZOO_MODELS_PATH if not provided
 * @return dai::Path: Path to yaml file
 */
Path getYamlFilePath(const Path& name, const Path& modelsPath = "");

Path combinePaths(const Path& path1, const Path& path2);

#ifdef DEPTHAI_ENABLE_CURL
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

std::vector<Path> ZooManager::getFilesInFolder(const Path& folder) const {
    auto metadata = utility::loadYaml(getMetadataFilePath());
    auto downloadedFiles = utility::yamlGet<std::vector<std::string>>(metadata, "downloaded_files");
    std::vector<Path> files;
    for(const auto& downloadedFile : downloadedFiles) {
        if(std::filesystem::exists(combinePaths(folder, downloadedFile))) {
            files.push_back(Path(combinePaths(folder, downloadedFile)));
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

Path ZooManager::getModelCacheFolderPath(const Path& cacheDirectory) const {
    return combinePaths(cacheDirectory, getModelCacheFolderName());
}

void ZooManager::createCacheFolder() const {
    Path cacheFolderPath = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::create_directories(cacheFolderPath.native());
}

void ZooManager::removeModelCacheFolder() const {
    Path cacheFolderPath = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::remove_all(cacheFolderPath.native());

    // Remove global metadata entry
    if(!modelDescription.globalMetadataEntryName.empty()) {
        Path globalMetadataPath = getGlobalMetadataFilePath();
        if(std::filesystem::exists(globalMetadataPath)) {
            auto globalMetadata = utility::loadYaml(globalMetadataPath);
            globalMetadata.remove(modelDescription.globalMetadataEntryName);
            utility::saveYaml(globalMetadata, globalMetadataPath);
        }
    }
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
    cpr::Response response = cpr::Get(cpr::Url{dai::modelzoo::getDownloadEndpoint()}, headers, params);
    if(checkIsErrorHub(response)) {
        removeModelCacheFolder();
        throw std::runtime_error(generateErrorMessageHub(response));
    }

    // Extract download links from response
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    return responseJson;
}

void ZooManager::downloadModel(const nlohmann::json& responseJson, std::unique_ptr<cpr::ProgressCallback> cprCallback) {
    // Extract download links from response
    auto downloadLinks = responseJson["download_links"].get<std::vector<std::string>>();
    auto downloadHash = responseJson["hash"].get<std::string>();
    auto modelId = responseJson.value("model_id", "");
    auto modelVersionId = responseJson.value("model_version_id", "");
    auto modelInstanceId = responseJson.value("model_instance_id", "");

    // Metadata
    YAML::Node metadata;
    metadata["hash"] = downloadHash;
    metadata["model_id"] = modelId;
    metadata["model_version_id"] = modelVersionId;
    metadata["model_instance_id"] = modelInstanceId;
    metadata["downloaded_files"] = std::vector<std::string>();

    // Download all files and store them in cache folder
    for(const auto& downloadLink : downloadLinks) {
        cpr::Response downloadResponse = cpr::Get(cpr::Url(downloadLink), *cprCallback);
        if(checkIsErrorModelDownload(downloadResponse)) {
            removeModelCacheFolder();
            throw std::runtime_error(generateErrorMessageModelDownload(downloadResponse));
        }

        // Save downloaded file to cache folder
        std::string filename = getFilenameFromUrl(downloadLink);
        Path filepath = combinePaths(getModelCacheFolderPath(cacheDirectory), filename);
        std::ofstream file(std::filesystem::path(filepath.native()), std::ios::binary);
        file.write(downloadResponse.text.c_str(), downloadResponse.text.size());
        file.close();

        // Add filename to metadata
        metadata["downloaded_files"].push_back(filename);
    }

    // Save metadata to file
    utility::saveYaml(metadata, getMetadataFilePath());

    // Save global metadata to file
    if(!modelDescription.globalMetadataEntryName.empty()) {
        YAML::Node globalMetadata;
        Path globalMetadataPath = getGlobalMetadataFilePath();
        if(std::filesystem::exists(globalMetadataPath)) {
            globalMetadata = utility::loadYaml(globalMetadataPath);
        }
        metadata["folder_name"] = getModelCacheFolderName();
        globalMetadata[modelDescription.globalMetadataEntryName] = metadata;
        utility::saveYaml(globalMetadata, globalMetadataPath);
    }
}

Path ZooManager::loadModelFromCache() const {
    const Path cacheFolder = getModelCacheFolderPath(cacheDirectory);

    // Make sure the cache folder exists
    if(!std::filesystem::exists(cacheFolder)) throw std::runtime_error("Cache folder " + cacheFolder.string() + " not found.");

    // Find all files in cache folder
    std::vector<Path> folderFiles = getFilesInFolder(cacheFolder);

    // Make sure there are files in the folder
    if(folderFiles.empty()) throw std::runtime_error("No files found in cache folder " + cacheFolder.string());

    // Return absolute path to the first file found
    return Path(std::filesystem::absolute(folderFiles[0]));
}

class CprCallback {
   public:
    virtual ~CprCallback() = default;
    CprCallback(const std::string& modelName) : modelName(modelName) {}

    virtual void cprCallback(
        cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) = 0;

    virtual std::unique_ptr<cpr::ProgressCallback> getCprProgressCallback() {
        return std::make_unique<cpr::ProgressCallback>(
            [this](cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) {
                this->cprCallback(downloadTotal, downloadNow, uploadTotal, uploadNow, userdata);
                return true;
            });
    }

   protected:
    std::string modelName;
};

class JsonCprCallback : public CprCallback {
    constexpr static long long PRINT_INTERVAL_MS = 100;

   public:
    JsonCprCallback(const std::string& modelName) : CprCallback(modelName) {
        startTime = std::chrono::steady_clock::time_point::min();
    }

    void print(long downloadTotal, long downloadNow, const std::string& modelName) {
        nlohmann::json json = {
            {"download_total", downloadTotal},
            {"download_now", downloadNow},
            {"model_name", modelName},
        };
        std::cout << json.dump() << std::endl;
    }

    void cprCallback(
        cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) override {
        (void)uploadTotal;
        (void)uploadNow;
        (void)userdata;

        bool firstCall = startTime == std::chrono::steady_clock::time_point::min();
        if(firstCall || downloadTotal == 0) {
            startTime = std::chrono::steady_clock::now();
        }

        bool shouldPrint = std::chrono::steady_clock::now() - startTime > std::chrono::milliseconds(PRINT_INTERVAL_MS) || this->downloadTotal != downloadTotal;

        if(shouldPrint) {
            print(downloadTotal, downloadNow, modelName);
            startTime = std::chrono::steady_clock::now();
        }

        this->downloadTotal = downloadTotal;
        this->downloadNow = downloadNow;
    }

    ~JsonCprCallback() override {
        if(downloadTotal != 0) {
            print(downloadTotal, downloadNow, modelName);
        }
    }

   private:
    long downloadTotal = 0;
    long downloadNow = 0;
    std::chrono::steady_clock::time_point startTime;
};

class PrettyCprCallback : public CprCallback {
   public:
    PrettyCprCallback(const std::string& modelName) : CprCallback(modelName), finalProgressPrinted(false) {}

    void cprCallback(
        cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) override {
        (void)uploadTotal;
        (void)uploadNow;
        (void)userdata;

        if(finalProgressPrinted) return;

        if(downloadTotal > 0) {
            float progress = static_cast<float>(downloadNow) / downloadTotal;
            int barWidth = 50;
            int pos = static_cast<int>(barWidth * progress);

            std::cout << "\rDownloading " << modelName << " [";
            for(int i = 0; i < barWidth; ++i) {
                if(i < pos)
                    std::cout << "=";
                else if(i == pos)
                    std::cout << ">";
                else
                    std::cout << " ";
            }
            std::cout << "] " << std::fixed << std::setprecision(3) << progress * 100.0f << "% " << downloadNow / 1024.0f / 1024.0f << "/"
                      << downloadTotal / 1024.0f / 1024.0f << " MB";

            if(downloadNow == downloadTotal) {
                std::cout << std::endl;
                finalProgressPrinted = true;
            } else {
                std::cout << "\r";
                std::cout.flush();
            }
        }
    }

   private:
    bool finalProgressPrinted;
};

class NoneCprCallback : public CprCallback {
   public:
    NoneCprCallback(const std::string& modelName) : CprCallback(modelName) {}

    void cprCallback(cpr::cpr_off_t, cpr::cpr_off_t, cpr::cpr_off_t, cpr::cpr_off_t, intptr_t) override {
        // Do nothing
    }
};

std::unique_ptr<CprCallback> getCprCallback(const std::string& format, const std::string& name) {
    if(format == "json") {
        return std::make_unique<JsonCprCallback>(name);
    } else if(format == "pretty") {
        return std::make_unique<PrettyCprCallback>(name);
    } else if(format == "none") {
        return std::make_unique<NoneCprCallback>(name);
    }
    throw std::runtime_error("Invalid format: " + format);
}

Path getModelFromZoo(
    const NNModelDescription& modelDescription, bool useCached, const Path& cacheDirectory, const std::string& apiKey, const std::string& progressFormat) {
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

    logger::info(fmt::format(
        "Model is cached: {} | Metadata present: {} | Use cached model: {} | Perform internet check: {} | Internet is available: {} | useCached: {}",
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
            Path modelPath = zooManager.loadModelFromCache();
            logger::info("Using cached model located at {}", modelPath.string());
            return modelPath;
        }

        auto responseHash = responseJson["hash"].get<std::string>();
        auto metadata = utility::loadYaml(zooManager.getMetadataFilePath());
        auto metadataHash = utility::yamlGet<std::string>(metadata, "hash");

        if(responseHash == metadataHash) {
            Path modelPath = zooManager.loadModelFromCache();
            logger::info("Using cached model located at {}", modelPath.string());
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
        throw std::runtime_error("Model not available. No internet connection available. Could not connect to host: " + dai::modelzoo::getHealthEndpoint());
    }

    // Create cache folder
    zooManager.createCacheFolder();

    // Create download progress callback
    std::unique_ptr<CprCallback> cprCallback =
        getCprCallback(progressFormat, modelDescription.globalMetadataEntryName.size() > 0 ? modelDescription.globalMetadataEntryName : modelDescription.model);

    // Download model
    logger::info("Downloading model from model zoo");
    zooManager.downloadModel(responseJson, cprCallback->getCprProgressCallback());

    // Store model as yaml in the cache folder
    Path yamlPath = combinePaths(zooManager.getModelCacheFolderPath(cacheDirectory), "model.yaml");
    modelDescription.saveToYamlFile(yamlPath);

    // Find path to model in cache
    Path modelPath = zooManager.loadModelFromCache();
    return modelPath;
}

bool downloadModelsFromZoo(const Path& path, const Path& cacheDirectory, const std::string& apiKey, const std::string& progressFormat) {
    logger::info("Downloading models from zoo");
    // Make sure 'path' exists
    if(!std::filesystem::exists(path)) throw std::runtime_error("Path does not exist: " + path.string());

    // Find all yaml files in 'path'
    std::vector<Path> models;
    for(const auto& entry : std::filesystem::recursive_directory_iterator(path)) {
        // This path is relative and without the ./ prefix (OS agnostic)
        // For instance, if path is ./models, and entry is thus ./models/something/foo.yaml
        // this modelPath variable becomes something/foo.yaml
        Path modelPath = std::filesystem::path(entry).lexically_relative(path).lexically_normal();
        if(utility::isYamlFile(modelPath)) {
            logger::debug("Found model in folder {}: {}", path.string(), modelPath.string());
            models.push_back(modelPath);
        }
    }

    // Download models from yaml files
    int numSuccess = 0, numFail = 0;
    for(size_t i = 0; i < models.size(); ++i) {
        // Parse yaml file
        const Path& modelName = models[i];

        // Download model - ignore the returned model path here == we are only interested in downloading the model
        try {
            logger::info("Downloading model [{} / {}]: {}", i + 1, models.size(), modelName.string());
            auto modelDescription = NNModelDescription::fromYamlFile(modelName, path);
            getModelFromZoo(modelDescription, true, cacheDirectory, apiKey, progressFormat);
            logger::info("Downloaded model [{} / {}]: {}", i + 1, models.size(), modelName.string());
            numSuccess++;
        } catch(const std::exception& e) {
            logger::error("Failed to download model [{} / {}]: {} in folder {}\n{}", i + 1, models.size(), modelName.string(), path.string(), e.what());
            numFail++;
        }
    }

    logger::info("Downloaded {} models from folder {} | {} failed.", numSuccess, path.string(), numFail);
    return numFail == 0;
}

bool ZooManager::connectionToZooAvailable() {
    int timeoutMs = utility::getEnvAs<int>("DEPTHAI_ZOO_INTERNET_CHECK_TIMEOUT", 1000);  // default is 1000ms
    const std::string host = dai::modelzoo::getHealthEndpoint();

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

Path ZooManager::getMetadataFilePath() const {
    return combinePaths(getModelCacheFolderPath(cacheDirectory), "metadata.yaml");
}

Path ZooManager::getGlobalMetadataFilePath() const {
    return combinePaths(cacheDirectory, "metadata.yaml");
}

#else

Path getModelFromZoo(
    const NNModelDescription& modelDescription, bool useCached, const Path& cacheDirectory, const std::string& apiKey, const std::string& progressFormat) {
    (void)modelDescription;
    (void)useCached;
    (void)cacheDirectory;
    (void)apiKey;
    (void)progressFormat;
    throw std::runtime_error("getModelFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}

bool downloadModelsFromZoo(const Path& path, const Path& cacheDirectory, const std::string& apiKey, const std::string& progressFormat) {
    (void)path;
    (void)cacheDirectory;
    (void)apiKey;
    (void)progressFormat;
    throw std::runtime_error("downloadModelsFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}

#endif

Path combinePaths(const Path& path1, const Path& path2) {
    return Path(std::filesystem::path(path1.native()) / path2.native());
}

Path getYamlFilePath(const Path& name, const Path& modelsPath) {
    // No empty names allowed
    if(name.empty()) throw std::runtime_error("name cannot be empty!");

#if defined(_WIN32) && defined(_MSC_VER)
    auto alphanum_check = [](wchar_t c) { return std::iswalnum(c); };
#else
    auto alphanum_check = [](char c) { return std::isalnum(c); };
#endif

    // If the name does not start with any dot or slash, we treat it as the special
    // case of where we prepend the DEPTHAI_ZOO_MODELS_PATH environment variable first.
    // We check whether the first character is a letter or a number here (model.yaml, model, 3model, ...)
    if(alphanum_check(name.native()[0])) {
        Path useModelsPath = modelsPath;
        if(useModelsPath.empty()) {
            useModelsPath = Path(utility::getEnvAs<Path>("DEPTHAI_ZOO_MODELS_PATH", dai::modelzoo::getDefaultModelsPath(), false));
        }

        Path path = combinePaths(useModelsPath, name);

        // if path does not contain the yaml or yml extension, add it
        if(!utility::isYamlFile(path)) {
            bool hasExtension = std::filesystem::path(path.native()).has_extension();
            if(hasExtension) {
                throw std::runtime_error("Model file not found!!!: " + path.string());
            }

            std::filesystem::path yamlPath = std::filesystem::path(path.native()).replace_extension(".yaml");
            if(std::filesystem::exists(yamlPath)) {
                path = Path(yamlPath);
            }

            std::filesystem::path ymlPath = std::filesystem::path(path.native()).replace_extension(".yml");
            if(std::filesystem::exists(ymlPath)) {
                return Path(ymlPath);
            }

            throw std::runtime_error("Model file not found: (neither `" + path.string() + ".yaml` nor `" + path.string()
                                     + ".yml` exists) | If you meant to use a relative path, prefix with ./ (e.g. `./" + name.string()
                                     + "`) | Also, make sure the file exists. Read the documentation for more information.");
        }
        return path;
    }

    // We treat the name either as a relative path or an absolute path
    return Path(name);
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

NNModelDescription NNModelDescription::fromYamlFile(const Path& modelName, const Path& modelsPath) {
    Path yamlPath = getYamlFilePath(modelName, modelsPath);
    if(!std::filesystem::exists(yamlPath)) {
        throw std::runtime_error("Model file not found: `" + yamlPath.string() + "` | If you meant to use a relative path, prefix with ./ (e.g. `./"
                                 + modelName.string() + "`) | Also, make sure the file exists. Read the documentation for more information.");
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

    // Get global metadata entry name
    auto globalMetadataEntryName = modelName.string();

    return {model, platform, optimizationLevel, compressionLevel, snpeVersion, modelPrecisionType, globalMetadataEntryName};
}

void NNModelDescription::saveToYamlFile(const Path& yamlPath) const {
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

namespace modelzoo {

void setHealthEndpoint(const std::string& endpoint) {
    MODEL_ZOO_HEALTH_ENDPOINT = endpoint;
}

void setDownloadEndpoint(const std::string& endpoint) {
    MODEL_ZOO_DOWNLOAD_ENDPOINT = endpoint;
}

void setDefaultCachePath(const Path& path) {
    MODEL_ZOO_DEFAULT_CACHE_PATH = path;
}

void setDefaultModelsPath(const Path& path) {
    MODEL_ZOO_DEFAULT_MODELS_PATH = path;
}

std::string getHealthEndpoint() {
    return MODEL_ZOO_HEALTH_ENDPOINT;
}

std::string getDownloadEndpoint() {
    return MODEL_ZOO_DOWNLOAD_ENDPOINT;
}

Path getDefaultCachePath() {
    return MODEL_ZOO_DEFAULT_CACHE_PATH;
}

Path getDefaultModelsPath() {
    return MODEL_ZOO_DEFAULT_MODELS_PATH;
}

}  // namespace modelzoo

}  // namespace dai
