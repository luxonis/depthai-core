#include <argparse/argparse.hpp>
#include <iostream>

#include "../utility/Environment.hpp"
#include "depthai/build/version.hpp"
#include "depthai/modelzoo/Zoo.hpp"

int main(int argc, char* argv[]) {
    // version information
    std::ostringstream versionInfo;
    versionInfo << "DepthAI Model Zoo Helper v1.0.2" << std::endl;
    versionInfo << "Commit: " << dai::build::COMMIT << std::endl;
    versionInfo << "Commit datetime: " << dai::build::COMMIT_DATETIME << std::endl;
    versionInfo << "DepthAI version: " << dai::build::VERSION << std::endl;
    versionInfo << "Build date: " << dai::build::BUILD_DATETIME;

    // Initialize parser
    argparse::ArgumentParser program("DepthAI Model Zoo Helper", versionInfo.str());

    // Add arguments
    const std::string DEFAULT_YAML_FOLDER = dai::utility::getEnvAs<std::string>("DEPTHAI_ZOO_MODELS_PATH", dai::modelzoo::getDefaultModelsPath());
    program.add_argument("--yaml_folder").default_value(DEFAULT_YAML_FOLDER).help("Folder with YAML files describing models to download");

    const std::string DEFAULT_CACHE_FOLDER = dai::utility::getEnvAs<std::string>("DEPTHAI_ZOO_CACHE_PATH", dai::modelzoo::getDefaultCachePath());
    program.add_argument("--cache_folder").default_value(DEFAULT_CACHE_FOLDER).help("Cache folder to download models into");

    const std::string DEFAULT_API_KEY = "";
    program.add_argument("--api_key").default_value(DEFAULT_API_KEY).help("API key to use for downloading models");

    const std::string DEFAULT_HEALTH_ENDPOINT = dai::modelzoo::getHealthEndpoint();
    program.add_argument("--health_endpoint").default_value(DEFAULT_HEALTH_ENDPOINT).help("Endpoint to use for internet connection check");

    const std::string DEFAULT_DOWNLOAD_ENDPOINT = dai::modelzoo::getDownloadEndpoint();
    program.add_argument("--download_endpoint").default_value(DEFAULT_DOWNLOAD_ENDPOINT).help("Endpoint to use for downloading models");

    const std::string DEFAULT_FORMAT = "pretty";
    program.add_argument("--format").default_value(DEFAULT_FORMAT).help("Format to use for output (possible values: pretty, json)");

    program.add_argument("--verbose").default_value(false).implicit_value(true).help("Verbose output");

    // Parse arguments
    try {
        program.parse_args(argc, argv);
    } catch(const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return EXIT_FAILURE;
    }

    // Unpack arguments
    auto yamlFolder = program.get<std::string>("--yaml_folder");
    auto cacheFolder = program.get<std::string>("--cache_folder");
    auto apiKey = program.get<std::string>("--api_key");
    auto healthEndpoint = program.get<std::string>("--health_endpoint");
    auto downloadEndpoint = program.get<std::string>("--download_endpoint");
    auto format = program.get<std::string>("--format");

    bool verbose = program.get<bool>("--verbose");
    if(!dai::utility::isEnvSet("DEPTHAI_LEVEL") && verbose && format == "pretty") {
        dai::Logging::getInstance().logger.set_level(spdlog::level::info);
    }

    // Set endpoints
    dai::modelzoo::setHealthEndpoint(healthEndpoint);
    dai::modelzoo::setDownloadEndpoint(downloadEndpoint);

    // Print arguments
    if(format == "pretty") {
        std::cout << "Downloading models defined in yaml files in folder: " << yamlFolder << std::endl;
        std::cout << "Downloading models into cache folder: " << cacheFolder << std::endl;
        if(!apiKey.empty()) {
            std::cout << "Using API key: " << apiKey << std::endl;
        }
    }

    // Download models
    bool success = dai::downloadModelsFromZoo(yamlFolder, cacheFolder, apiKey, format);
    if(!success && format == "pretty") {
        std::cerr << "Failed to download all models from " << yamlFolder << std::endl;
        return EXIT_FAILURE;
    }

    if(format == "pretty") {
        std::cout << "Successfully downloaded all models from " << yamlFolder << std::endl;
    }
    return EXIT_SUCCESS;
}
