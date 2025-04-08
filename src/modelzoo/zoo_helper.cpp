#include <argparse/argparse.hpp>
#include <iostream>

#include "../utility/Environment.hpp"
#include "depthai/modelzoo/Zoo.hpp"

int main(int argc, char* argv[]) {
    // Initialize parser
    argparse::ArgumentParser program("DepthAI Model Zoo Helper");

    // Add arguments
    const std::string DEFAULT_YAML_FOLDER = dai::utility::getEnvAs<std::string>("DEPTHAI_ZOO_MODELS_PATH", dai::modelzoo::getDefaultModelsPath());
    program.add_argument("--yaml_folder").default_value(DEFAULT_YAML_FOLDER).help("Folder with YAML files describing models to download");

    const std::string DEFAULT_CACHE_FOLDER = dai::utility::getEnvAs<std::string>("DEPTHAI_ZOO_CACHE_PATH", dai::modelzoo::getDefaultCachePath());
    program.add_argument("--cache_folder").default_value(DEFAULT_CACHE_FOLDER).help("Cache folder to download models into");

    const std::string DEFAULT_API_KEY = "";
    program.add_argument("--api_key").default_value(DEFAULT_API_KEY).help("API key to use for downloading models");

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

    // Print arguments
    std::cout << "Downloading models defined in yaml files in folder: " << yamlFolder << std::endl;
    std::cout << "Downloading models into cache folder: " << cacheFolder << std::endl;
    if(!apiKey.empty()) {
        std::cout << "Using API key: " << apiKey << std::endl;
    }

    // Download models
    dai::downloadModelsFromZoo(yamlFolder, cacheFolder, apiKey);

    return EXIT_SUCCESS;
}
