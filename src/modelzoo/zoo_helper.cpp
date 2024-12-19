#include <iostream>
#include <argparse/argparse.hpp>

#include "depthai/modelzoo/Zoo.hpp"

int main(int argc, char* argv[]) {

    // Add arguments
    argparse::ArgumentParser program("DepthAI Model Zoo Helper");
    program.add_argument("--yaml_folder").help("Folder with YAML files describing models to download");
    program.add_argument("--cache_folder").help("Cache folder to download models into");

    // Parse arguments
    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return EXIT_FAILURE;
    }

    // Unpack arguments
    auto yamlFolder = program.get<std::string>("--yaml_folder");
    auto cacheFolder = program.get<std::string>("--cache_folder");

    // Print arguments
    std::cout << "Downloading models defined in yaml files in folder: " << yamlFolder << std::endl;
    std::cout << "Downloading models into cache folder: " << cacheFolder << std::endl;

    // Download models
    dai::downloadModelsFromZoo(yamlFolder, cacheFolder);

    return EXIT_SUCCESS;
}
