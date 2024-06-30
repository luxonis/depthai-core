#include <iostream>
#include <string>

#include "depthai/modelzoo/Zoo.hpp"

int main(int argc, char* argv[]) {
    // Check if the number of arguments is correct
    if(argc != 3) {
        std::cout << "Usage: zoo_helper folder_with_yaml_files cache_folder" << std::endl;
        return EXIT_FAILURE;
    }

    // Unpack arguments
    const std::string yamlFolder = argv[1];
    const std::string cacheFolder = argv[2];

    // Download models
    dai::downloadModelsFromZoo(yamlFolder, cacheFolder, true); // true for verbose

    return EXIT_SUCCESS;
}