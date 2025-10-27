#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    // Option 1: In program
    // dai::NNModelDescription modelDescription;
    // modelDescription.model = "yolov6-nano";
    // modelDescription.platform = "RVC2";

    // Option 2: From yaml file
    auto modelDescription = dai::NNModelDescription::fromYamlFile("./mymodel.yaml");
    std::cout << "Model description: " << modelDescription << std::endl;

    // If you want, you can store the model description in a yaml file
    modelDescription.saveToYamlFile("./mymodel.yaml");

    // Return path to downloaded model - yolov6-nano-r2-288x512.tar.xz for this example
    auto modelPath = dai::getModelFromZoo(modelDescription, false);  // false means don't use cached model
    std::cout << "Model path: " << modelPath << std::endl;

    // Load the model (most of the time it's a NNArchive)
    dai::NNArchive archive(modelPath);

    // Get input size and name of the model
    auto inputSize = archive.getInputSize();
    if(inputSize) {
        std::cout << "Input size of the model: " << inputSize->first << "x" << inputSize->second << std::endl;
    }
    std::cout << "Model name is: " << archive.getConfig<dai::nn_archive::v1::Config>().model.metadata.name << std::endl;

    return 0;
}
