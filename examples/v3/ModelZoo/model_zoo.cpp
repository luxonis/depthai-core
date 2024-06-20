#include <iostream>

#include "depthai/depthai.hpp"

int main(int argc, char* argv[]) {
    dai::Device device;  // Or pipeline.getDefaultDevice() if you have a pipeline
    dai::NNModelDescription modelDescription = dai::NNModelDescription::fromParameters("ales-test", "rvc2");

    std::cout << "Using model description:\n" << modelDescription << std::endl;

    NNArchive model = dai::getModelFromZoo(modelDescription, true);  // True means use cached model if available

    return EXIT_SUCCESS;
}