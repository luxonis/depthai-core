#include <depthai/models/ModelLoader.hpp>
#include <iostream>
#include <string>
#include "depthai/models/Models.hpp"
#include <depthai/models/ModelZoo.hpp>
#include <chrono>

int main(int argc, char *argv[]) {

    // Load model from ModelZoo
    auto start = std::chrono::high_resolution_clock::now();
    depthai::model::ModelVariant model = depthai::model::zoo::load({"yolov6-nano", "RVC2"});
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time taken to load model: " << elapsed.count() << " seconds" << std::endl;
    depthai::model::SuperBlobModel superblob = std::get<depthai::model::SuperBlobModel>(model);
    auto end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed2 = end2 - end;
    std::cout << "Time taken to get superblob: " << elapsed2.count() << " seconds" << std::endl;
    std::cout << "Superblob: " << static_cast<int>(superblob.type()) << std::endl;
    std::cout << "Number of shaves: " << superblob.settings().numShaves << std::endl;

    return 0;
}