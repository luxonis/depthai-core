#include <depthai/models/ModelLoader.hpp>
#include <iostream>
#include <string>
#include "depthai/models/Models.hpp"
#include <depthai/models/ModelZoo.hpp>

int main(int argc, char *argv[]) {

    depthai::model::ModelVariant model = depthai::model::zoo::load({"yolov6-nano", "RVC2"});
    depthai::model::SuperBlobModel superblob = std::get<depthai::model::SuperBlobModel>(model);
    std::cout << "Superblob: " << static_cast<int>(superblob.type()) << std::endl;
    std::cout << "Number of shaves: " << superblob.settings().numShaves << std::endl;

    return 0;
}