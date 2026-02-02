#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/openvino/OpenVINO.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Get model from zoo
        dai::NNModelDescription modelDesc;
        modelDesc.model = "yolov6-nano";
        modelDesc.platform = "RVC2";
        auto archivePath = dai::getModelFromZoo(modelDesc, true);  // true to use cached if available, otherwise re-download

        // Load NN archive
        dai::NNArchive archive(archivePath);

        // Verify archive type and properties
        if(archive.getModelType() != dai::model::ModelType::SUPERBLOB) {
            throw std::runtime_error("Archive is not a superblob type");
        }

        if(!archive.getSuperBlob()) {
            throw std::runtime_error("SuperBlob should not be null for superblob type");
        }

        if(archive.getBlob()) {
            throw std::runtime_error("Blob should be null for superblob type");
        }

        // Get config and print some fields
        auto config = archive.getConfig<dai::nn_archive::v1::Config>();
        std::cout << "----------" << std::endl;
        std::cout << "Config fields:" << std::endl;
        std::cout << "\tConfig version: " << config.configVersion.value() << std::endl;
        std::cout << "\tModel heads: " << config.model.heads.value().size() << std::endl;
        std::cout << "\tModel inputs: " << config.model.inputs.size() << std::endl;
        std::cout << "\tModel outputs: " << config.model.outputs.size() << std::endl;
        std::cout << "----------" << std::endl;

        // Create pipeline
        dai::Pipeline pipeline;

        // Color camera node
        auto camRgb = pipeline.create<dai::node::Camera>()->build();
        auto camOut = camRgb->requestOutput(std::make_pair(416, 416), dai::ImgFrame::Type::BGR888p);

        // Neural network node
        auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>();
        neuralNetwork->setBlob(archive.getSuperBlob()->getBlobWithNumShaves(6));
        neuralNetwork->setNumInferenceThreads(2);

        // Linking
        camOut->link(neuralNetwork->input);

        // Create output queues
        auto qDet = neuralNetwork->out.createOutputQueue();
        auto qPassthrough = neuralNetwork->passthrough.createOutputQueue();

        // Start pipeline
        pipeline.start();

        while(pipeline.isRunning() && !quitEvent) {
            auto inDet = qDet->get<dai::NNData>();
            auto inPassthrough = qPassthrough->get<dai::ImgFrame>();

            if(inDet != nullptr) {
                std::cout << "Detection data received" << std::endl;
            }

            if(inPassthrough != nullptr) {
                std::cout << "Passthrough frame received" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Cleanup
        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}