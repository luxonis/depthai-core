#include <catch2/catch_all.hpp>

// std
#include <atomic>
#include <iostream>
#include <memory>

// Include depthai library
#include <depthai/depthai.hpp>
#include <depthai/pipeline/InputQueue.hpp>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"

const auto MOBILENET_BLOB_PATH = BLOB_PATH;
const auto MOBILENET_WIDTH = 300;
const auto MOBILENET_HEIGHT = 300;
const auto MOBILENET_CHANNEL = 3;
const size_t MOBILENET_DATA_SIZE = MOBILENET_WIDTH * MOBILENET_HEIGHT * MOBILENET_CHANNEL;
const auto MOBILENET_INPUT_TENSOR = "data";
const auto MOBILENET_OUTPUT_TENSOR = "detection_out";

dai::Pipeline createNeuralNetworkPipeline(bool manualBlob) {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();

    // Load blob
    if(manualBlob) {
        dai::OpenVINO::Blob blob(BLOB_PATH);
        nn->setBlob(std::move(blob));
    } else {
        nn->setBlobPath(BLOB_PATH);
    }

    return p;
}

void test(bool manualBlob) {
    using namespace std::chrono;
    using namespace std::chrono_literals;

    auto pipeline = createNeuralNetworkPipeline(manualBlob);
    std::shared_ptr<dai::node::NeuralNetwork> nn = std::dynamic_pointer_cast<dai::node::NeuralNetwork>(pipeline.getAllNodes()[0]);

    auto inputQueue = nn->input.createInputQueue();
    auto outputQueue = nn->out.createOutputQueue();

    std::atomic<bool> receivedLogMessage{false};

    // no warnings should appear
    pipeline.getDefaultDevice()->setLogLevel(dai::LogLevel::WARN);
    pipeline.getDefaultDevice()->addLogCallback([&receivedLogMessage](dai::LogMessage msg) {
        if(msg.level >= dai::LogLevel::WARN) {
            receivedLogMessage = true;
        }
    });
    pipeline.start();

    // Iterate 10 times
    for(int i = 0; i < 10; i++) {
        // Setup messages
        auto buffer = std::make_shared<dai::Buffer>();
        buffer->setData(std::vector<uint8_t>(MOBILENET_DATA_SIZE + i * 1024 * 10));

        auto nndata1 = std::make_shared<dai::NNData>();
        auto nndata2 = std::make_shared<dai::NNData>();
        // Specify tensor by name
        nndata1->addTensor(MOBILENET_INPUT_TENSOR, std::vector<uint8_t>(MOBILENET_DATA_SIZE + i * 1024 * 10));
        // Specify tensor by index
        nndata2->addTensor("", std::vector<uint8_t>(MOBILENET_DATA_SIZE + i * 1024 * 10));

        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setWidth(MOBILENET_WIDTH);
        frame->setHeight(MOBILENET_HEIGHT);
        frame->setType(dai::ImgFrame::Type::BGR888p);
        frame->setData(std::vector<uint8_t>(MOBILENET_DATA_SIZE + i * 1024 * 10));

        // Create an explicit initializer list of shared pointers
        std::initializer_list<std::shared_ptr<dai::ADatatype>> messages = {buffer, nndata1, nndata2, frame};

        int msgIndex = 0;
        for(const auto& msg : messages) {
            std::cout << msgIndex << "\n";
            inputQueue->send(msg);
            bool timedOut = false;
            auto inference = outputQueue->get<dai::NNData>(1s, timedOut);
            REQUIRE(inference != nullptr);
            REQUIRE(timedOut == false);
            REQUIRE(inference->hasLayer(MOBILENET_OUTPUT_TENSOR));
            msgIndex++;
        }
    }

    // At the end test if any error messages appeared
    REQUIRE(receivedLogMessage == false);
}

TEST_CASE("Neural network node data checks - setBlobPath") {
    test(false);
}
TEST_CASE("Neural network node data checks - setBlob") {
    test(true);
}
