
#include <chrono>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/utility/EventsManager.hpp"

int main(int argc, char* argv[]) {
    dai::Pipeline pipeline(true);

    auto eventsManager = std::make_shared<dai::utility::EventsManager>("deviceSerialNumber");
    eventsManager->setUrl("https://events-ingest.apps.stg.hubcloud/v1/events/");
    eventsManager->setLogResponse(true);
    eventsManager->setToken("tapi.7-yg7mpG0prDfDRZs1aSAQ.nUm5Kl3tg-sRmTvdsQzIwtmi9yH-m0jn-hJSXlJ-2ZfUsjRFDr-SqG9Ce66SQBPGLmuHmccidO86YtfIrPAdKw");
    // Download model from zoo
    // dai::NNModelDescription modelDescription;
    // modelDescription.modelSlug = "ales-test";
    // modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    // std::string modelPath = dai::getModelFromZoo(modelDescription, false);  // True means use cached model if available
    // std::cout << "Model path: " << modelPath << std::endl;

    // Color camera node
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(256, 256);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setInterleaved(false);

    // Neural network node
    // auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>();
    // neuralNetwork->setBlobPath(modelPath);
    // neuralNetwork->setNumInferenceThreads(2);

    // Linking
    // camRgb->preview.link(neuralNetwork->input);

    auto previewQ = camRgb->preview.createOutputQueue();
    // auto nnPassthroughQueue = neuralNetwork->passthrough.createOutputQueue();

    pipeline.start();
    bool sent = false;
    eventsManager->sendEvent("test", nullptr, {}, {"tag1", "tag2"}, {{"key1", "value1"}});

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    auto fileData = std::make_unique<dai::utility::EventData>("abc", "test.txt", "text/plain");
	std::vector<std::shared_ptr<dai::utility::EventData>> data;
	data.emplace_back(std::move(fileData));
    eventsManager->sendSnap("testdata", nullptr, std::move(data), {"tag3, tag4"}, {{"key8", "value8"}});
    auto fileData2 = std::make_unique<dai::utility::EventData>("/test.txt");
	std::vector<std::shared_ptr<dai::utility::EventData>> data2;
	data2.emplace_back(std::move(fileData2));
    eventsManager->sendSnap("testdata2", nullptr, std::move(data2), {"tag5, tag6"}, {{"key8", "value8"}});
    while(pipeline.isRunning()) {
        auto detection = previewQ->get<dai::ImgFrame>();

        // Do something with the data
        // ...

        if(!sent) {
            eventsManager->sendEvent("dets", detection, {}, {"tag1", "tag2"}, {{"key", "value"}});
            sent = true;
        }
        //
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // eventsManager->sendSnap("test8", {{"key8", "value8"}}, {"tag15", "tag16"});
    }

    return EXIT_SUCCESS;
}
