
#include <chrono>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/utility/EventsManager.hpp"

int main(int argc, char* argv[]) {
    dai::Pipeline pipeline(true);

	auto eventsManager = std::make_shared<dai::utility::EventsManager>("deviceSerialNumber");
	eventsManager->setUrl("http://0.0.0.0:80/post");
	eventsManager->setLogResponse(true);
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
	eventsManager->sendEvent("test", {{"key", "value"}}, {"tag1", "tag2"});
	eventsManager->sendSnap("test2", {{"key1", "value2"}}, {"tag3", "tag4"});
	eventsManager->sendSnap("test3", {{"key3", "value3"}}, {"tag5", "tag6"});

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	eventsManager->sendSnap("test4", {{"key4", "value4"}}, {"tag7", "tag8"});
	eventsManager->sendSnap("test5", {{"key5", "value5"}}, {"tag9", "tag10"});
	eventsManager->sendSnap("test6", {{"key6", "value6"}}, {"tag11", "tag12"});
	eventsManager->sendSnap("test7", {{"key7", "value7"}}, {"tag13", "tag14"});
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	dai::utility::FileData fileData;
	fileData.data = {'a', 'b', 'c'};
	fileData.fileName = "test.txt";
	fileData.mimeType = "text/plain";
	eventsManager->sendSnap("testdata", {{"key8", "value8"}}, {"tag15", "tag16"}, {fileData});
	dai::utility::FileData fileData2;
	fileData2.fileUrl = "/test.txt";
	eventsManager->sendSnap("testdata2", {{"key8", "value8"}}, {"tag15", "tag16"}, {fileData2});
    while(pipeline.isRunning()) {
        auto detection = previewQ->get<dai::ImgFrame>();

        // Do something with the data
        // ...

		if (!sent) {
			eventsManager->sendEvent("dets", {{"key", "value"}}, {"tag1", "tag2"}, {}, detection);
			sent = true;
		}
		//
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
		eventsManager->sendSnap("test8", {{"key8", "value8"}}, {"tag15", "tag16"});
    }

    return EXIT_SUCCESS;
}
