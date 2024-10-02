
#include <chrono>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/utility/EventsManager.hpp"

int main(int argc, char* argv[]) {
    dai::Pipeline pipeline(true);

	auto eventsManager = std::make_shared<dai::utility::EventsManager>("sessionToken", "agentToken", "deviceSerialNumber");
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
    while(pipeline.isRunning()) {
        auto detection = previewQ->get<dai::ImgFrame>();

        // Do something with the data
        // ...

		if (!sent) {
			eventsManager->sendEvent("Detected", detection, {"tag1", "tag2"});
			sent = true;
		}
		//
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return EXIT_SUCCESS;
}
