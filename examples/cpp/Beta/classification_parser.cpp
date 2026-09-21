#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    const std::string modelSlug = "luxonis/emotion-recognition:260x260";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::ClassificationParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::beta::Classifications>();
        cv::Mat frame = frameMessage->getCvFrame();
        const auto count = std::min<std::size_t>({5, parserOutput->classes.size(), parserOutput->scores.size()});
        for(std::size_t index = 0; index < count; ++index) {
            std::ostringstream text;
            text << parserOutput->classes[index] << ": " << std::fixed << std::setprecision(2) << parserOutput->scores[index];
            cv::putText(frame, text.str(), cv::Point(20, 35 + static_cast<int>(index) * 25), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("ClassificationParser", frame);
        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}
