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

    const std::string modelSlug = "luxonis/gaze-estimation-adas:60x60";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::RegressionParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::beta::Predictions>();
        cv::Mat frame = frameMessage->getCvFrame();
        std::ostringstream values;
        values << std::fixed << std::setprecision(3);
        for(std::size_t index = 0; index < parserOutput->predictions.size(); ++index) {
            if(index > 0) values << ' ';
            values << parserOutput->predictions[index].prediction;
        }
        cv::putText(frame, values.str(), cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);

        cv::imshow("RegressionParser", frame);
        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}
