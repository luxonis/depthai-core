#include <cmath>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    const std::string modelSlug = "luxonis/arcface:lfw-112x112";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::EmbeddingsParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::NNData>();
        cv::Mat frame = frameMessage->getCvFrame();
        const auto embedding = parserOutput->getFirstTensor<float>();
        double squaredNorm = 0.0;
        for(const auto value : embedding) squaredNorm += static_cast<double>(value) * value;
        std::ostringstream text;
        text << "Embedding size: " << embedding.size() << ", norm: " << std::fixed << std::setprecision(2) << std::sqrt(squaredNorm);
        cv::putText(frame, text.str(), cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);

        cv::imshow("EmbeddingsParser", frame);
        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}
