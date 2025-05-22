#include <depthai/depthai.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // First prepare the model for benchmarking
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    modelDescription.platform = device->getPlatformAsString();

    auto modelPath = getModelFromZoo(modelDescription);
    dai::NNArchive modelArchive(modelPath);
    auto inputSize = modelArchive.getInputSize().value();

    dai::ImgFrame::Type frameType;
    if(device->getPlatform() == dai::Platform::RVC2) {
        frameType = dai::ImgFrame::Type::BGR888p;
    } else {
        frameType = dai::ImgFrame::Type::BGR888i;
    }

    // Construct the input (white) image for benchmarking
    cv::Mat img(std::get<1>(inputSize), std::get<0>(inputSize), CV_8UC3, cv::Scalar(255, 255, 255));
    auto inputFrame = std::make_shared<dai::ImgFrame>();
    inputFrame->setCvFrame(img, frameType);

    dai::Pipeline pipeline(device);

    auto benchmarkOut = pipeline.create<dai::node::BenchmarkOut>();
    benchmarkOut->setRunOnHost(false);  // The node can run on host or on device
    benchmarkOut->setFps(-1);           // As fast as possible

    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>();
    neuralNetwork->setNNArchive(modelArchive);

    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(false);  // The node can run on host or on device
    benchmarkIn->sendReportEveryNMessages(100);
    benchmarkIn->logReportsAsWarnings(false);

    // Linking
    benchmarkOut->out.link(neuralNetwork->input);
    neuralNetwork->out.link(benchmarkIn->input);

    auto outputQueue = benchmarkIn->report.createOutputQueue();
    auto inputQueue = benchmarkOut->input.createInputQueue();

    pipeline.start();
    inputQueue->send(inputFrame);

    while(pipeline.isRunning()) {
        auto benchmarkReport = outputQueue->get<dai::BenchmarkReport>();
        std::cout << "FPS is " << benchmarkReport->fps << std::endl;
    }

    return 0;
}