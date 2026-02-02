#include <optional>

#include "depthai/depthai.hpp"

constexpr int FPS = 480;

int main() {
    dai::Pipeline pipeline;

    auto platform = pipeline.getDefaultDevice()->getPlatform();
    if(platform != dai::Platform::RVC4) {
        throw std::runtime_error("This example is only supported on RVC4 devices");
    }

    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    modelDescription.platform = "RVC4";

    auto nnArchivePath = getModelFromZoo(modelDescription);
    dai::NNArchive nnArchive(nnArchivePath);
    auto inputSize = nnArchive.getInputSize().value();

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();

    // Configure the ImageManip as in HFR mode requesting arbitrary outputs is not yet supported
    auto cameraOutput = cameraNode->requestOutput(std::make_pair(1280, 720), std::nullopt, dai::ImgResizeMode::CROP, static_cast<float>(FPS));

    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->initialConfig->setOutputSize(std::get<0>(inputSize), std::get<1>(inputSize));
    imageManip->setMaxOutputFrameSize(static_cast<int>(std::get<0>(inputSize) * std::get<1>(inputSize) * 3));
    imageManip->initialConfig->setFrameType(dai::ImgFrame::Type::BGR888i);
    cameraOutput->link(imageManip->inputImage);

    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
    detectionNetwork->setNNArchive(nnArchive);
    imageManip->out.link(detectionNetwork->input);

    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(true);
    benchmarkIn->sendReportEveryNMessages(FPS);
    detectionNetwork->out.link(benchmarkIn->input);

    auto qDet = detectionNetwork->out.createOutputQueue();
    pipeline.start();

    while(pipeline.isRunning()) {
        auto inDet = qDet->get<dai::ImgDetections>();
        if(inDet == nullptr) continue;
        (void)inDet;
    }

    return 0;
}
