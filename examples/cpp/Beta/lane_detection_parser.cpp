#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    const std::string modelSlug = "luxonis/ultra-fast-lane-detection:culane-800x288";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::LaneDetectionParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::beta::Clusters>();
        cv::Mat frame = frameMessage->getCvFrame();
        for(const auto& cluster : parserOutput->clusters) {
            std::vector<cv::Point> points;
            for(const auto& point : cluster.points) {
                points.emplace_back(static_cast<int>(point.x * frame.cols), static_cast<int>(point.y * frame.rows));
            }
            if(points.size() > 1) cv::polylines(frame, points, false, cv::Scalar(0, 255, 0), 3);
        }

        cv::imshow("LaneDetectionParser", frame);
        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}
