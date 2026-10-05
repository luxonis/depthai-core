#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>

#include "depthai/depthai.hpp"

constexpr float FPS = 30.0f;
const std::pair<uint32_t, uint32_t> SOURCE_SIZE{640, 640};
const std::pair<uint32_t, uint32_t> ALIGN_TO_SIZE{1280, 720};

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

// Custom message carrying a line segment. Overriding transformTo() lets the Align node remap it.
class Line : public dai::TransformableBuffer {
   public:
    dai::Point2f startPoint;
    dai::Point2f endPoint;

    std::shared_ptr<dai::TransformableBuffer> transformTo(const dai::ImgTransformation& target) const override {
        auto source = getTransformation();
        if(!source.has_value()) {
            throw std::runtime_error("Source transformation is not set for the line");
        }

        auto out = std::make_shared<Line>();
        out->startPoint = source->remapPointTo(target, startPoint);
        out->endPoint = source->remapPointTo(target, endPoint);
        out->setTransformation(target);
        return out;
    }
};

void drawLine(cv::Mat& frame, const Line& line, const cv::Scalar& color, const std::string& label) {
    const cv::Point start(static_cast<int>(std::lround(line.startPoint.x)), static_cast<int>(std::lround(line.startPoint.y)));
    const cv::Point end(static_cast<int>(std::lround(line.endPoint.x)), static_cast<int>(std::lround(line.endPoint.y)));

    cv::line(frame, start, end, color, 1);
    cv::circle(frame, start, 2, color, -1);
    cv::circle(frame, end, 2, color, -1);
    cv::putText(frame, label, {start.x + 12, std::max(24, start.y - 12)}, cv::FONT_HERSHEY_TRIPLEX, 0.6, color);
}

std::shared_ptr<Line> makeLine(const dai::ImgFrame& sourceFrame) {
    auto line = std::make_shared<Line>();
    line->setTransformation(sourceFrame.transformation);
    line->startPoint = dai::Point2f(110.0f, 150.0f);
    line->endPoint = dai::Point2f(520.0f, 430.0f);
    return line;
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::nullopt, FPS);

    auto* sourceOutput = camera->requestOutput(SOURCE_SIZE, std::nullopt, dai::ImgResizeMode::LETTERBOX, FPS, false);
    auto* alignToOutput = camera->requestOutput(ALIGN_TO_SIZE, std::nullopt, dai::ImgResizeMode::STRETCH, FPS, true);

    auto align = pipeline.create<dai::node::Align>();
    align->setRunOnHost(true);  // for custom messages, Align needs to run on host.
    alignToOutput->link(align->inputAlignTo);

    auto sourceQueue = sourceOutput->createOutputQueue();
    auto alignToQueue = alignToOutput->createOutputQueue();
    auto alignedLineQueue = align->outputAligned.createOutputQueue();

    auto lineInputQueue = align->input.createInputQueue();

    pipeline.start();

    while(pipeline.isRunning() && !quitEvent) {
        auto sourceFrameMsg = sourceQueue->get<dai::ImgFrame>();
        auto alignToFrameMsg = alignToQueue->get<dai::ImgFrame>();
        if(sourceFrameMsg == nullptr || alignToFrameMsg == nullptr) {
            continue;
        }

        auto sourceLine = makeLine(*sourceFrameMsg);
        lineInputQueue->send(sourceLine);
        auto alignedLine = alignedLineQueue->get<Line>();
        if(alignedLine == nullptr) {
            throw std::runtime_error("Aligned message is not a Line");
        }

        cv::Mat sourceFrame = sourceFrameMsg->getCvFrame();
        cv::Mat alignToFrame = alignToFrameMsg->getCvFrame();

        drawLine(sourceFrame, *sourceLine, {0, 255, 0}, "source line");
        drawLine(alignToFrame, *alignedLine, {0, 140, 255}, "aligned line");

        cv::putText(sourceFrame, "source: LETTERBOX 640x640", {10, 28}, cv::FONT_HERSHEY_TRIPLEX, 0.6, {255, 255, 255});
        cv::putText(alignToFrame, "alignTo: STRETCH 1280x720", {10, 28}, cv::FONT_HERSHEY_TRIPLEX, 0.6, {255, 255, 255});

        cv::imshow("Source output", sourceFrame);
        cv::imshow("Aligned output", alignToFrame);

        if(cv::waitKey(1) == 'q') {
            pipeline.stop();
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
