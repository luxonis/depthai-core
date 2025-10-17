#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"

class ImageReplay : public dai::NodeCRTP<dai::node::ThreadedHostNode, ImageReplay> {
   public:
    constexpr static const char* NAME = "ImageReplay";

    Output output{*this, {"out", DEFAULT_GROUP, {{{dai::DatatypeEnum::ImgFrame, true}}}}};

    ImageReplay() {
        // Load and prepare the image
        cv::Mat frame = cv::imread(APRIL_TAGS_PATH);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

        // Create ImgFrame
        auto imgFrame = std::make_shared<dai::ImgFrame>();
        std::vector<uint8_t> data(frame.data, frame.data + frame.total() * frame.elemSize());
        imgFrame->setData(data);
        imgFrame->setWidth(frame.cols);
        imgFrame->setHeight(frame.rows);
        imgFrame->setType(dai::ImgFrame::Type::GRAY8);
        _imgFrame = imgFrame;
    }

    void run() override {
        while(mainLoop()) {
            output.send(_imgFrame);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }

   private:
    std::shared_ptr<dai::ImgFrame> _imgFrame;
};

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Create nodes
    auto imageReplay = pipeline.create<ImageReplay>();
    auto aprilTagNode = pipeline.create<dai::node::AprilTag>();

    // Link nodes
    imageReplay->output.link(aprilTagNode->inputImage);
    aprilTagNode->initialConfig->setFamily(dai::AprilTagConfig::Family::TAG_16H5);

    // Create output queues
    auto passthroughOutputQueue = aprilTagNode->passthroughInputImage.createOutputQueue();
    auto outQueue = aprilTagNode->out.createOutputQueue();

    // Start pipeline
    pipeline.start();

    // FPS calculation variables
    cv::Scalar color(0, 255, 0);
    auto startTime = std::chrono::steady_clock::now();
    int counter = 0;
    float fps = 0.0f;

    // Main loop
    while(pipeline.isRunning()) {
        auto aprilTagMessage = outQueue->get<dai::AprilTags>();
        auto aprilTags = aprilTagMessage->aprilTags;

        // Calculate FPS
        counter++;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        if(elapsed >= 1) {
            fps = counter / static_cast<float>(elapsed);
            counter = 0;
            startTime = currentTime;
        }

        // Get passthrough image
        auto passthroughImage = passthroughOutputQueue->get<dai::ImgFrame>();
        cv::Mat frame = passthroughImage->getCvFrame();
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

        // Helper function to convert points to integers
        auto to_int = [](const dai::Point2f& p) { return cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)); };

        // Draw detections
        for(const auto& tag : aprilTags) {
            auto topLeft = to_int(tag.topLeft);
            auto topRight = to_int(tag.topRight);
            auto bottomRight = to_int(tag.bottomRight);
            auto bottomLeft = to_int(tag.bottomLeft);

            auto center = cv::Point((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);

            // Draw rectangle
            cv::line(frame, topLeft, topRight, color, 2, cv::LINE_AA, 0);
            cv::line(frame, topRight, bottomRight, color, 2, cv::LINE_AA, 0);
            cv::line(frame, bottomRight, bottomLeft, color, 2, cv::LINE_AA, 0);
            cv::line(frame, bottomLeft, topLeft, color, 2, cv::LINE_AA, 0);

            // Draw ID
            std::string idStr = "ID: " + std::to_string(tag.id);
            cv::putText(frame, idStr, center, cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            // Draw FPS
            cv::putText(frame, "fps: " + std::to_string(fps).substr(0, 4), cv::Point(200, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        }

        cv::imshow("detections", frame);
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
