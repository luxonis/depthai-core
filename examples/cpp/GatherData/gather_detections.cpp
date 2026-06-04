#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"

class CropConfigsCreator : public dai::node::CustomThreadedNode<CropConfigsCreator> {
   public:
    Output configOutput{*this, {"config_output", DEFAULT_GROUP, {{{dai::DatatypeEnum::ImageManipConfig, true}}}}};
    Input detectionsInput{*this,
                          {"detections_input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{dai::DatatypeEnum::ImgDetections, true}}}, false}};

    void run() override {
        while(mainLoop()) {
            auto detectionsInputMessage = detectionsInput.get<dai::ImgDetections>();
            if(detectionsInputMessage == nullptr) {
                continue;
            }

            const auto& detections = detectionsInputMessage->detections;

            dai::ImageManipConfig cfg;
            cfg.setReusePreviousImage(false);

            for(size_t i = 0; i < detections.size(); ++i) {
                const auto& detection = detections[i];

                auto bbox = detection.getBoundingBox();
                const auto outerBbox = bbox.getOuterRect();
                const float x1 = std::clamp(outerBbox[0], 0.0f, 0.9999f);
                const float y1 = std::clamp(outerBbox[1], 0.0f, 0.9999f);
                const float x2 = std::clamp(outerBbox[2], 0.0f, 0.9999f);
                const float y2 = std::clamp(outerBbox[3], 0.0f, 0.9999f);
                bbox = dai::RotatedRect(dai::Rect(x1, y1, x2 - x1, y2 - y1, true));

                cfg.addCropRotatedRect(bbox, true);
                cfg.setOutputSize(200, 200, dai::ImageManipConfig::ResizeMode::STRETCH);
                if(i == 0) {
                    cfg.setReusePreviousImage(false);
                } else {
                    cfg.setReusePreviousImage(true);
                }

                auto cfgMessage = std::make_shared<dai::ImageManipConfig>(cfg);
                bool sendStatus = false;
                int attempts = 0;
                while(!sendStatus && attempts < 100 && mainLoop()) {
                    sendStatus = configOutput.trySend(cfgMessage);
                    if(!sendStatus) {
                        ++attempts;
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }
            }
        }
    }
};

int main() {
    dai::Pipeline pipeline;

    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto detNN = pipeline.create<dai::node::DetectionNetwork>()->build(cam, dai::NNModelDescription{"luxonis/yolov8-instance-segmentation-large:coco-640x480"});

    auto* fullResOutput = cam->requestOutput({1280, 800});

    auto cropConfigsCreator = pipeline.create<CropConfigsCreator>();
    detNN->out.link(cropConfigsCreator->detectionsInput);

    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->inputConfig.setReusePreviousMessage(false);

    imageManip->setMaxOutputFrameSize(300 * 480 * 3);
    cropConfigsCreator->configOutput.link(imageManip->inputConfig);
    fullResOutput->link(imageManip->inputImage);

    auto cropQ = imageManip->out.createOutputQueue();
    auto detQ = detNN->out.createOutputQueue();
    auto passthroughQ = detNN->passthrough.createOutputQueue();

    pipeline.start();

    const cv::Scalar color(255, 0, 0);
    while(pipeline.isRunning()) {
        auto crops = cropQ->get<dai::ImgFrame>();
        auto detections = detQ->get<dai::ImgDetections>();
        auto passthrough = passthroughQ->get<dai::ImgFrame>();

        if(crops == nullptr || detections == nullptr || passthrough == nullptr) {
            continue;
        }

        std::cout << "Got " << detections->detections.size() << " detections)" << std::endl;

        auto frame = passthrough->getCvFrame();
        const int height = frame.rows;
        const int width = frame.cols;
        for(const auto& detection : detections->detections) {
            const int x1 = static_cast<int>(detection.xmin * width);
            const int y1 = static_cast<int>(detection.ymin * height);
            const int x2 = static_cast<int>(detection.xmax * width);
            const int y2 = static_cast<int>(detection.ymax * height);
            cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
        }

        cv::imshow("passthrough", frame);
        if(cv::waitKey(1) == 'q') {
            pipeline.stop();
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
