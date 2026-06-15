#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"
#include "message_group_visualizer.hpp"

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

            dai::ImageManipConfig clear_cfg;
            clear_cfg.setSkipCurrentImage(true);
            bool send_status = configOutput.trySend(std::make_shared<dai::ImageManipConfig>(clear_cfg));

            for(size_t i = 0; i < detections.size(); ++i) {
                dai::ImageManipConfig cfg;

                const auto& detection = detections[i];

                auto bbox = detection.getBoundingBox();
                const auto outerBbox = bbox.getOuterRect();
                const float x1 = std::clamp(outerBbox[0], 0.0f, 0.9999f);
                const float y1 = std::clamp(outerBbox[1], 0.0f, 0.9999f);
                const float x2 = std::clamp(outerBbox[2], 0.0f, 0.9999f);
                const float y2 = std::clamp(outerBbox[3], 0.0f, 0.9999f);
                bbox = dai::RotatedRect(dai::Rect(x1, y1, x2 - x1, y2 - y1, true));

                cfg.addCropRotatedRect(bbox, true);
                cfg.setOutputSize(300, 400, dai::ImageManipConfig::ResizeMode::CENTER_CROP);
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

    // Create crops
    auto cropConfigsCreator = pipeline.create<CropConfigsCreator>();
    detNN->out.link(cropConfigsCreator->detectionsInput);

    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->inputConfig.setReusePreviousMessage(false);

    imageManip->setMaxOutputFrameSize(300 * 480 * 3);
    cropConfigsCreator->configOutput.link(imageManip->inputConfig);
    fullResOutput->link(imageManip->inputImage);
    // end Create crops

    auto gatherData = pipeline.create<dai::node::GatherData>();
    // gatherData->setRunOnHost(true);
    detNN->out.link(gatherData->referenceInput);
    imageManip->out.link(gatherData->collectingInput);

    auto collectedQ = gatherData->output.createOutputQueue();
    auto passthroughQ = detNN->passthrough.createOutputQueue();

    pipeline.start();

    const cv::Scalar color(255, 0, 0);
    std::string lastMessageGroupTree;
    while(pipeline.isRunning()) {
        auto collected = collectedQ->get<dai::MessageGroup>();
        auto passthrough = passthroughQ->get<dai::ImgFrame>();

        if(collected != nullptr) {
            gather_data_examples::showMessageGroupTreeIfChanged(*collected, lastMessageGroupTree, "GatherData tree");
        }

        auto detections = collected != nullptr ? collected->get<dai::ImgDetections>(0) : nullptr;

        if(collected == nullptr || detections == nullptr || passthrough == nullptr) {
            continue;
        }

        // std::cout << "Got " << detections->detections.size() << " detections)" << std::endl;
        // std::cout << "Message group has " << collected->getNumMessages() << " messages." << std::endl;

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

        for(size_t i = 0; i < detections->detections.size(); ++i) {
            const auto& detection = detections->detections[i];
            // if you are looking for a specific detection, you can filter out here and only then look at the crop message via getChildren(0,
            // static_cast<uint32_t>(i));

            const auto cropNodeIndices = collected->getChildren(0, static_cast<uint32_t>(i));
            if(cropNodeIndices.empty()) {
                continue;
            }

            if(cropNodeIndices.size() > 1) {
                std::cerr << "Expected only one crop message per detection, but found " << cropNodeIndices.size() << " for detection " << i << ". Skipping."
                          << std::endl;
                continue;
            }

            auto crop = collected->get<dai::ImgFrame>(cropNodeIndices.front());
            if(crop == nullptr) {
                continue;
            }

            auto cropFrame = crop->getCvFrame();
            const auto className = detection.labelName.empty() ? std::to_string(detection.label) : detection.labelName;
            const auto detWidth = static_cast<int>(detection.getWidth() * width);
            const auto detHeight = static_cast<int>(detection.getHeight() * height);

            std::cout << "Detection " << i << ": class: " << className << ", size: " << detWidth << "x" << detHeight << std::endl;

            cv::putText(cropFrame, "class: " + className, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            cv::putText(
                cropFrame, "size: " + std::to_string(detWidth) + "x" + std::to_string(detHeight), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            cv::imshow("crop_" + std::to_string(i), cropFrame);
        }

        if(cv::waitKey(1) == 'q') {
            pipeline.stop();
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
