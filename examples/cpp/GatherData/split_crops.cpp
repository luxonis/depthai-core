#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/SplitterNode.hpp"
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
            configOutput.trySend(std::make_shared<dai::ImageManipConfig>(clear_cfg));

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
                cfg.setOutputSize(512, 384, dai::ImageManipConfig::ResizeMode::CENTER_CROP);
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
    dai::NNModelDescription firstStageModelDescription;
    firstStageModelDescription.model = "luxonis/yolov8-instance-segmentation-large:coco-640x480";
    firstStageModelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    auto detNN = pipeline.create<dai::node::DetectionNetwork>()->build(cam, firstStageModelDescription);

    auto* fullResOutput = cam->requestOutput({1920, 1440 });

    // Create crops
    auto cropConfigsCreator = pipeline.create<CropConfigsCreator>();
    detNN->out.link(cropConfigsCreator->detectionsInput);
    //

    // // add crop configs to message group
    // auto gatherCropConfigs = pipeline.create<dai::node::GatherData>();
    // detNN->out.link(gatherCropConfigs->referenceInput);
    // cropConfigsCreator->configOutput.link(gatherCropConfigs->collectingInput);
    // //

    // // need to split out the leaves of the message group. Those leaves at this stage are cropConfigs
    // auto splitCropConfigs = pipeline.create<dai::node::SplitterNode>();
    // gatherCropConfigs->output.link(splitCropConfigs->input);
        // //

    // imageManip
    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->inputConfig.setReusePreviousMessage(false);
    imageManip->setMaxOutputFrameSize(384 * 512 * 3);
    
    // splitCropConfigs->output.link(imageManip->inputConfig);  // splitCropConfigs is the node !!
    cropConfigsCreator->configOutput.link(imageManip->inputConfig);
    fullResOutput->link(imageManip->inputImage);
    //

    //  append
    auto gatherData = pipeline.create<dai::node::GatherData>();
    // gatherData->setRunOnHost(true);
    // gatherCropConfigs->output.link(gatherData->referenceInput);  // LINK the previous gather into it here, will append msgs to the end
    detNN->out.link(gatherData->referenceInput);
    imageManip->out.link(gatherData->collectingInput);


    dai::NNModelDescription poseModelDescription;
    poseModelDescription.model = "luxonis/yolov8-nano-pose-estimation:coco-512x384";
    poseModelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    auto detNN_2 = pipeline.create<dai::node::DetectionNetwork>()->build(imageManip->out, dai::NNArchive(dai::getModelFromZoo(poseModelDescription)));

    auto gatherData_2 = pipeline.create<dai::node::GatherData>();
    gatherData->output.link(gatherData_2->referenceInput);
    detNN_2->out.link(gatherData_2->collectingInput);

    auto collectedQ = gatherData_2->output.createOutputQueue();
    auto passthroughQ = detNN->passthrough.createOutputQueue();

    pipeline.start();

    const cv::Scalar color(255, 0, 0);
    const cv::Scalar keypointColor(0, 255, 0);
    size_t previousCropWindowCount = 0;
    std::string lastMessageGroupTree;
    while(pipeline.isRunning()) {
        auto collected = collectedQ->get<dai::MessageGroup>();
        auto passthrough = passthroughQ->get<dai::ImgFrame>();

        if(collected != nullptr) {
            gather_data_examples::printMessageGroupTreeIfChanged(*collected, lastMessageGroupTree, std::cout, "GatherData tree");
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

        size_t shownCropWindowCount = 0;
        for(size_t i = 0; i < detections->detections.size(); ++i) {
            // if you are looking for a specific detection, you can filter out here and only then look at the crop message via getChildren(0,
            // static_cast<uint32_t>(i));

            const auto cropNodeIndices = collected->getChildren(0, static_cast<uint32_t>(i));
            if(cropNodeIndices.empty()) {
                cv::destroyWindow("crop_" + std::to_string(i));
                continue;
            }

            if(cropNodeIndices.size() > 1) {
                std::cerr << "Expected only one crop message per detection, but found " << cropNodeIndices.size() << " for detection " << i << ". Skipping."
                          << std::endl;
                cv::destroyWindow("crop_" + std::to_string(i));
                continue;
            }

            auto crop = collected->get<dai::ImgFrame>(cropNodeIndices.front());
            if(crop == nullptr) {
                cv::destroyWindow("crop_" + std::to_string(i));
                continue;
            }

            auto cropFrame = crop->getCvFrame();
            const auto poseNodeIndices = collected->getChildren(cropNodeIndices.front());
            for(const auto poseNodeIndex : poseNodeIndices) {
                auto poseDetections = collected->get<dai::ImgDetections>(poseNodeIndex);
                if(poseDetections == nullptr) {
                    continue;
                }

                const auto poseDetectionsInCrop =
                    poseDetections->getTransformation().has_value() ? poseDetections->transformTo(crop->transformation) : *poseDetections;
                for(const auto& poseDetection : poseDetectionsInCrop.detections) {
                    for(const auto& keypoint : poseDetection.getKeypoints()) {
                        const auto keypointPos = cv::Point(static_cast<int>(keypoint.imageCoordinates.x * cropFrame.cols),
                                                           static_cast<int>(keypoint.imageCoordinates.y * cropFrame.rows));
                        cv::circle(cropFrame, keypointPos, 3, keypointColor, -1);
                    }
                }
            }

            cv::imshow("crop_" + std::to_string(i), cropFrame);
            shownCropWindowCount = i + 1;
        }

        for(size_t i = shownCropWindowCount; i < previousCropWindowCount; ++i) {
            cv::destroyWindow("crop_" + std::to_string(i));
        }
        previousCropWindowCount = shownCropWindowCount;

        if(cv::waitKey(1) == 'q') {
            pipeline.stop();
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
