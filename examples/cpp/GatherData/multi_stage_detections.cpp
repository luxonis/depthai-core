#include <depthai/remote_connection/RemoteConnection.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/node/SplitterNode.hpp"

namespace {

constexpr char kFilterDetectionsScript[] = R"(
from depthai import ImgDetection, ImgDetections

kMaxDetections = 5

while True:
    src = node.inputs["detections"].get()
    if src is None:
        break

    # Build a fresh message so truncating detections does not leave a stale
    # instance-segmentation payload attached to the output message.
    dst = ImgDetections()
    dst.setSequenceNum(src.getSequenceNum())
    dst.setTimestamp(src.getTimestamp())
    dst.setTimestampDevice(src.getTimestampDevice())
    dst.setTransformation(src.getTransformation())

    filtered_detections = []
    for det in src.detections[:kMaxDetections]:
        copied = ImgDetection()
        copied.label = det.label
        copied.labelName = det.labelName
        copied.confidence = det.confidence

        try:
            copied.setBoundingBox(det.getBoundingBox())
        except Exception:
            pass

        keypoints = det.getKeypoints()
        if len(keypoints) > 0:
            copied.setKeypoints(keypoints)
            edges = det.getEdges()
            if len(edges) > 0:
                copied.setEdges(edges)

        filtered_detections.append(copied)

    dst.detections = filtered_detections
    node.outputs["filtered"].send(dst)
)";

constexpr char kCropConfigScript[] = R"(
from depthai import ImageManipConfig, ImgFrame, Rect, RotatedRect

while True:
    detections_msg = node.inputs["detections"].get()
    if detections_msg is None:
        break

    clear_cfg = ImageManipConfig()
    clear_cfg.setSkipCurrentImage(True)
    node.outputs["config"].send(clear_cfg)

    for index, detection in enumerate(detections_msg.detections):
        outer_bbox = detection.getBoundingBox().getOuterRect()
        x1 = max(0.0, min(outer_bbox[0], 0.9999))
        y1 = max(0.0, min(outer_bbox[1], 0.9999))
        x2 = max(0.0, min(outer_bbox[2], 0.9999))
        y2 = max(0.0, min(outer_bbox[3], 0.9999))

        cfg = ImageManipConfig()
        cfg.addCropRotatedRect(RotatedRect(Rect(x1, y1, x2 - x1, y2 - y1, True)), True)
        cfg.setOutputSize(512, 288, ImageManipConfig.ResizeMode.LETTERBOX)
        cfg.setFrameType(ImgFrame.Type.BGR888i)
        cfg.setReusePreviousImage(True)
        node.outputs["config"].send(cfg)
)";

}  // namespace

/**
 * This Example showcases how to link the following nodes together:
 * - An ImageFrame (in this case the passthrough)
 * - ImgDetections (the output of the first detection network)
 * - Crops from all imgDetections
 * - second stage detections on top of those crops
 *
 *  The final Tree Message should look like this:
 *
 *      ImgFrame (passthrough)
 *         |
 *         \/
 *     ImgDetections (from first stage detection network)
 *     /       |           |             |          \
 *    \/       \/          \/            \/           \/
 *    Crop 0   Crop 1      Crop 2        Crop 3       Crop 4
 *    |         |            |             |             |
 *    \/        \/           \/            \/            \/
 * Pose Dets  Pose Dets     Pose Dets     Pose Dets      Pose Dets (from second stage detection network running on the crops)
 */

int main() {
    // int webSocketPort = 8765;
    // int httpPort = 8082;
    // dai::RemoteConnection remoteConnector(dai::RemoteConnection::DEFAULT_ADDRESS, webSocketPort, true, httpPort);

    dai::Pipeline pipeline;

    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::nullopt, 15.0f);
    dai::NNModelDescription firstStageModelDescription;
    firstStageModelDescription.model = "luxonis/yolov8-instance-segmentation-large:coco-640x480";
    firstStageModelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    auto detNN = pipeline.create<dai::node::DetectionNetwork>()->build(cam, firstStageModelDescription);

    auto* fullResOutput = cam->requestOutput({1920, 1080}, dai::ImgFrame::Type::BGR888i);

    auto filterDetectionsScript = pipeline.create<dai::node::Script>();
    filterDetectionsScript->setScript(kFilterDetectionsScript);
    detNN->out.link(filterDetectionsScript->inputs["detections"]);

    // creates link from ImgFrame --> ImgDetections
    auto gatherImgFrameImgDetection = pipeline.create<dai::node::GatherData>();
    detNN->passthrough.link(gatherImgFrameImgDetection->referenceInput);
    filterDetectionsScript->outputs["filtered"].link(gatherImgFrameImgDetection->collectingInput);

    // Generate crop configs
    auto cropConfigScript = pipeline.create<dai::node::Script>();
    cropConfigScript->inputs["detections"].setMaxSize(20);
    cropConfigScript->setScript(kCropConfigScript);
    gatherImgFrameImgDetection->passthroughCollectingInput.link(cropConfigScript->inputs["detections"]);

    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->inputConfig.setMaxSize(50);
    imageManip->inputImage.setMaxSize(20);
    imageManip->inputConfig.setReusePreviousMessage(false);
    imageManip->setMaxOutputFrameSize(288 * 512 * 3);
    cropConfigScript->outputs["config"].link(imageManip->inputConfig);
    fullResOutput->link(imageManip->inputImage);

    auto gatherCrops = pipeline.create<dai::node::GatherData>();
    imageManip->out.link(gatherCrops->collectingInput);
    gatherImgFrameImgDetection->output.link(gatherCrops->referenceInput);
    gatherCrops->setRunOnHost(false);

    // This node will be implicitly created inside each node as a subnode. For demosntration purposes its explicitly created here
    auto splitCrops = pipeline.create<dai::node::SplitterNode>();
    gatherCrops->output.link(splitCrops->input);
    splitCrops->setRunOnHost(false);

    dai::NNModelDescription poseModelDescription;
    poseModelDescription.model = "luxonis/yolov8-nano-pose-estimation:coco-512x288";
    poseModelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    auto poseDetNN = pipeline.create<dai::node::DetectionNetwork>()->build(splitCrops->output, dai::NNArchive(dai::getModelFromZoo(poseModelDescription)));

    auto gatherPoseDets = pipeline.create<dai::node::GatherData>();
    gatherCrops->output.link(gatherPoseDets->referenceInput);
    poseDetNN->out.link(gatherPoseDets->collectingInput);
    gatherPoseDets->setRunOnHost(false);

    auto collectedQ = gatherPoseDets->output.createOutputQueue();

    pipeline.start();
    // remoteConnector.registerPipeline(pipeline);

    const cv::Scalar color(255, 0, 0);
    const cv::Scalar keypointColor(0, 255, 0);
    std::string lastMessageGroupTree;
    while(pipeline.isRunning()) {
        auto collected = collectedQ->get<dai::MessageGroup>();
        // auto passthrough = passthroughQ->get<dai::ImgFrame>();

        // if(collected == nullptr || detections == nullptr /* || passthrough == nullptr */) {
        //     continue;
        // }

        // std::cout << "Got " << detections->detections.size() << " detections)" << std::endl;
        // std::cout << "Message group has " << collected->getNumMessages() << " messages." << std::endl;

        // auto frame = passthrough->getCvFrame();
        auto frame = collected->get<dai::ImgFrame>(0)->getCvFrame();  // get the original full frame from the message group

        auto detections = collected != nullptr ? collected->get<dai::ImgDetections>(1) : nullptr;
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

        // auto detections = collected != nullptr ? collected->get<dai::ImgDetections>(0) : nullptr;
        size_t shownCropWindowCount = 0;
        for(size_t i = 0; i < detections->detections.size(); ++i) {
            const auto& detection = detections->detections[i];
            // if you are looking for a specific detection, you can filter out here and only then look at the crop message via getChildren(0,
            // static_cast<uint32_t>(i));

            const auto cropNodeIndices = collected->getChildren(1, static_cast<uint32_t>(i));
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

            const auto className = detection.labelName.empty() ? std::to_string(detection.label) : detection.labelName;
            constexpr int labelPadding = 8;
            int baseline = 0;
            const auto labelSize = cv::getTextSize(className, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
            const int labelBannerHeight = labelSize.height + (2 * labelPadding);
            cv::rectangle(cropFrame, cv::Point(0, 0), cv::Point(cropFrame.cols, labelBannerHeight), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(cropFrame, className, cv::Point(labelPadding, labelBannerHeight - labelPadding), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

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
