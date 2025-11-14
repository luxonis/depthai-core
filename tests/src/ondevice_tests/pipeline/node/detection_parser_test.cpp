#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "depthai/depthai.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
    #include <opencv2/imgcodecs.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#include "depthai/common/DetectionParserOptions.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/node/DetectionParser.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

constexpr unsigned int NUM_MSGS = 150;

void validateDetections(const std::shared_ptr<dai::ImgDetections>& detections, const std::filesystem::path& groundTruthPath, int frameNumber) {
    std::ifstream gtFile(groundTruthPath);
    REQUIRE(gtFile.is_open());

    auto groundTruthJson = nlohmann::json::parse(gtFile, nullptr, false);
    REQUIRE_FALSE(groundTruthJson.is_discarded());
    REQUIRE(groundTruthJson.is_array());

    const auto frameIt = std::find_if(groundTruthJson.begin(), groundTruthJson.end(), [frameNumber](const nlohmann::json& frame) {
        return frame.contains("frame_id") && frame["frame_id"].is_number_integer() && frame["frame_id"].get<int>() == frameNumber;
    });

    REQUIRE(frameIt != groundTruthJson.end());
    REQUIRE(frameIt->contains("detections"));
    REQUIRE((*frameIt)["detections"].is_array());

    const auto& gtDetections = (*frameIt)["detections"];

    REQUIRE(detections != nullptr);
    REQUIRE(detections->detections.size() == gtDetections.size());

    constexpr float tolerance = 1e-4F;

    struct GroundTruthDetection {
        struct KeypointCoord {
            float x = 0.F;
            float y = 0.F;
        };

        int label = -1;
        float score = 0.F;
        float xMin = 0.F;
        float yMin = 0.F;
        float xMax = 0.F;
        float yMax = 0.F;
        bool hasKeypoints = false;
        std::vector<KeypointCoord> keypoints;
    };

    std::vector<GroundTruthDetection> parsedGt;
    parsedGt.reserve(gtDetections.size());

    for(std::size_t gtIdx = 0; gtIdx < gtDetections.size(); ++gtIdx) {
        INFO("Ground truth detection index: " << gtIdx);
        const auto& gtDet = gtDetections.at(gtIdx);

        REQUIRE(gtDet.contains("bbox"));
        REQUIRE(gtDet["bbox"].is_array());
        REQUIRE(gtDet["bbox"].size() == 4);

        const auto& bbox = gtDet["bbox"];
        GroundTruthDetection parsed;

        parsed.xMin = bbox.at(0).get<float>();
        parsed.yMin = bbox.at(1).get<float>();
        parsed.xMax = bbox.at(2).get<float>();
        parsed.yMax = bbox.at(3).get<float>();

        parsed.label = gtDet.value("cls", -1);
        parsed.score = gtDet.value("score", 0.F);

        REQUIRE(parsed.label >= 0);

        if(gtDet.contains("keypoints")) {
            parsed.hasKeypoints = true;
            const auto& gtKeypoints = gtDet["keypoints"];
            REQUIRE(gtKeypoints.is_array());
            parsed.keypoints.reserve(gtKeypoints.size());

            for(std::size_t kpIdx = 0; kpIdx < gtKeypoints.size(); ++kpIdx) {
                INFO("Ground truth keypoint index: " << kpIdx);
                const auto& kp = gtKeypoints.at(kpIdx);
                REQUIRE(kp.is_array());
                REQUIRE(kp.size() >= 2);

                GroundTruthDetection::KeypointCoord coord;
                coord.x = kp.at(0).get<float>();
                coord.y = kp.at(1).get<float>();
                parsed.keypoints.push_back(coord);
            }
        }

        parsedGt.push_back(std::move(parsed));
    }

    std::vector<bool> matched(parsedGt.size(), false);

    for(std::size_t detIdx = 0; detIdx < detections->detections.size(); ++detIdx) {
        INFO("Detection index: " << detIdx);
        const dai::ImgDetection& det = detections->detections.at(detIdx);
        const std::vector<dai::Keypoint> detKeypoints = det.getKeypoints();

        bool foundMatch = false;

        for(std::size_t gtIdx = 0; gtIdx < parsedGt.size(); ++gtIdx) {
            if(matched.at(gtIdx)) continue;

            const auto& gt = parsedGt.at(gtIdx);

            if(det.label != static_cast<uint32_t>(gt.label)) continue;
            if(std::fabs(det.confidence - gt.score) > tolerance) continue;
            if(std::fabs(det.xmin - gt.xMin) > tolerance) continue;
            if(std::fabs(det.ymin - gt.yMin) > tolerance) continue;
            if(std::fabs(det.xmax - gt.xMax) > tolerance) continue;
            if(std::fabs(det.ymax - gt.yMax) > tolerance) continue;

            if(gt.hasKeypoints) {
                if(detKeypoints.size() != gt.keypoints.size()) continue;

                bool keypointsMatch = true;
                for(std::size_t kpIdx = 0; kpIdx < gt.keypoints.size(); ++kpIdx) {
                    const auto& gtKp = gt.keypoints.at(kpIdx);
                    const auto& detKp = detKeypoints.at(kpIdx).imageCoordinates;

                    if(std::fabs(detKp.x - gtKp.x) > tolerance || std::fabs(detKp.y - gtKp.y) > tolerance) {
                        keypointsMatch = false;
                        break;
                    }
                }

                if(!keypointsMatch) continue;
            } else if(!detKeypoints.empty()) {
                continue;
            }

            matched.at(gtIdx) = true;
            foundMatch = true;
            break;
        }

        if(!foundMatch) {
            INFO("Failed to match detection label " << det.label << " with confidence " << det.confidence);
        }
        REQUIRE(foundMatch);
    }

    const auto unmatchedIt = std::find(matched.begin(), matched.end(), false);
    if(unmatchedIt != matched.end()) {
        const std::size_t unmatchedIdx = static_cast<std::size_t>(std::distance(matched.begin(), unmatchedIt));
        const auto& gt = parsedGt.at(unmatchedIdx);
        INFO("Unmatched ground truth detection index: " << unmatchedIdx << ", label: " << gt.label << ", score: " << gt.score);
    }
    REQUIRE(std::all_of(matched.begin(), matched.end(), [](bool value) { return value; }));
}

void runDetectionParserReplayTest(const std::string& modelName, const std::filesystem::path& groundTruthPath, const std::filesystem::path& testVideoPath) {
    dai::Pipeline p;
    auto device = p.getDefaultDevice();

    auto description = dai::NNModelDescription{modelName, "RVC4"};
    auto archivePath = dai::getModelFromZoo(description);
    dai::NNArchive nnArchive{archivePath};

    auto size = nnArchive.getInputSize();
    REQUIRE(size.has_value());

    auto replayNode = p.create<dai::node::ReplayVideo>();
    replayNode->setLoop(false);
    replayNode->setFps(30);
    replayNode->setOutFrameType(dai::ImgFrame::Type::BGR888i);
    replayNode->setReplayVideoFile(testVideoPath);
    replayNode->setSize(*size);

    auto nn = p.create<dai::node::NeuralNetwork>()->build(replayNode->out, nnArchive);
    auto detectionParser = p.create<dai::node::DetectionParser>()->build(nn->out, nnArchive);

    auto outputQueue = detectionParser->out.createOutputQueue(4, true);

    p.start();
    for(int i = 0; i < NUM_MSGS; i++) {
        INFO("Frame number: " << i);
        if(!p.isRunning()) break;
        auto detections = outputQueue->get<dai::ImgDetections>();
        REQUIRE(detections != nullptr);
        REQUIRE(detections->getSequenceNum() == i);

        validateDetections(detections, groundTruthPath, i);
    }
}

TEST_CASE("DetectionParser can set properties") {
    dai::node::DetectionParser parser;
    SECTION("Yolo v6 base") {
        auto description = dai::NNModelDescription{"yolov6-nano:r2-coco-512x288:a26d1ee", "RVC4"};
        auto archivePath = dai::getModelFromZoo(description);
        dai::NNArchive nnArchive{archivePath};

        REQUIRE_NOTHROW(parser.setNNArchive(nnArchive));

        dai::DetectionParserOptions properties = parser.properties.parser;

        REQUIRE(properties.decodingFamily == YoloDecodingFamily::TLBR);
        REQUIRE_FALSE(properties.decodeSegmentation);
        REQUIRE_FALSE(properties.decodeKeypoints);
        REQUIRE(properties.classes == 80);
        REQUIRE(properties.anchorsV2.empty());
        REQUIRE_FALSE(properties.nKeypoints.has_value());
    }

    SECTION("Yolo v8 segmentation") {
        auto description = dai::NNModelDescription{"yolov8-instance-segmentation-nano", "RVC4"};
        auto archivePath = dai::getModelFromZoo(description);
        dai::NNArchive nnArchive{archivePath};
        REQUIRE_NOTHROW(parser.setNNArchive(nnArchive));

        dai::DetectionParserOptions properties = parser.properties.parser;

        REQUIRE(properties.decodingFamily == YoloDecodingFamily::TLBR);
        REQUIRE(properties.decodeSegmentation);
        REQUIRE_FALSE(properties.decodeKeypoints);
        REQUIRE(properties.classes == 80);
        REQUIRE(properties.anchorsV2.empty());
        REQUIRE_FALSE(properties.nKeypoints.has_value());
    }

    SECTION("Yolo-P") {
        auto description = dai::NNModelDescription{"yolo-p", "RVC4"};
        auto archivePath = dai::getModelFromZoo(description);
        dai::NNArchive nnArchive{archivePath};
        REQUIRE_NOTHROW(parser.setNNArchive(nnArchive));

        dai::DetectionParserOptions properties = parser.properties.parser;

        REQUIRE(properties.decodingFamily == YoloDecodingFamily::v5AB);
        REQUIRE_FALSE(properties.decodeSegmentation);
        REQUIRE_FALSE(properties.decodeKeypoints);
        REQUIRE(properties.classes == 1);
        REQUIRE(properties.anchorsV2.size() == 3);
        REQUIRE_FALSE(properties.nKeypoints.has_value());
    }

    SECTION("Yolo v8 pose estimation") {
        auto description = dai::NNModelDescription{"yolov8-large-pose-estimation", "RVC4"};
        auto archivePath = dai::getModelFromZoo(description);
        dai::NNArchive nnArchive{archivePath};
        REQUIRE_NOTHROW(parser.setNNArchive(nnArchive));

        dai::DetectionParserOptions properties = parser.properties.parser;

        REQUIRE(properties.decodingFamily == YoloDecodingFamily::TLBR);
        REQUIRE_FALSE(properties.decodeSegmentation);
        REQUIRE(properties.decodeKeypoints);
        REQUIRE(properties.classes == 1);
        REQUIRE(properties.anchorsV2.empty());
        REQUIRE(properties.nKeypoints.has_value());
    }
}

TEST_CASE("DetectionParser replay test") {
    const std::filesystem::path yoloV6R2Coco512x288GroundTruth{YOLO_V6_R2_COCO_512x288_GROUND_TRUTH};
    const std::filesystem::path yoloV6R2Coco512x384GroundTruth{YOLO_V6_R2_COCO_512x384_GROUND_TRUTH};
    const std::filesystem::path yoloV8InstanceSegmentationNanoCoco512x288GroundTruth{YOLO_V8_INSTANCE_SEGMENTATION_NANO_COCO_512x288_GROUND_TRUTH};
    const std::filesystem::path yoloV8LargePoseEstimationCoco640x352GroundTruth{YOLO_V8_LARGE_POSE_ESTIMATION_COCO_640x352_GROUND_TRUTH};
    const std::filesystem::path yoloV8InstanceSegmentationLargeCoco640x352GroundTruth{YOLO_V8_INSTANCE_SEGMENTATION_LARGE_COCO_640x352_GROUND_TRUTH};
    const std::filesystem::path yoloV10NanoCoco512x288GroundTruth{YOLO_V10_NANO_COCO_512x288_GROUND_TRUTH};
    const std::filesystem::path ppeDetection640x640GroundTruth{PPE_DETECTION_640x640_GROUND_TRUTH};
    const std::filesystem::path yoloPBdd100k320x320GroundTruth{YOLO_P_BDD100K_320x320_GROUND_TRUTH};
    const std::filesystem::path fireDetection512x288GroundTruth{FIRE_DETECTION_512x288_GROUND_TRUTH};

    const std::filesystem::path peopleWalkingVideo{PEOPLE_WALKING_VIDEO};
    const std::filesystem::path fireVideo{FIRE_VIDEO};

    const std::vector<std::tuple<std::string, std::filesystem::path, std::filesystem::path>> testCases = {
        {"yolov6-nano:r2-coco-512x288:a26d1ee", yoloV6R2Coco512x288GroundTruth, peopleWalkingVideo},
        {"yolov6-nano:r2-coco-512x384:fb1429e", yoloV6R2Coco512x384GroundTruth, peopleWalkingVideo},
        {"yolov8-instance-segmentation-nano:coco-512x288:6c0402a", yoloV8InstanceSegmentationNanoCoco512x288GroundTruth, peopleWalkingVideo},
        {"yolov8-large-pose-estimation:coco-640x352:1868e39", yoloV8LargePoseEstimationCoco640x352GroundTruth, peopleWalkingVideo},
        {"yolov8-instance-segmentation-large:coco-640x352:701031f", yoloV8InstanceSegmentationLargeCoco640x352GroundTruth, peopleWalkingVideo},
        {"yolov10-nano:coco-512x288:007b5fe", yoloV10NanoCoco512x288GroundTruth, peopleWalkingVideo},
        {"ppe-detection:640x640:419a4e5", ppeDetection640x640GroundTruth, peopleWalkingVideo},
        {"yolo-p:bdd100k-320x320:7019bb8", yoloPBdd100k320x320GroundTruth, peopleWalkingVideo},
        {"fire-detection:512x288:4a8263c", fireDetection512x288GroundTruth, fireVideo}};

    for(const auto& testCase : testCases) {
        INFO("Running DetectionParser replay test for model: " << std::get<0>(testCase));

        runDetectionParserReplayTest(std::get<0>(testCase), std::get<1>(testCase), std::get<2>(testCase));
    }
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
TEST_CASE("DetectionParser segmentation mask test") {
    const std::string modelName = "yolov8-instance-segmentation-large:coco-640x352:701031f";

    const std::filesystem::path kitchenImagePath{KITCHEN_IMAGE_PATH};
    const std::string segmentationGroundTruth = YOLO_V8_INSTANCE_SEGMENTATION_LARGE_COCO_640x352_KITCHEN_SEGMENTATION_GROUND_TRUTH;

    cv::Mat kitchenGtSegmentation = cv::imread(segmentationGroundTruth, cv::IMREAD_GRAYSCALE);
    REQUIRE_FALSE(kitchenGtSegmentation.empty());

    cv::Mat kitchenImage = cv::imread(kitchenImagePath.string(), cv::IMREAD_COLOR);
    REQUIRE_FALSE(kitchenImage.empty());

    dai::Pipeline p;
    auto device = p.getDefaultDevice();

    auto description = dai::NNModelDescription{modelName, "RVC4"};
    auto archivePath = dai::getModelFromZoo(description);
    dai::NNArchive nnArchive{archivePath};

    const auto inputSize = nnArchive.getInputSize();
    REQUIRE(inputSize.has_value());

    const cv::Size networkSize{static_cast<int>(inputSize->first), static_cast<int>(inputSize->second)};
    cv::resize(kitchenImage, kitchenImage, networkSize, 0.0, 0.0, cv::INTER_AREA);

    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setModelPath(archivePath);

    auto detectionParser = p.create<dai::node::DetectionParser>()->build(nn->out, nnArchive);

    auto nnInput = nn->input.createInputQueue();
    auto outputQueue = detectionParser->out.createOutputQueue();

    auto inputFrame = std::make_shared<dai::ImgFrame>();
    auto transformation = dai::ImgTransformation{inputSize->first, inputSize->second};
    inputFrame->setCvFrame(kitchenImage, dai::ImgFrame::Type::BGR888i);
    inputFrame->setTimestamp(std::chrono::steady_clock::now());
    inputFrame->setSequenceNum(0);
    inputFrame->transformation = transformation;

    p.start();
    REQUIRE(p.isRunning());

    nnInput->send(inputFrame);

    auto detections = outputQueue->get<dai::ImgDetections>();
    REQUIRE(detections != nullptr);

    std::optional<cv::Mat> optSegmentationMask = detections->getCvSegmentationMask();
    REQUIRE(optSegmentationMask.has_value());
    cv::Mat segmentationMask = *optSegmentationMask;
    REQUIRE_FALSE(segmentationMask.empty());
    REQUIRE(detections->getSegmentationMaskWidth() == kitchenGtSegmentation.cols);
    REQUIRE(detections->getSegmentationMaskHeight() == kitchenGtSegmentation.rows);

    REQUIRE(segmentationMask.type() == kitchenGtSegmentation.type());

    cv::Mat diff;
    cv::absdiff(segmentationMask, kitchenGtSegmentation, diff);
    REQUIRE(cv::countNonZero(diff) <= 5000);
}
#endif
