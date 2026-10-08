#include <opencv2/imgproc.hpp>

#include "depthai/pipeline/node/Stitching.hpp"
#include "img_detections_filter_test_harness.hpp"

using namespace filtertest;

TEST_CASE("ImgDetectionsFilter I-1: detections align with calibrated colored squares", "[ImgDetectionsFilter][RM-14]") {
    FilterHarness h({}, {"a", "b"});
    // Stitching is in the public node namespace in this tree.
    auto stitching = h.pipeline.create<dai::node::Stitching>()->build(2);
    stitching->setMode(dai::node::Stitching::Mode::PANORAMA);
    stitching->setUseInputCalibration(true);
    stitching->setSeamFinder(dai::node::Stitching::SeamFinder::NONE);
    // Deliver the reference before the observation queue, so its read is the synchronization point.
    h.linkReference(stitching->out);
    const auto panoramaQueue = stitching->out.createOutputQueue();
    std::array<std::shared_ptr<dai::InputQueue>, 2> imageQueues = {stitching->inputs["input0"].createInputQueue(),
                                                                   stitching->inputs["input1"].createInputQueue()};
    const std::array<dai::ImgTransformation, 2> cameras = {transformation(512, 512, rotation(-30, true)), transformation(512, 512, rotation(30, true))};
    h.pipeline.start();
    for(std::size_t i = 0; i < cameras.size(); ++i) {
        cv::Mat pixels(512, 512, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::rectangle(pixels, cv::Rect(216, 216, 80, 80), i == 0 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), cv::FILLED);
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setCvFrame(pixels, dai::ImgFrame::Type::BGR888i);
        frame->transformation = cameras[i];
        frame->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::seconds(10)));
        imageQueues[i]->send(frame);
    }
    bool timedOut = false;
    const auto panorama = panoramaQueue->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);
    h.send(message(cameras[0], {{256, 256, 80, 80, .9f, 0}}), "a");
    h.send(message(cameras[1], {{256, 256, 80, 80, .9f, 1}}), "b");
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 2);
    REQUIRE(out->getTransformation().has_value());
    REQUIRE(out->getTransformation()->isEqualTransformation(panorama->transformation));
    const auto pixels = panorama->getCvFrame();
    for(std::size_t i = 0; i < out->detections.size(); ++i) {
        CAPTURE(i);
        REQUIRE(out->detections[i].label == i);
        cv::Mat selected;
        cv::inRange(pixels, i == 0 ? cv::Scalar(0, 0, 128) : cv::Scalar(0, 128, 0), i == 0 ? cv::Scalar(64, 64, 255) : cv::Scalar(64, 255, 64), selected);
        REQUIRE(cv::countNonZero(selected) > 0);
        const cv::Rect2f colorBounds = cv::boundingRect(selected);
        const auto outer = out->detections[i].getBoundingBox().denormalize(panorama->getWidth(), panorama->getHeight()).getOuterRect();
        const cv::Rect2f detectionBounds(outer[0], outer[1], outer[2] - outer[0], outer[3] - outer[1]);
        const float intersection = (colorBounds & detectionBounds).area();
        const float unionArea = colorBounds.area() + detectionBounds.area() - intersection;
        REQUIRE(unionArea > 0);
        REQUIRE(intersection / unionArea >= .7f);
    }
}
