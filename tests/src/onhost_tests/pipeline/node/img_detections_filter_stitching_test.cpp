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
    std::array<dai::ImgTransformation, 2> cameras = {transformation(512, 512, rotation(-30, true)), transformation(512, 512, rotation(30, true))};
    for(auto& camera : cameras) {
        auto extrinsics = camera.getExtrinsics();
        extrinsics.toDeviceId = "test-rig";
        camera.setExtrinsics(extrinsics);
    }
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

TEST_CASE("ImgDetectionsFilter encloses curved edges in stitched images", "[ImgDetectionsFilter][Stitching][RM-3][RM-4]") {
    auto model = dai::CameraModel::Perspective;
    SECTION("perspective control") {}
    SECTION("cylindrical") {
        model = dai::CameraModel::Cylindrical;
    }
    SECTION("equirectangular") {
        model = dai::CameraModel::Equirectangular;
    }
    FilterHarness h({}, {"a", "b"});
    auto stitching = h.pipeline.create<dai::node::Stitching>()->build(2);
    stitching->setCameraModel(model);
    stitching->setSeamFinder(dai::node::Stitching::SeamFinder::NONE);
    h.linkReference(stitching->out);
    const auto panoramaQueue = stitching->out.createOutputQueue();
    std::array<std::shared_ptr<dai::InputQueue>, 2> imageQueues = {stitching->inputs["input0"].createInputQueue(),
                                                                   stitching->inputs["input1"].createInputQueue()};
    auto camera = transformation();
    auto extrinsics = camera.getExtrinsics();
    extrinsics.toDeviceId = "test-rig";
    camera.setExtrinsics(extrinsics);
    h.pipeline.start();
    for(std::size_t i = 0; i < imageQueues.size(); ++i) {
        cv::Mat pixels(512, 512, CV_8UC3, cv::Scalar(0, 0, 0));
        // The midpoint of the top edge projects above both corners on curved panorama surfaces.
        if(i == 1) cv::rectangle(pixels, cv::Rect(128, 56, 257, 201), cv::Scalar(0, 255, 0), cv::FILLED);
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setCvFrame(pixels, dai::ImgFrame::Type::BGR888i);
        frame->setTransformation(camera);
        frame->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::seconds(10)));
        imageQueues[i]->send(frame);
    }
    bool timedOut = false;
    const auto panorama = panoramaQueue->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);
    h.send(message(camera, {}), "a");
    h.send(message(camera, {{256, 156, 256, 200, .9f, 1}}), "b");
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 1);
    REQUIRE(out->getTransformation().has_value());
    REQUIRE(out->getTransformation()->isEqualTransformation(panorama->getTransformation()));
    cv::Mat selected;
    cv::inRange(panorama->getCvFrame(), cv::Scalar(0, 128, 0), cv::Scalar(64, 255, 64), selected);
    REQUIRE(cv::countNonZero(selected) > 0);
    const auto visible = cv::boundingRect(selected);
    const auto outer = out->detections[0].getBoundingBox().denormalize(panorama->getWidth(), panorama->getHeight()).getOuterRect();
    REQUIRE_THAT(outer[0], Catch::Matchers::WithinAbs(visible.x, 1.5));
    REQUIRE_THAT(outer[1], Catch::Matchers::WithinAbs(visible.y, 1.5));
    REQUIRE_THAT(outer[2], Catch::Matchers::WithinAbs(visible.x + visible.width - 1, 1.5));
    REQUIRE_THAT(outer[3], Catch::Matchers::WithinAbs(visible.y + visible.height - 1, 1.5));
}

TEST_CASE("ImgDetectionsFilter chooses boxes from visible panorama sources", "[ImgDetectionsFilter][Stitching]") {
    bool useMasks = true;
    auto seamFinder = dai::node::Stitching::SeamFinder::NONE;
    FilterSettings settings;
    settings.mode = Mode::NMS;
    SECTION("direct composition") {}
    SECTION("blended composition") {
        seamFinder = dai::node::Stitching::SeamFinder::VORONOI;
    }
    SECTION("averaging also excludes hidden sources") {
        settings.mode = Mode::AVERAGE;
    }
    SECTION("unlinked masks preserve confidence selection") {
        useMasks = false;
    }
    FilterHarness h(settings, {"a", "b"});
    auto stitching = h.pipeline.create<dai::node::Stitching>()->build(2);
    stitching->setCameraModel(dai::CameraModel::Perspective);
    stitching->setSeamFinder(seamFinder);
    h.linkReference(stitching->out);
    if(useMasks) {
        stitching->outSourceMasks["input0"].link(h.node->inputSourceMasks["a"]);
        stitching->outSourceMasks["input1"].link(h.node->inputSourceMasks["b"]);
    }
    const auto panoramaQueue = stitching->out.createOutputQueue();
    const auto mask0Queue = stitching->outSourceMasks["input0"].createOutputQueue();
    const auto mask1Queue = stitching->outSourceMasks["input1"].createOutputQueue();
    std::array<std::shared_ptr<dai::InputQueue>, 2> imageQueues = {stitching->inputs["input0"].createInputQueue(),
                                                                   stitching->inputs["input1"].createInputQueue()};
    auto camera = transformation();
    auto extrinsics = camera.getExtrinsics();
    extrinsics.toDeviceId = "test-rig";
    camera.setExtrinsics(extrinsics);
    h.pipeline.start();
    for(std::size_t i = 0; i < imageQueues.size(); ++i) {
        cv::Mat pixels(512, 512, CV_8UC3, cv::Scalar(32, 32, 32));
        // Simulate parallax: the hidden camera sees the same object 20 pixels to the right.
        cv::rectangle(pixels, cv::Rect(i == 0 ? 236 : 216, 216, 81, 81), i == 0 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), cv::FILLED);
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setCvFrame(pixels, dai::ImgFrame::Type::BGR888i);
        frame->setTransformation(camera);
        frame->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::seconds(10)));
        imageQueues[i]->send(frame);
    }
    bool timedOut = false;
    const auto panorama = panoramaQueue->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);
    const auto hidden = mask0Queue->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(hidden != nullptr);
    const auto visible = mask1Queue->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(visible != nullptr);
    REQUIRE(hidden->getTransformation().isEqualTransformation(panorama->getTransformation()));
    REQUIRE(visible->getTransformation().isEqualTransformation(panorama->getTransformation()));
    REQUIRE(hidden->getCvFrame().at<std::uint8_t>(256, 276) == 0);
    REQUIRE(visible->getCvFrame().at<std::uint8_t>(256, 256) > 0);
    // The hidden source has higher confidence; gating must happen before NMS or averaging.
    for(int round = 0; round < 2; ++round) {
        h.send(message(camera, {{276, 256, 80, 80, .9f, 1}}), "a");
        h.send(message(camera, {{256, 256, 80, 80, .8f, 1}}), "b");
        const auto out = h.receive();
        REQUIRE(out->detections.size() == 1);
        const auto box = out->detections[0].getBoundingBox().denormalize(panorama->getWidth(), panorama->getHeight());
        REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs(useMasks ? 256 : 276, .2));
        REQUIRE_THAT(out->detections[0].confidence, Catch::Matchers::WithinAbs(useMasks ? .8 : .9, .001));
        cv::Mat green;
        cv::inRange(panorama->getCvFrame(), cv::Scalar(0, 64, 0), cv::Scalar(64, 255, 64), green);
        REQUIRE(cv::countNonZero(green) > 0);
        if(useMasks) {
            const auto bounds = cv::boundingRect(green);
            REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs(bounds.x + bounds.width / 2.0, 1.5));
        }
    }
}

TEST_CASE("ImgDetectionsFilter latches and updates optional source masks", "[ImgDetectionsFilter][Stitching]") {
    FilterSettings settings;
    settings.reference = transformation();
    FilterHarness h(settings);
    const auto masks = h.node->inputSourceMasks["cam"].createInputQueue();
    h.pipeline.start();
    h.send(message());
    h.requireNoOutput();
    auto mask = std::make_shared<dai::ImgFrame>();
    cv::Mat pixels(512, 512, CV_8U, cv::Scalar(255));
    pixels(cv::Rect(0, 0, 256, 512)).setTo(0);
    mask->setCvFrame(pixels, dai::ImgFrame::Type::GRAY8);
    mask->setTransformation(transformation());
    masks->send(mask);
    for(int round = 0; round < 2; ++round) {
        h.send(message(transformation(), {{100, 100, 64, 64, .9f, 1}, {400, 100, 64, 64, .8f, 2}}));
        requireOutput(*h.receive(), transformation(), {{400, 100, 64, 64, .8f, 2}});
    }
    auto replacement = std::make_shared<dai::ImgFrame>();
    replacement->setCvFrame(cv::Mat::zeros(512, 512, CV_8U), dai::ImgFrame::Type::GRAY8);
    replacement->setTransformation(transformation());
    masks->send(replacement);
    h.send(message(transformation(), {{400, 100, 64, 64, .8f, 2}}));
    REQUIRE(h.receive()->detections.empty());
}
