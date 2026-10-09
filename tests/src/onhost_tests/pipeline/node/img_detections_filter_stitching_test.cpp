#include <limits>
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

TEST_CASE("ImgDetectionsFilter retains a person spanning a seam with both centers hidden", "[ImgDetectionsFilter][Stitching]") {
    FilterSettings settings;
    settings.reference = transformation();
    settings.mode = Mode::OFF;
    SECTION("duplicates off") {}
    SECTION("NMS") {
        settings.mode = Mode::NMS;
    }
    SECTION("average") {
        settings.mode = Mode::AVERAGE;
    }
    FilterHarness h(settings, {"a", "b"});
    const std::array masks = {h.node->inputSourceMasks["a"].createInputQueue(), h.node->inputSourceMasks["b"].createInputQueue()};
    h.pipeline.start();
    for(std::size_t i = 0; i < masks.size(); ++i) {
        cv::Mat pixels(512, 512, CV_8U, cv::Scalar(0));
        pixels(cv::Rect(i == 0 ? 0 : 256, 0, 256, 512)).setTo(255);
        // Parallax puts each box center on the other camera's side of the seam.
        REQUIRE(pixels.at<std::uint8_t>(256, i == 0 ? 280 : 232) == 0);
        auto mask = std::make_shared<dai::ImgFrame>();
        mask->setCvFrame(pixels, dai::ImgFrame::Type::GRAY8);
        mask->setTransformation(*settings.reference);
        masks[i]->send(mask);
    }
    for(int round = 0; round < 2; ++round) {
        h.send(message(transformation(), {{280, 256, 120, 400, .93f, 0, 0, "person"}}), "a");
        h.send(message(transformation(), {{232, 256, 120, 400, .95f, 0, 0, "person"}}), "b");
        std::vector<ExpectedDetection> expected = {{232, 256, 120, 400, .95f, 0, 0, "person"}};
        if(settings.mode == Mode::OFF)
            expected.insert(expected.begin(), {280, 256, 120, 400, .93f, 0, 0, "person"});
        else if(settings.mode == Mode::AVERAGE)
            expected[0].x = (280 * .93f + 232 * .95f) / (.93f + .95f);
        requireOutput(*h.receive(), *settings.reference, expected);
    }
}

TEST_CASE("ImgDetectionsFilter uses visible area for duplicates and confidence for count limits", "[ImgDetectionsFilter][Stitching]") {
    FilterSettings settings;
    settings.reference = transformation();
    settings.mode = Mode::NMS;
    float confidenceA = .95f, confidenceB = .6f, expectedX = 240;
    SECTION("NMS prefers visibility over confidence") {}
    SECTION("average weights visibility and confidence") {
        settings.mode = Mode::AVERAGE;
        expectedX = (260 * confidenceA * .25f + 240 * confidenceB) / (confidenceA * .25f + confidenceB);
    }
    SECTION("zero-confidence average weights visibility") {
        settings.mode = Mode::AVERAGE;
        confidenceA = confidenceB = 0;
        expectedX = 244;
    }
    SECTION("very small confidence retains finite averaging weights") {
        settings.mode = Mode::AVERAGE;
        confidenceA = confidenceB = std::numeric_limits<float>::denorm_min();
        expectedX = 244;
    }
    SECTION("maximum count still selects highest confidence") {
        settings.mode = Mode::OFF;
        settings.maxDetections = 1;
        expectedX = 260;
    }
    FilterHarness h(settings, {"a", "b"});
    const auto masks = h.node->inputSourceMasks["a"].createInputQueue();
    auto mask = std::make_shared<dai::ImgFrame>();
    cv::Mat pixels(512, 512, CV_8U, cv::Scalar(0));
    pixels(cv::Rect(250, 0, 20, 512)).setTo(255);
    mask->setCvFrame(pixels, dai::ImgFrame::Type::GRAY8);
    mask->setTransformation(*settings.reference);
    h.pipeline.start();
    masks->send(mask);
    h.send(message(transformation(), {{260, 256, 80, 160, confidenceA, 0}}), "a");
    h.send(message(transformation(), {{240, 256, 80, 160, confidenceB, 0}}), "b");
    requireOutput(*h.receive(), *settings.reference, {{expectedX, 256, 80, 160, settings.mode == Mode::OFF ? confidenceA : confidenceB, 0}});
}

TEST_CASE("ImgDetectionsFilter source visibility respects rotated and clipped footprints", "[ImgDetectionsFilter][Stitching]") {
    FilterSettings settings;
    settings.reference = transformation();
    FilterHarness h(settings);
    const auto masks = h.node->inputSourceMasks["cam"].createInputQueue();
    cv::Mat pixels(512, 512, CV_8U, cv::Scalar(0));
    ExpectedDetection detection{256, 256, 80, 40, .9f, 0, 45};
    bool visible = true;
    SECTION("visible away from the center") {
        pixels.at<std::uint8_t>(256, 270) = 255;
    }
    SECTION("pixel in enclosing bounds but outside the rotated box") {
        pixels.at<std::uint8_t>(220, 220) = 255;
        visible = false;
    }
    SECTION("center outside image but part of the box remains visible") {
        pixels.setTo(255);
        detection = {-16, 256, 128, 128, .9f, 0};
    }
    auto mask = std::make_shared<dai::ImgFrame>();
    mask->setCvFrame(pixels, dai::ImgFrame::Type::GRAY8);
    mask->setTransformation(*settings.reference);
    // Exercise stride and plane offsets for every section, including clipping at x=0.
    mask->setStride(520);
    mask->fb.p1Offset = 3;
    std::vector<std::uint8_t> data(3 + 520 * 512, 0);
    for(int y = 0; y < pixels.rows; ++y) std::copy_n(pixels.ptr<std::uint8_t>(y), pixels.cols, data.begin() + 3 + y * 520);
    mask->setData(data);
    h.pipeline.start();
    masks->send(mask);
    h.send(message(transformation(), {detection}));
    requireOutput(*h.receive(), *settings.reference, visible ? std::vector<ExpectedDetection>{detection} : std::vector<ExpectedDetection>{});
}
