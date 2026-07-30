#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/node/host/Stitching.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace {

constexpr int VIEW_WIDTH = 640;
constexpr int VIEW_HEIGHT = 480;
constexpr double FOCAL = 600.0;

/**
 * Render the view a pinhole camera with the given yaw would see of a distant scene, so that the
 * resulting images are related by a pure rotation and can be stitched with the panorama model.
 */
cv::Mat renderView(const cv::Mat& scene, double yawDegrees) {
    cv::Mat viewIntrinsics = (cv::Mat_<double>(3, 3) << FOCAL, 0, VIEW_WIDTH / 2.0, 0, FOCAL, VIEW_HEIGHT / 2.0, 0, 0, 1);
    cv::Mat sceneIntrinsics = (cv::Mat_<double>(3, 3) << FOCAL, 0, scene.cols / 2.0, 0, FOCAL, scene.rows / 2.0, 0, 0, 1);

    const double yaw = yawDegrees * CV_PI / 180.0;
    cv::Mat rotation = (cv::Mat_<double>(3, 3) << std::cos(yaw), 0, std::sin(yaw), 0, 1, 0, -std::sin(yaw), 0, std::cos(yaw));

    cv::Mat view;
    cv::warpPerspective(
        scene, view, sceneIntrinsics * rotation * viewIntrinsics.inv(), cv::Size(VIEW_WIDTH, VIEW_HEIGHT), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);
    return view;
}

std::shared_ptr<dai::ImgFrame> toFrame(const cv::Mat& image, int64_t sequenceNum) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setCvFrame(image, dai::ImgFrame::Type::BGR888i);
    frame->setSequenceNum(sequenceNum);
    frame->setTimestamp(std::chrono::steady_clock::now());
    return frame;
}

}  // namespace

TEST_CASE("Stitching rejects fewer than two inputs", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::node::Stitching>();

    REQUIRE_THROWS(stitching->build(1));
}

TEST_CASE("Stitching stitches panoramas by default", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::node::Stitching>();

    REQUIRE(stitching->getMode() == dai::node::Stitching::Mode::PANORAMA);
}

TEST_CASE("Stitching combines three rotated views into a wider panorama", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> views = {renderView(scene, -15.0), renderView(scene, 0.0), renderView(scene, 15.0)};

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::node::Stitching>()->build(views.size());
    stitching->setCameraModel(dai::node::Stitching::CameraModel::CYLINDRICAL);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    for(size_t i = 0; i < views.size(); ++i) {
        inputQueues.push_back(stitching->inputs["input" + std::to_string(i)].createInputQueue());
    }
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    for(size_t i = 0; i < views.size(); ++i) {
        inputQueues[i]->send(toFrame(views[i], 0));
    }

    auto panorama = output->get<dai::ImgFrame>();
    pipeline.stop();

    REQUIRE(panorama != nullptr);
    REQUIRE(panorama->getType() == dai::ImgFrame::Type::BGR888i);
    // A panorama of three views 15 degrees apart is wider than a single view, but a degenerate
    // registration blows the canvas up to many times the view size
    REQUIRE(panorama->getWidth() > static_cast<unsigned int>(VIEW_WIDTH));
    REQUIRE(panorama->getWidth() < static_cast<unsigned int>(2 * VIEW_WIDTH));
    REQUIRE(panorama->getHeight() > static_cast<unsigned int>(VIEW_HEIGHT * 3 / 4));
}

TEST_CASE("Stitching re-estimates every frame when continuous", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> views = {renderView(scene, -6.0), renderView(scene, 6.0)};

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::node::Stitching>()->build(views.size());
    stitching->setContinuous(true);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    for(size_t i = 0; i < views.size(); ++i) {
        inputQueues.push_back(stitching->inputs["input" + std::to_string(i)].createInputQueue());
    }
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    for(size_t i = 0; i < views.size(); ++i) {
        inputQueues[i]->send(toFrame(views[i], 7));
    }

    auto panorama = output->get<dai::ImgFrame>();
    pipeline.stop();

    REQUIRE(panorama != nullptr);
    REQUIRE(panorama->getWidth() > static_cast<unsigned int>(VIEW_WIDTH));
    REQUIRE(panorama->getWidth() < static_cast<unsigned int>(2 * VIEW_WIDTH));
    // Metadata comes from the first input
    REQUIRE(panorama->getSequenceNum() == 7);
}

TEST_CASE("Stitching reuses the averaged transform once the estimation frames are consumed", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> views = {renderView(scene, -6.0), renderView(scene, 6.0)};
    // Views the transform is deliberately not estimated for, to show it is no longer re-estimated
    const std::vector<cv::Mat> unseenViews = {renderView(scene, -10.0), renderView(scene, 10.0)};
    constexpr uint32_t ESTIMATION_FRAMES = 2;
    constexpr int64_t NUM_GROUPS = 4;

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::node::Stitching>()->build(views.size());
    stitching->setContinuous(false);
    stitching->setEstimationFrames(ESTIMATION_FRAMES);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    for(size_t i = 0; i < views.size(); ++i) {
        inputQueues.push_back(stitching->inputs["input" + std::to_string(i)].createInputQueue());
    }
    auto output = stitching->out.createOutputQueue();

    pipeline.start();

    unsigned int fixedWidth = 0;
    for(int64_t group = 0; group < NUM_GROUPS; ++group) {
        const bool transformFixed = group >= ESTIMATION_FRAMES;
        // Once the transform is fixed, images the node never estimated on still get warped by it
        const auto& groupViews = group == NUM_GROUPS - 1 ? unseenViews : views;
        for(size_t i = 0; i < groupViews.size(); ++i) {
            inputQueues[i]->send(toFrame(groupViews[i], group));
        }

        auto panorama = output->get<dai::ImgFrame>();
        REQUIRE(panorama != nullptr);
        REQUIRE(panorama->getSequenceNum() == group);

        if(!transformFixed) continue;
        if(fixedWidth == 0) {
            fixedWidth = panorama->getWidth();
        } else {
            REQUIRE(panorama->getWidth() == fixedWidth);
        }
    }

    pipeline.stop();
}
