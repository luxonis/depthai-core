#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <depthai/beta/node/Stitching.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
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

dai::ImgTransformation calibratedTransformation(double yawDegrees,
                                                const std::string& originDeviceId = "reference-device",
                                                std::vector<float> distortionCoefficients = {}) {
    const double yaw = yawDegrees * CV_PI / 180.0;
    const std::vector<std::vector<float>> rotation = {
        {static_cast<float>(std::cos(yaw)), 0.0f, static_cast<float>(std::sin(yaw))},
        {0.0f, 1.0f, 0.0f},
        {static_cast<float>(-std::sin(yaw)), 0.0f, static_cast<float>(std::cos(yaw))},
    };
    // Deliberately different centers: calibrated panorama mode must ignore translation.
    dai::Extrinsics extrinsics(rotation, {static_cast<float>(yawDegrees), 50.0f, -25.0f}, dai::CameraBoardSocket::CAM_A);
    extrinsics.toDeviceId = originDeviceId;
    const std::array<std::array<float, 3>, 3> intrinsics = {
        {{static_cast<float>(FOCAL), 0.0f, VIEW_WIDTH / 2.0f}, {0.0f, static_cast<float>(FOCAL), VIEW_HEIGHT / 2.0f}, {0.0f, 0.0f, 1.0f}}};
    return {VIEW_WIDTH, VIEW_HEIGHT, intrinsics, dai::CameraModel::Perspective, std::move(distortionCoefficients), extrinsics};
}

std::shared_ptr<dai::ImgFrame> toCalibratedFrame(const cv::Mat& image, double yawDegrees, int64_t sequenceNum) {
    auto frame = toFrame(image, sequenceNum);
    frame->getTransformation() = calibratedTransformation(yawDegrees);
    return frame;
}

}  // namespace

TEST_CASE("Stitching rejects fewer than two inputs", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>();

    REQUIRE_THROWS(stitching->build(1));
}

TEST_CASE("Stitching stitches panoramas by default", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>();

    REQUIRE(stitching->getMode() == dai::beta::node::Stitching::Mode::PANORAMA);
}

TEST_CASE("Stitching uses input calibration to compose a cylindrical panorama", "[Stitching]") {
    const std::vector<double> yaws = {-15.0, 0.0, 15.0};
    const std::vector<cv::Mat> featurelessViews = {
        cv::Mat(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, cv::Scalar(40, 60, 80)),
        cv::Mat(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, cv::Scalar(80, 60, 40)),
        cv::Mat(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, cv::Scalar(60, 80, 40)),
    };

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(featurelessViews.size());
    stitching->setUseInputCalibration(true);
    stitching->setCameraModel(dai::beta::node::Stitching::CameraModel::CYLINDRICAL);
    stitching->setSeamFinder(dai::beta::node::Stitching::SeamFinder::NONE);
    stitching->setSyncThreshold(std::chrono::seconds(1));
    REQUIRE(stitching->getUseInputCalibration());

    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    for(size_t i = 0; i < featurelessViews.size(); ++i) {
        inputQueues.push_back(stitching->inputs["input" + std::to_string(i)].createInputQueue());
    }
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    for(size_t i = 0; i < featurelessViews.size(); ++i) {
        inputQueues[i]->send(toCalibratedFrame(featurelessViews[i], yaws[i], 11));
    }

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    pipeline.stop();

    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);
    REQUIRE(panorama->getSequenceNum() == 11);
    REQUIRE(panorama->getWidth() > static_cast<unsigned int>(VIEW_WIDTH));
    REQUIRE(panorama->getWidth() < static_cast<unsigned int>(2 * VIEW_WIDTH));
    REQUIRE(panorama->getHeight() >= static_cast<unsigned int>(VIEW_HEIGHT * 0.95));
    REQUIRE(panorama->getHeight() <= static_cast<unsigned int>(VIEW_HEIGHT));
}

TEST_CASE("Calibrated panorama directly copies overlapping inputs", "[Stitching]") {
    const cv::Vec3b firstColor(20, 40, 60);
    const cv::Vec3b secondColor(180, 200, 220);
    const cv::Mat firstImage(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, firstColor);
    const cv::Mat secondImage(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, secondColor);

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(2);
    stitching->setUseInputCalibration(true);
    stitching->setCameraModel(dai::beta::node::Stitching::CameraModel::CYLINDRICAL);
    stitching->setSeamFinder(dai::beta::node::Stitching::SeamFinder::NONE);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    auto firstInput = stitching->inputs["input0"].createInputQueue();
    auto secondInput = stitching->inputs["input1"].createInputQueue();
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    firstInput->send(toCalibratedFrame(firstImage, 0.0, 12));
    secondInput->send(toCalibratedFrame(secondImage, 0.0, 12));

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    pipeline.stop();

    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);
    const auto result = panorama->getCvFrame();
    REQUIRE(result.at<cv::Vec3b>(result.rows / 2, result.cols / 2) == secondColor);
}

TEST_CASE("Calibrated panorama blends overlapping inputs when seam finding is enabled", "[Stitching]") {
    const cv::Vec3b firstColor(20, 40, 60);
    const cv::Vec3b secondColor(180, 200, 220);
    const cv::Mat firstImage(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, firstColor);
    const cv::Mat secondImage(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, secondColor);

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(2);
    stitching->setUseInputCalibration(true);
    stitching->setCameraModel(dai::beta::node::Stitching::CameraModel::CYLINDRICAL);
    stitching->setSeamFinder(dai::beta::node::Stitching::SeamFinder::GRAPHCUT_COLOR);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    auto firstInput = stitching->inputs["input0"].createInputQueue();
    auto secondInput = stitching->inputs["input1"].createInputQueue();
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    firstInput->send(toCalibratedFrame(firstImage, 0.0, 14));
    secondInput->send(toCalibratedFrame(secondImage, 0.0, 14));

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    pipeline.stop();

    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);
    const auto result = panorama->getCvFrame();
    const auto center = result.at<cv::Vec3b>(result.rows / 2, result.cols / 2);
    REQUIRE(center != firstColor);
    REQUIRE(center != secondColor);
}

TEST_CASE("Calibrated cylindrical panorama masks inputs crossing the wrap boundary", "[Stitching]") {
    const cv::Vec3b referenceColor(20, 80, 140);
    const cv::Vec3b wrappedColor(180, 100, 40);
    const cv::Mat referenceImage(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, referenceColor);
    const cv::Mat wrappedImage(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, wrappedColor);

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(2);
    stitching->setUseInputCalibration(true);
    stitching->setCameraModel(dai::beta::node::Stitching::CameraModel::CYLINDRICAL);
    stitching->setSeamFinder(dai::beta::node::Stitching::SeamFinder::NONE);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    auto referenceInput = stitching->inputs["input0"].createInputQueue();
    auto wrappedInput = stitching->inputs["input1"].createInputQueue();
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    referenceInput->send(toCalibratedFrame(referenceImage, 0.0, 13));
    wrappedInput->send(toCalibratedFrame(wrappedImage, 170.0, 13));

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    pipeline.stop();

    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);
    const auto result = panorama->getCvFrame();
    REQUIRE(result.at<cv::Vec3b>(result.rows / 2, result.cols / 2) == referenceColor);
}

TEST_CASE("Calibrated panorama rejects camera geometry changes after preparation", "[Stitching]") {
    const cv::Mat image(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, cv::Scalar(40, 60, 80));

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(2);
    stitching->setUseInputCalibration(true);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    auto firstInput = stitching->inputs["input0"].createInputQueue();
    auto secondInput = stitching->inputs["input1"].createInputQueue();
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    firstInput->send(toCalibratedFrame(image, -10.0, 20));
    secondInput->send(toCalibratedFrame(image, 10.0, 20));

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);

    firstInput->send(toCalibratedFrame(image, -10.0, 21));
    secondInput->send(toCalibratedFrame(image, 15.0, 21));
    panorama = output->get<dai::ImgFrame>(std::chrono::milliseconds(200), timedOut);
    pipeline.stop();

    REQUIRE(timedOut);
    REQUIRE(panorama == nullptr);
}

TEST_CASE("Calibrated panorama rejects distorted inputs", "[Stitching]") {
    const cv::Mat image(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, cv::Scalar(40, 60, 80));

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(2);
    stitching->setUseInputCalibration(true);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    auto firstInput = stitching->inputs["input0"].createInputQueue();
    auto secondInput = stitching->inputs["input1"].createInputQueue();
    auto output = stitching->out.createOutputQueue();

    auto distorted = toCalibratedFrame(image, 10.0, 30);
    distorted->getTransformation() = calibratedTransformation(10.0, "reference-device", {0.1f, 0.0f, 0.0f, 0.0f});

    pipeline.start();
    firstInput->send(toCalibratedFrame(image, -10.0, 30));
    secondInput->send(distorted);

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::milliseconds(200), timedOut);
    pipeline.stop();

    REQUIRE(timedOut);
    REQUIRE(panorama == nullptr);
}

TEST_CASE("Calibrated panorama rejects inputs with different destination origins", "[Stitching]") {
    const cv::Mat image(VIEW_HEIGHT, VIEW_WIDTH, CV_8UC3, cv::Scalar(40, 60, 80));

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(2);
    stitching->setUseInputCalibration(true);
    stitching->setSeamFinder(dai::beta::node::Stitching::SeamFinder::NONE);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    auto firstInput = stitching->inputs["input0"].createInputQueue();
    auto secondInput = stitching->inputs["input1"].createInputQueue();
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    firstInput->send(toCalibratedFrame(image, -10.0, 0));
    auto mismatched = toCalibratedFrame(image, 10.0, 0);
    auto mismatchedTransformation = calibratedTransformation(10.0);
    auto mismatchedExtrinsics = mismatchedTransformation.getExtrinsics();
    SECTION("device ID") {
        mismatchedExtrinsics.toDeviceId = "different-device";
    }
    SECTION("camera socket") {
        mismatchedExtrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_B;
    }
    mismatchedTransformation.setExtrinsics(mismatchedExtrinsics);
    mismatched->getTransformation() = mismatchedTransformation;
    secondInput->send(mismatched);

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::milliseconds(200), timedOut);
    REQUIRE(timedOut);
    REQUIRE(panorama == nullptr);

    firstInput->send(toCalibratedFrame(image, -10.0, 1));
    secondInput->send(toCalibratedFrame(image, 10.0, 1));
    panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(panorama != nullptr);
    REQUIRE(panorama->getSequenceNum() == 1);

    firstInput->send(toCalibratedFrame(image, -10.0, 2));
    auto changedOrigin = toCalibratedFrame(image, 10.0, 2);
    auto changedTransformation = calibratedTransformation(10.0, "changed-after-preparation");
    changedOrigin->getTransformation() = changedTransformation;
    secondInput->send(changedOrigin);
    panorama = output->get<dai::ImgFrame>(std::chrono::milliseconds(200), timedOut);
    pipeline.stop();

    REQUIRE(timedOut);
    REQUIRE(panorama == nullptr);
}

TEST_CASE("Stitching combines three rotated views into a wider panorama", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> views = {renderView(scene, -15.0), renderView(scene, 0.0), renderView(scene, 15.0)};

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(views.size());
    stitching->setCameraModel(dai::beta::node::Stitching::CameraModel::CYLINDRICAL);
    stitching->setEstimationFrames(1);
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

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    pipeline.stop();

    REQUIRE(panorama != nullptr);
    REQUIRE(panorama->getType() == dai::ImgFrame::Type::BGR888i);
    // A panorama of three views 15 degrees apart is wider than a single view, but a degenerate
    // registration blows the canvas up to many times the view size
    REQUIRE(panorama->getWidth() > static_cast<unsigned int>(VIEW_WIDTH));
    REQUIRE(panorama->getWidth() < static_cast<unsigned int>(2 * VIEW_WIDTH));
    REQUIRE(panorama->getHeight() > static_cast<unsigned int>(VIEW_HEIGHT * 3 / 4));
}

TEST_CASE("Stitching rejects panoramas larger than the configured canvas", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> views = {renderView(scene, -6.0), renderView(scene, 6.0)};

    for(const bool continuous : {false, true}) {
        CAPTURE(continuous);
        dai::Pipeline pipeline(false);
        auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(views.size());
        stitching->setContinuous(continuous);
        stitching->setEstimationFrames(1);
        stitching->setMaxPanoramaSize(320, 240);
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

        bool timedOut = false;
        auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
        pipeline.stop();
        pipeline.wait();

        REQUIRE(timedOut);
        REQUIRE(panorama == nullptr);
    }
}

TEST_CASE("Stitching re-estimates every frame when continuous", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> views = {renderView(scene, -6.0), renderView(scene, 6.0)};

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(views.size());
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

    bool timedOut = false;
    auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    pipeline.stop();

    REQUIRE(panorama != nullptr);
    REQUIRE(panorama->getWidth() > static_cast<unsigned int>(VIEW_WIDTH));
    REQUIRE(panorama->getWidth() < static_cast<unsigned int>(2 * VIEW_WIDTH));
    // Metadata comes from the first input
    REQUIRE(panorama->getSequenceNum() == 7);
}

TEST_CASE("Stitching reuses the selected transform once the estimation frames are consumed", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> views = {renderView(scene, -6.0), renderView(scene, 6.0)};
    // Views the transform is deliberately not estimated for, to show it is no longer re-estimated
    const std::vector<cv::Mat> unseenViews = {renderView(scene, -10.0), renderView(scene, 10.0)};
    constexpr uint32_t ESTIMATION_FRAMES = 2;
    constexpr int64_t NUM_GROUPS = 4;

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(views.size());
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
        const bool panoramaExpected = group + 1 >= ESTIMATION_FRAMES;
        // Once the transform is fixed, images the node never estimated on still get warped by it
        const auto& groupViews = group == NUM_GROUPS - 1 ? unseenViews : views;
        for(size_t i = 0; i < groupViews.size(); ++i) {
            inputQueues[i]->send(toFrame(groupViews[i], group));
        }

        if(!panoramaExpected) continue;
        bool timedOut = false;
        auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(panorama != nullptr);
        REQUIRE(panorama->getSequenceNum() == group);

        if(fixedWidth == 0) {
            fixedWidth = panorama->getWidth();
        } else {
            REQUIRE(panorama->getWidth() == fixedWidth);
        }
    }

    pipeline.stop();
}

TEST_CASE("Stitching rebuilds fixed panorama composition on request", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> views = {renderView(scene, -6.0), renderView(scene, 6.0)};

    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(views.size());
    stitching->setContinuous(false);
    stitching->setEstimationFrames(1);
    stitching->setSyncThreshold(std::chrono::seconds(1));

    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    for(size_t i = 0; i < views.size(); ++i) {
        inputQueues.push_back(stitching->inputs["input" + std::to_string(i)].createInputQueue());
    }
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    cv::Size fixedSize;
    for(int64_t group = 0; group < 3; ++group) {
        if(group == 2) stitching->resetTransform();
        for(size_t i = 0; i < views.size(); ++i) {
            inputQueues[i]->send(toFrame(views[i], group));
        }

        bool timedOut = false;
        const auto panorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(panorama != nullptr);
        REQUIRE(panorama->getSequenceNum() == group);
        const cv::Size size(panorama->getWidth(), panorama->getHeight());
        if(group == 0) {
            fixedSize = size;
        } else if(group == 1) {
            REQUIRE(size == fixedSize);
        } else {
            // A fresh ORB registration can differ by a rounding pixel even for the same images.
            REQUIRE(std::abs(size.width - fixedSize.width) <= 1);
            REQUIRE(std::abs(size.height - fixedSize.height) <= 1);
        }
    }
    pipeline.stop();
}

TEST_CASE("Stitching freezes the strongest of multiple estimation candidates", "[Stitching]") {
    const cv::Mat scene = cv::imread(KITCHEN_IMAGE_PATH);
    REQUIRE(!scene.empty());

    const std::vector<cv::Mat> strongViews = {renderView(scene, -6.0), renderView(scene, 6.0)};
    std::vector<cv::Mat> weakViews = {renderView(scene, -18.0), renderView(scene, 18.0)};
    for(auto& view : weakViews) {
        cv::GaussianBlur(view, view, cv::Size(9, 9), 3.0);
    }

    std::optional<cv::Size> selectedSize;
    for(const bool strongFirst : {true, false}) {
        CAPTURE(strongFirst);
        const std::array<const std::vector<cv::Mat>*, 2> candidates = strongFirst ? std::array<const std::vector<cv::Mat>*, 2>{&strongViews, &weakViews}
                                                                                  : std::array<const std::vector<cv::Mat>*, 2>{&weakViews, &strongViews};
        dai::Pipeline pipeline(false);
        auto stitching = pipeline.create<dai::beta::node::Stitching>()->build(strongViews.size());
        stitching->setCameraModel(dai::beta::node::Stitching::CameraModel::PINHOLE);
        stitching->setContinuous(false);
        stitching->setEstimationFrames(2);
        stitching->setSyncThreshold(std::chrono::seconds(1));

        std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
        for(size_t i = 0; i < strongViews.size(); ++i) {
            inputQueues.push_back(stitching->inputs["input" + std::to_string(i)].createInputQueue());
        }
        auto output = stitching->out.createOutputQueue();

        pipeline.start();
        const auto sendGroup = [&](const std::vector<cv::Mat>& views, int64_t sequenceNum) {
            for(size_t i = 0; i < views.size(); ++i) {
                inputQueues[i]->send(toFrame(views[i], sequenceNum));
            }
        };

        sendGroup(*candidates[0], 0);
        sendGroup(*candidates[1], 1);
        bool timedOut = false;
        const auto selectedPanorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
        REQUIRE_FALSE(timedOut);
        sendGroup(strongViews, 2);
        timedOut = false;
        const auto fixedPanorama = output->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
        REQUIRE_FALSE(timedOut);
        pipeline.stop();

        REQUIRE(selectedPanorama != nullptr);
        REQUIRE(fixedPanorama != nullptr);
        REQUIRE(selectedPanorama->getSequenceNum() == 1);
        REQUIRE(fixedPanorama->getSequenceNum() == 2);
        REQUIRE(fixedPanorama->getWidth() == selectedPanorama->getWidth());
        REQUIRE(fixedPanorama->getHeight() == selectedPanorama->getHeight());

        const cv::Size currentSize(selectedPanorama->getWidth(), selectedPanorama->getHeight());
        if(selectedSize.has_value()) {
            // Pinhole composition leaves wave correction disabled, so equivalent registrations can differ by a couple
            // of rounding pixels at the canvas boundary.
            REQUIRE(std::abs(currentSize.width - selectedSize->width) <= 2);
            REQUIRE(std::abs(currentSize.height - selectedSize->height) <= 2);
        } else {
            selectedSize = currentSize;
        }
    }
}
