#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "../../../helpers/CameraGeometryDevice.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"
#include "depthai/schemas/ImgFrame.pb.h"
#include "utility/RecordReplayImpl.hpp"

namespace {

using Intrinsics = std::array<std::array<float, 3>, 3>;
const Intrinsics INPUT_K = {{{480, 0, 331}, {0, 520, 231}, {0, 0, 1}}};
const std::vector<float> INPUT_D = {0.16f, -0.04f, 0.001f, -0.002f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

std::vector<cv::Point3f> landmarks() {
    std::vector<cv::Point3f> points;
    for(float y : {-0.18f, 0.0f, 0.18f}) {
        for(float x : {-0.32f, 0.0f, 0.32f}) points.emplace_back(x, y, 1.0f);
    }
    return points;
}

std::vector<cv::Point2f> project(const Intrinsics& intrinsics, const std::vector<float>& distortion) {
    cv::Mat k(3, 3, CV_32F);
    for(int row = 0; row < 3; ++row)
        for(int col = 0; col < 3; ++col) k.at<float>(row, col) = intrinsics[row][col];
    std::vector<cv::Point2f> pixels;
    cv::projectPoints(landmarks(), cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), k, distortion, pixels);
    return pixels;
}

cv::Mat renderLandmarks() {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar::all(0));
    // Subpixel Gaussian centers avoid integer drawing/threshold rounding. The
    // centers are independently projected by OpenCV, not ImgTransformation.
    for(const auto& point : project(INPUT_K, INPUT_D)) {
        for(int y = static_cast<int>(point.y) - 20; y <= static_cast<int>(point.y) + 20; ++y) {
            for(int x = static_cast<int>(point.x) - 20; x <= static_cast<int>(point.x) + 20; ++x) {
                const double distance = (x - point.x) * (x - point.x) + (y - point.y) * (y - point.y);
                const auto value = cv::saturate_cast<unsigned char>(240 * std::exp(-distance / 50.0));
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
            }
        }
    }
    return image;
}

struct SyntheticRecording {
    std::filesystem::path directory =
        std::filesystem::temp_directory_path() / ("depthai-geometry-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    SyntheticRecording() {
        REQUIRE(std::filesystem::create_directory(directory));
    }
    SyntheticRecording(const SyntheticRecording&) = delete;
    SyntheticRecording& operator=(const SyntheticRecording&) = delete;
    SyntheticRecording(SyntheticRecording&&) = delete;
    SyntheticRecording& operator=(SyntheticRecording&&) = delete;

    void write() {
        const auto image = renderLandmarks();
        if(const auto* artifactDirectory = std::getenv("DEPTHAI_GEOMETRY_OUTPUT_DIR")) {
            std::filesystem::create_directories(artifactDirectory);
            CHECK(cv::imwrite((std::filesystem::path(artifactDirectory) / "input.png").string(), image));
        }
        cv::VideoWriter video((directory / "landmarks.avi").string(), cv::VideoWriter::fourcc('F', 'F', 'V', '1'), 10, image.size());
        REQUIRE(video.isOpened());
        dai::utility::ByteRecorder metadata;
        metadata.init<dai::proto::img_frame::ImgFrame>((directory / "landmarks.mcap").string(), dai::RecordConfig::CompressionLevel::NONE, "synthetic");
        for(int index = 0; index < 10; ++index) {
            video.write(image);
            dai::ImgFrame frame;
            frame.setCvFrame(image, dai::ImgFrame::Type::BGR888i);
            frame.setSourceSize(640, 480);
            frame.setTransformation(dai::ImgTransformation(640, 480, INPUT_K, dai::CameraModel::Perspective, INPUT_D, dai::Extrinsics{}));
            frame.setSequenceNum(index);
            const auto timestamp = std::chrono::steady_clock::time_point(std::chrono::milliseconds(100 * index));
            frame.setTimestamp(timestamp);
            frame.setTimestampDevice(timestamp);
            metadata.write(frame.serializeProto(true));
        }
        metadata.close();
        video.release();
        // A lossy codec could invalidate the pixel oracle before replay starts.
        cv::VideoCapture reader((directory / "landmarks.avi").string());
        cv::Mat decoded;
        REQUIRE(reader.read(decoded));
        REQUIRE(cv::norm(decoded, image, cv::NORM_INF) == 0);
        if(const auto* artifactDirectory = std::getenv("DEPTHAI_GEOMETRY_OUTPUT_DIR")) {
            for(const auto* filename : {"landmarks.avi", "landmarks.mcap"}) {
                std::filesystem::copy_file(
                    directory / filename, std::filesystem::path(artifactDirectory) / filename, std::filesystem::copy_options::overwrite_existing);
            }
        }
    }
    ~SyntheticRecording() {
        std::error_code error;
        std::filesystem::remove_all(directory, error);
    }
};

std::vector<cv::Point2f> detectLandmarks(const cv::Mat& image) {
    cv::Mat gray;
    if(image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else
        gray = image;
    cv::Mat labels, stats, centers;
    const int count = cv::connectedComponentsWithStats(gray > 40, labels, stats, centers);
    REQUIRE(count == 10);  // Nine independently detected markers plus background.
    std::vector<cv::Point2f> points;
    for(int label = 1; label < count; ++label) {
        auto region = cv::Rect(stats.at<int>(label, cv::CC_STAT_LEFT) - 3,
                               stats.at<int>(label, cv::CC_STAT_TOP) - 3,
                               stats.at<int>(label, cv::CC_STAT_WIDTH) + 6,
                               stats.at<int>(label, cv::CC_STAT_HEIGHT) + 6)
                      & cv::Rect(0, 0, gray.cols, gray.rows);
        const auto moments = cv::moments(gray(region));
        REQUIRE(moments.m00 > 0);
        points.emplace_back(region.x + moments.m10 / moments.m00, region.y + moments.m01 / moments.m00);
    }
    // The constellation has three separated rows and columns in every output.
    // Detection does not search near the metadata-predicted coordinates.
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a.y < b.y; });
    for(size_t row = 0; row < 3; ++row)
        std::sort(points.begin() + row * 3, points.begin() + (row + 1) * 3, [](const auto& a, const auto& b) { return a.x < b.x; });
    return points;
}

struct OutputCase {
    const char* name;
    std::pair<uint32_t, uint32_t> size;
    dai::ImgResizeMode mode;
    bool undistort;
    float alpha;
    float scaleX, scaleY, offsetX, offsetY;
    dai::ImgFrame::Type type = dai::ImgFrame::Type::BGR888i;
};

void checkPixels(dai::ImgFrame& frame, const OutputCase& output) {
    CAPTURE(output.name);
    REQUIRE(frame.getWidth() == static_cast<int>(output.size.first));
    REQUIRE(frame.getHeight() == static_cast<int>(output.size.second));
    const auto transformation = frame.getTransformation();
    const auto k = transformation.getIntrinsicMatrix();
    const auto d = transformation.getDistortionCoefficients();
    CHECK(transformation.getDistortionModel() == dai::CameraModel::Perspective);
    if(output.undistort) {
        for(float coefficient : d) CHECK(coefficient == 0);
    } else {
        CHECK(d == INPUT_D);
        CHECK(k[0][0] == Catch::Approx(INPUT_K[0][0] * output.scaleX).margin(0.01));
        CHECK(k[1][1] == Catch::Approx(INPUT_K[1][1] * output.scaleY).margin(0.01));
        CHECK(k[0][2] == Catch::Approx(INPUT_K[0][2] * output.scaleX + output.offsetX).margin(0.01));
        CHECK(k[1][2] == Catch::Approx(INPUT_K[1][2] * output.scaleY + output.offsetY).margin(0.01));
    }
    const auto detected = detectLandmarks(frame.getCvFrame());
    const auto predicted = project(k, d);
    double worstError = 0;
    for(size_t index = 0; index < predicted.size(); ++index) {
        const double error = cv::norm(detected[index] - predicted[index]);
        CAPTURE(index, detected[index].x, detected[index].y, predicted[index].x, predicted[index].y, error);
        // RVC2 warp-mesh endpoint sampling differs from OpenCV by up to one
        // output pixel; marker centroid/interpolation rounding adds a fraction.
        CHECK(error < 1.25);
        worstError = std::max(worstError, error);
    }
    std::cout << output.name << " maximum pixel / intrinsics error: " << worstError << " px\n";
    if(const auto* artifactDirectory = std::getenv("DEPTHAI_GEOMETRY_OUTPUT_DIR")) {
        std::filesystem::create_directories(artifactDirectory);
        auto annotated = frame.getCvFrame();
        if(annotated.channels() == 1) cv::cvtColor(annotated, annotated, cv::COLOR_GRAY2BGR);
        for(size_t index = 0; index < predicted.size(); ++index) {
            cv::circle(annotated, detected[index], 5, cv::Scalar(0, 255, 0), 1);
            cv::drawMarker(annotated, predicted[index], cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 7, 1);
        }
        CHECK(cv::imwrite((std::filesystem::path(artifactDirectory) / (std::string(output.name) + ".png")).string(), annotated));
    }
    // Prove this constellation rejects the original ~8.75% focal-length error.
    auto incorrectK = k;
    incorrectK[0][0] *= 3840.0f / 4208.0f;
    incorrectK[1][1] *= 3840.0f / 4208.0f;
    const auto incorrect = project(incorrectK, d);
    double incorrectError = 0;
    for(size_t index = 0; index < incorrect.size(); ++index) incorrectError = std::max(incorrectError, cv::norm(detected[index] - incorrect[index]));
    CHECK(incorrectError > 3);
    std::cout << output.name << " with incorrect focal lengths: " << incorrectError << " px\n";
}

}  // namespace

TEST_CASE("Synthetic replay pixels agree with Camera output intrinsics", "[camera-replay-geometry]") {
    SyntheticRecording recording;
    recording.write();
    dai::Pipeline pipeline(dai::test::openGeometryDevice());
    const auto platform = pipeline.getDefaultDevice()->getPlatform();
    if(platform != dai::Platform::RVC2 && platform != dai::Platform::RVC4) SKIP("Requires RVC2 or RVC4");
    // Enable the modern undistortion path in this pipeline only. Older EEPROM
    // policy otherwise deliberately suppresses Camera output undistortion.
    auto calibration = pipeline.getDefaultDevice()->readCalibration().getEepromData();
    calibration.stereoEnableDistortionCorrection = true;
    pipeline.setCalibrationData(dai::CalibrationHandler(calibration));
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(recording.directory / "landmarks.avi");
    replay->setReplayMetadataFile(recording.directory / "landmarks.mcap");
    replay->setFps(10);
    replay->setLoop(true);
    const auto features = pipeline.getDefaultDevice()->getConnectedCameraFeatures();
    const auto sensor = std::find_if(features.begin(), features.end(), [](const auto& feature) {
        return std::any_of(feature.supportedTypes.begin(), feature.supportedTypes.end(), [](auto type) {
            return type == dai::CameraSensorType::COLOR || type == dai::CameraSensorType::MONO;
        });
    });
    if(sensor == features.end()) SKIP("Replay through Camera requires a color or mono socket");
    auto camera = pipeline.create<dai::node::Camera>()->build(sensor->socket, *replay);
    const std::array<OutputCase, 7> outputs = {
        {{"ISP", {640, 480}, dai::ImgResizeMode::STRETCH, false, 0, 1, 1, 0, 0},
         {"stretch", {320, 240}, dai::ImgResizeMode::STRETCH, false, 0, .5f, .5f, 0, 0},
         {"crop", {320, 180}, dai::ImgResizeMode::CROP, false, 0, .5f, .5f, 0, -30},
         {"letterbox", {320, 320}, dai::ImgResizeMode::LETTERBOX, false, 0, .5f, .5f, 0, 40},
         {"undistort alpha 0", {320, 240}, dai::ImgResizeMode::STRETCH, true, 0, 0, 0, 0, 0},
         {"undistort alpha 1", {320, 240}, dai::ImgResizeMode::STRETCH, true, 1, 0, 0, 0, 0},
         {"nonuniform grayscale", {320, 160}, dai::ImgResizeMode::STRETCH, false, 0, .5f, 1.0f / 3, 0, 0, dai::ImgFrame::Type::GRAY8}}};
    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    queues.push_back(camera->requestIspOutput()->createOutputQueue(1, false));
    for(size_t index = 1; index < outputs.size(); ++index) {
        const auto& output = outputs[index];
        queues.push_back(
            camera
                ->requestOutput(output.size, output.type, output.mode, 10, output.undistort, output.undistort ? std::make_optional(output.alpha) : std::nullopt)
                ->createOutputQueue(1, false));
    }
    auto manip = pipeline.create<dai::node::ImageManip>();
    replay->out.link(manip->inputImage);
    manip->initialConfig->addCrop(64, 48, 512, 384);
    manip->initialConfig->setOutputSize(256, 192, dai::ImageManipConfig::ResizeMode::STRETCH);
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::BGR888i);
    manip->setMaxOutputFrameSize(256 * 192 * 3);
    const auto manipQueue = manip->out.createOutputQueue(1, false);
    pipeline.start();
    std::vector<Intrinsics> returnedIntrinsics;
    for(size_t index = 0; index < outputs.size(); ++index) {
        CAPTURE(outputs[index].name);
        bool timedOut = false;
        const auto frame = queues[index]->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(frame != nullptr);
        if(index != 0) CHECK(frame->getType() == outputs[index].type);
        checkPixels(*frame, outputs[index]);
        returnedIntrinsics.push_back(frame->getTransformation().getIntrinsicMatrix());
    }
    // Alpha 1 retains more FOV than alpha 0; both requests must be honoured.
    CHECK(returnedIntrinsics[4][0][0] > returnedIntrinsics[5][0][0]);
    CHECK(returnedIntrinsics[4][1][1] > returnedIntrinsics[5][1][1]);
    bool timedOut = false;
    const auto frame = manipQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(frame != nullptr);
    checkPixels(*frame, {"ImageManip crop and stretch", {256, 192}, dai::ImgResizeMode::STRETCH, false, 0, .5f, .5f, -32, -24});
}
