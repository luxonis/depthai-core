#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <cmath>
#include <depthai/depthai.hpp>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../../../src/pipeline/node/WorldConstraintsValidation.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
    #include <opencv2/imgproc.hpp>
#endif

namespace {
using Socket = dai::CameraBoardSocket;

const char* kWorldJson = R"({
    "markerType": "ARUCO",
    "arucoDictionary": "DICT_4X4_50",
    "markerDistances": [{"markers": [37, 43], "distance": 0.84}],
    "markerSizes": [{"marker": 43, "size": 0.15}]
})";
}  // namespace

TEST_CASE("MultiDeviceCalibration keeps world constraints and rejects the ones it cannot use", "[MultiDeviceWorldConstraints]") {
    dai::Pipeline pipeline(false);
    auto node = pipeline.create<dai::beta::node::MultiDeviceCalibration>();
    REQUIRE_FALSE(node->getWorldConstraints().has_value());

    node->setWorldConstraintsFromJson(kWorldJson);
    auto constraints = node->getWorldConstraints();
    REQUIRE(constraints.has_value());
    REQUIRE(constraints->markerType == dai::MarkerType::ARUCO);
    REQUIRE(constraints->markerDistances.size() == 1);
    REQUIRE(constraints->markerDistances[0].distance == Catch::Approx(0.84f));
    REQUIRE(constraints->markerSizes.size() == 1);

    // Socket names repeat across devices: a socket distance has to say which device it means.
    auto withSocket = *constraints;
    withSocket.socketDistances = {{37, Socket::CAM_A, 1.68f}};
    REQUIRE_THROWS_WITH(node->setWorldConstraints(withSocket), Catch::Matchers::ContainsSubstring("needs the deviceId"));
    withSocket.socketDistances = {{37, "device-b", Socket::CAM_A, 1.68f}};
    node->setWorldConstraints(withSocket);
    REQUIRE(node->getWorldConstraints()->socketDistances.at(0).deviceId == "device-b");
    node->setWorldConstraintsFromJson(
        R"({"socketDistances": [{"marker": 37, "deviceId": "device-a", "socket": "CAM_A", "distance": 1.5},
                                {"marker": 37, "deviceId": "device-b", "socket": "CAM_A", "distance": 1.7}]})");
    REQUIRE(node->getWorldConstraints()->socketDistances.at(1).deviceId == "device-b");
    node->setWorldConstraintsFromJson(kWorldJson);

    // The same validation as on the DynamicCalibration node.
    REQUIRE_THROWS_AS(node->setWorldConstraints(dai::WorldConstraints{}), std::invalid_argument);
    auto negative = *constraints;
    negative.markerDistances[0].distance = -1.0f;
    REQUIRE_THROWS_AS(node->setWorldConstraints(negative), std::invalid_argument);
    REQUIRE_THROWS_AS(node->setWorldConstraintsFromJson(R"({"markerDistances": [{"markers": [1, 2], "distance": 1.0}], "typo": 1})"), std::invalid_argument);

    // A rejected set leaves the previous one in place.
    REQUIRE(node->getWorldConstraints()->markerDistances.size() == 1);
    node->clearWorldConstraints();
    REQUIRE_FALSE(node->getWorldConstraints().has_value());
}

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && defined(DEPTHAI_TEST_WORLD_CONSTRAINTS_RECONSTRUCTION)

namespace {

const cv::Matx33d kIntrinsics(500.0, 0.0, 319.5, 0.0, 500.0, 239.5, 0.0, 0.0, 1.0);
constexpr int kWidth = 640;
constexpr int kHeight = 480;

/// A camera of the synthetic scene: world to camera.
struct SceneCamera {
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d translation{0.0, 0.0, 0.0};

    std::array<float, 2> project(const cv::Vec3d& point) const {
        const cv::Vec3d pixel = kIntrinsics * (rotation * point + translation);
        return {static_cast<float>(pixel[0] / pixel[2]), static_cast<float>(pixel[1] / pixel[2])};
    }
    std::vector<std::vector<float>> transform() const {
        std::vector<std::vector<float>> matrix(4, std::vector<float>(4, 0.0f));
        for(int row = 0; row < 3; ++row) {
            for(int col = 0; col < 3; ++col) matrix[row][col] = static_cast<float>(rotation(row, col));
            matrix[row][3] = static_cast<float>(translation[row]);
        }
        matrix[3][3] = 1.0f;
        return matrix;
    }
};

/// Camera at `center` turned by `yawDeg` about the vertical axis.
SceneCamera cameraAt(const cv::Vec3d& center, double yawDeg) {
    const double yaw = yawDeg * CV_PI / 180.0;
    SceneCamera camera;
    camera.rotation = cv::Matx33d(std::cos(yaw), 0.0, -std::sin(yaw), 0.0, 1.0, 0.0, std::sin(yaw), 0.0, std::cos(yaw));
    camera.translation = -(camera.rotation * center);
    return camera;
}

std::vector<std::vector<float>> intrinsicsMatrix() {
    std::vector<std::vector<float>> matrix(3, std::vector<float>(3, 0.0f));
    for(int row = 0; row < 3; ++row)
        for(int col = 0; col < 3; ++col) matrix[row][col] = static_cast<float>(kIntrinsics(row, col));
    return matrix;
}

}  // namespace

TEST_CASE("World constraints are reconstructed from cameras of several devices", "[MultiDeviceWorldConstraints]") {
    using dai::node::detail::PosedObservationCamera;
    // Two devices, each with one camera on the same socket: the cameras are told apart by their keys only.
    const std::vector<SceneCamera> scene = {cameraAt({0.0, 0.0, 0.0}, 0.0), cameraAt({0.8, 0.1, 0.0}, -12.0)};
    std::map<Socket, PosedObservationCamera> cameras;
    for(size_t idx = 0; idx < scene.size(); ++idx) {
        cameras.emplace(static_cast<Socket>(idx), PosedObservationCamera{{intrinsicsMatrix(), {}, dai::CameraModel::Perspective}, scene[idx].transform()});
    }
    const cv::Vec3d marker37(-0.3, 0.1, 3.0);
    const cv::Vec3d marker43(0.5, -0.2, 3.4);
    dai::MarkerObservationSet set;
    for(size_t idx = 0; idx < scene.size(); ++idx) {
        set.centers[37][static_cast<Socket>(idx)] = scene[idx].project(marker37);
        set.centers[43][static_cast<Socket>(idx)] = scene[idx].project(marker43);
    }
    const std::deque<dai::MarkerObservationSet> sets{set};

    dai::WorldConstraints constraints;
    constraints.markerDistances = {{37, 43, static_cast<float>(cv::norm(marker37 - marker43))}};
    auto residuals = dai::node::detail::worldConstraintResiduals(constraints, sets, cameras);
    REQUIRE(residuals.size() == 1);
    REQUIRE(residuals[0].measured.has_value());
    REQUIRE(*residuals[0].measured == Catch::Approx(cv::norm(marker37 - marker43)).margin(1e-3));
    REQUIRE(residuals[0].fits());

    // The same observations with the second device 20 % farther away: the scene scales with it and the
    // measurement is no longer met.
    auto farther = cameras;
    farther.at(static_cast<Socket>(1)).baseToCamera = cameraAt({0.96, 0.12, 0.0}, -12.0).transform();
    residuals = dai::node::detail::worldConstraintResiduals(constraints, sets, farther);
    REQUIRE(residuals[0].measured.has_value());
    REQUIRE(*residuals[0].measured == Catch::Approx(1.2 * cv::norm(marker37 - marker43)).epsilon(0.01));
    REQUIRE_FALSE(residuals[0].fits());
}

namespace {

// DICT_4X4_50 marker 37 with its one-cell black border, one character per cell ('1' = white).
constexpr std::array<const char*, 6> kMarker37 = {"000000", "000000", "001000", "011110", "011110", "000000"};

/// A texture full of blobs of every size, which gives the feature detector something to match.
cv::Mat blobTexture(int width, int height, uint64_t seed) {
    cv::RNG rng(seed);
    cv::Mat texture(height, width, CV_8UC1, cv::Scalar(128));
    for(int blob = 0; blob < width * height / 400; ++blob) {
        const cv::Point center(rng.uniform(0, width), rng.uniform(0, height));
        const int size = rng.uniform(3, 28);
        const cv::Scalar shade(rng.uniform(20, 235));
        if(blob % 2 == 0) {
            cv::circle(texture, center, size, shade, cv::FILLED, cv::LINE_AA);
        } else {
            cv::rectangle(texture, center, center + cv::Point(size, rng.uniform(3, 28)), shade, cv::FILLED);
        }
    }
    cv::GaussianBlur(texture, texture, cv::Size(0, 0), 0.8);
    return texture;
}

/// Draws marker 37 on a white board, `cell` texture pixels per cell, its top-left corner at (x, y).
void drawMarker37(cv::Mat& texture, int x, int y, int cell) {
    cv::rectangle(texture, cv::Rect(x - 2 * cell, y - 2 * cell, 10 * cell, 10 * cell), cv::Scalar(255), cv::FILLED);
    for(int row = 0; row < 6; ++row) {
        for(int col = 0; col < 6; ++col) {
            cv::rectangle(texture, cv::Rect(x + col * cell, y + row * cell, cell, cell), cv::Scalar(kMarker37[row][col] == '1' ? 255 : 0), cv::FILLED);
        }
    }
}

/// Paints a textured plane into the image of a camera. The plane starts at `origin` and spans the texture along
/// `u` (texture columns) and `v` (texture rows), `pxPerMeter` texture pixels per meter.
void drawPlane(
    cv::Mat& image, const cv::Mat& texture, double pxPerMeter, const cv::Vec3d& origin, const cv::Vec3d& u, const cv::Vec3d& v, const SceneCamera& camera) {
    const cv::Vec3d columnU = kIntrinsics * (camera.rotation * u) / pxPerMeter;
    const cv::Vec3d columnV = kIntrinsics * (camera.rotation * v) / pxPerMeter;
    const cv::Vec3d columnO = kIntrinsics * (camera.rotation * origin + camera.translation);
    const cv::Matx33d homography(columnU[0], columnV[0], columnO[0], columnU[1], columnV[1], columnO[1], columnU[2], columnV[2], columnO[2]);
    cv::Mat warped;
    cv::Mat mask;
    cv::warpPerspective(texture, warped, homography, image.size(), cv::INTER_LINEAR);
    cv::warpPerspective(cv::Mat(texture.size(), CV_8UC1, cv::Scalar(255)), mask, homography, image.size(), cv::INTER_NEAREST);
    warped.copyTo(image, mask);
}

constexpr double kMarkerSide = 0.48;  // meters: 6 cells of 16 texture pixels at 200 px per meter

/// A wall 3 m ahead with marker 37 on it, a side wall on the left and an inclined panel below: three planes, so
/// that the views are not related by a single homography.
cv::Mat renderScene(const SceneCamera& camera) {
    constexpr double pxPerMeter = 200.0;
    static const cv::Mat wall = [] {
        cv::Mat texture = blobTexture(1600, 800, 7);
        drawMarker37(texture, 800, 580, 16);
        return texture;
    }();
    static const cv::Mat side = blobTexture(300, 800, 11);
    static const cv::Mat panel = blobTexture(800, 300, 13);
    cv::Mat image(kHeight, kWidth, CV_8UC1, cv::Scalar(128));
    drawPlane(image, wall, pxPerMeter, {-4.0, -3.4, 3.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, camera);
    drawPlane(image, side, pxPerMeter, {-0.9, -3.4, 3.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, camera);
    drawPlane(image, panel, pxPerMeter, {-2.0, 0.25, 2.4}, {1.0, 0.0, 0.0}, {0.0, 0.6, -0.8}, camera);
    return image;
}

constexpr double kOtherSocketOffset = 0.10;  // meters between CAM_A and the image-less CAM_B of each device
const cv::Vec3d kMarkerCenter(-4.0 + (800.0 + 48.0) / 200.0, -3.4 + (580.0 + 48.0) / 200.0, 3.0);
const cv::Vec3d kDeviceBCenter(0.5, 0.0, 0.0);
constexpr double kDeviceBYawDeg = -6.0;

struct NodeRun {
    std::shared_ptr<dai::beta::MultiDeviceCalibrationResult> result;
    double deviceDistance = 0.0;
};

/// Runs the node on two single-camera devices that look at the synthetic scene from 0.5 m apart.
NodeRun runNode(const std::optional<dai::WorldConstraints>& constraints) {
    const std::vector<std::pair<std::string, SceneCamera>> devices = {{"device-a", cameraAt({0.0, 0.0, 0.0}, 0.0)},
                                                                      {"device-b", cameraAt(kDeviceBCenter, kDeviceBYawDeg)}};
    constexpr size_t kSamples = 2;

    dai::Pipeline pipeline(false);
    auto node = pipeline.create<dai::beta::node::MultiDeviceCalibration>();
    node->setSampleCount(kSamples);
    node->sync->setRunOnHost(true);
    node->sync->setSyncAttempts(-1);
    node->syncInput.setBlocking(true);
    node->syncInput.setMaxSize(kSamples);
    if(constraints) node->setWorldConstraints(*constraints);

    std::array<std::array<float, 3>, 3> intrinsics{};
    for(int row = 0; row < 3; ++row)
        for(int col = 0; col < 3; ++col) intrinsics[row][col] = static_cast<float>(kIntrinsics(row, col));

    std::vector<std::shared_ptr<dai::InputQueue>> queues;
    for(const auto& [deviceId, camera] : devices) {
        dai::CalibrationHandler calibration;
        calibration.setCameraIntrinsics(Socket::CAM_A, intrinsicsMatrix(), kWidth, kHeight);
        calibration.setDistortionCoefficients(Socket::CAM_A, std::vector<float>(14, 0.0f));
        calibration.setCameraType(Socket::CAM_A, dai::CameraModel::Perspective);
        // A second calibrated socket that supplies no images, kOtherSocketOffset to the right of the first.
        calibration.setCameraIntrinsics(Socket::CAM_B, intrinsicsMatrix(), kWidth, kHeight);
        calibration.setDistortionCoefficients(Socket::CAM_B, std::vector<float>(14, 0.0f));
        calibration.setCameraType(Socket::CAM_B, dai::CameraModel::Perspective);
        const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        const std::vector<float> otherToFirstCm = {static_cast<float>(100.0 * kOtherSocketOffset), 0.0f, 0.0f};
        calibration.setCameraExtrinsics(Socket::CAM_B, Socket::CAM_A, identity, otherToFirstCm, otherToFirstCm);
        node->setDeviceCalibration(deviceId, calibration);

        auto injector = pipeline.create<dai::node::ImageManip>();
        injector->setRunOnHost(true);
        injector->setMaxOutputFrameSize(kWidth * kHeight * 4);
        injector->inputImage.setBlocking(true);
        queues.push_back(injector->inputImage.createInputQueue());
        node->addCamera(deviceId, Socket::CAM_A, injector->out);
    }
    auto controlQueue = node->inputControl.createInputQueue();
    auto resultQueue = node->calibrationOutput.createOutputQueue();

    pipeline.start();
    controlQueue->send(dai::beta::MultiDeviceCalibrationControl::start());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for(size_t sample = 0; sample < kSamples; ++sample) {
        const auto timestamp = std::chrono::steady_clock::time_point(std::chrono::seconds(10) + std::chrono::milliseconds(100 * sample));
        for(size_t idx = 0; idx < devices.size(); ++idx) {
            auto frame = std::make_shared<dai::ImgFrame>();
            frame->setCvFrame(renderScene(devices[idx].second), dai::ImgFrame::Type::GRAY8);
            frame->setTransformation(dai::ImgTransformation(kWidth, kHeight, intrinsics, dai::CameraModel::Perspective, std::vector<float>(14, 0.0f)));
            frame->setTimestamp(timestamp);
            frame->setTimestampDevice(timestamp);
            frame->setSequenceNum(static_cast<int64_t>(sample));
            queues[idx]->send(frame);
        }
    }
    bool timedOut = false;
    NodeRun run;
    run.result = resultQueue->get<dai::beta::MultiDeviceCalibrationResult>(std::chrono::seconds(120), timedOut);
    pipeline.stop();
    REQUIRE_FALSE(timedOut);
    REQUIRE(run.result != nullptr);
    if(run.result->passed && run.result->graph.has_value() && !run.result->graph->empty()) {
        const auto translation = run.result->graph->front().extrinsics.getTranslationVector(false, dai::LengthUnit::METER);
        run.deviceDistance = std::sqrt(translation[0] * translation[0] + translation[1] * translation[1] + translation[2] * translation[2]);
    }
    return run;
}

}  // namespace

TEST_CASE("MultiDeviceCalibration takes its metric scale from world constraints", "[MultiDeviceWorldConstraints]") {
    // One camera per device: image features give the direction between the devices but not the distance.
    const auto withoutReference = runNode(std::nullopt);
    REQUIRE_FALSE(withoutReference.result->passed);

    // The printed size of a marker both cameras see is a metric reference.
    dai::WorldConstraints constraints;
    constraints.markerSizes = {{37, static_cast<float>(kMarkerSide)}};
    const auto withSize = runNode(constraints);
    INFO(withSize.result->info);
    REQUIRE(withSize.result->passed);
    REQUIRE(withSize.deviceDistance == Catch::Approx(0.5).epsilon(0.06));

    // A size entered 1.5 times too large moves the devices apart by the same factor: nothing else knows the scale.
    constraints.markerSizes = {{37, static_cast<float>(1.5 * kMarkerSide)}};
    const auto withWrongSize = runNode(constraints);
    REQUIRE(withWrongSize.result->passed);
    REQUIRE(withWrongSize.deviceDistance == Catch::Approx(0.75).epsilon(0.06));
}

TEST_CASE("MultiDeviceCalibration takes its metric scale from socket distances", "[MultiDeviceWorldConstraints]") {
    // The distance from the marker to the registered camera of the second device.
    dai::WorldConstraints constraints;
    constraints.socketDistances = {{37, "device-b", Socket::CAM_A, static_cast<float>(cv::norm(kMarkerCenter - kDeviceBCenter))}};
    const auto toRegistered = runNode(constraints);
    INFO(toRegistered.result->info);
    REQUIRE(toRegistered.result->passed);
    REQUIRE(toRegistered.deviceDistance == Catch::Approx(0.5).epsilon(0.06));

    // The distance to a socket of the second device that supplies no images: it keeps its place in the device.
    const double yaw = kDeviceBYawDeg * CV_PI / 180.0;
    const cv::Vec3d otherCenter = kDeviceBCenter + kOtherSocketOffset * cv::Vec3d(std::cos(yaw), 0.0, -std::sin(yaw));
    constraints.socketDistances = {{37, "device-b", Socket::CAM_B, static_cast<float>(cv::norm(kMarkerCenter - otherCenter))}};
    const auto toOther = runNode(constraints);
    INFO(toOther.result->info);
    REQUIRE(toOther.result->passed);
    REQUIRE(toOther.deviceDistance == Catch::Approx(0.5).epsilon(0.06));

    // The same measurement entered 1.5 times too long moves the devices apart: with one camera per device and
    // one measurement nothing else knows the scale.
    constraints.socketDistances[0].distance *= 1.5f;
    const auto wrong = runNode(constraints);
    REQUIRE(wrong.result->passed);
    REQUIRE(wrong.deviceDistance > 0.65);

    // A device nobody registered, and a socket the device calibration does not know.
    constraints.socketDistances = {{37, "device-c", Socket::CAM_A, 3.0f}};
    const auto unknownDevice = runNode(constraints);
    REQUIRE_FALSE(unknownDevice.result->passed);
    REQUIRE_THAT(unknownDevice.result->info, Catch::Matchers::ContainsSubstring("device without registered cameras"));
    constraints.socketDistances = {{37, "device-b", Socket::CAM_D, 3.0f}};
    const auto unknownSocket = runNode(constraints);
    REQUIRE_FALSE(unknownSocket.result->passed);
    REQUIRE_THAT(unknownSocket.result->info, Catch::Matchers::ContainsSubstring("socket without calibration"));
}

#endif
