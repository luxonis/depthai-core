// Behavioural on-device tests of ToFStereoFusion: depth frames must arrive from the GPU backend, with the rectified geometry
// the network is trained on, and with the calibrated confidence of the network shipped with the firmware.
//
// By default the tests use the network shipped with the RVC4 firmware. To test another network instead:
//   DEPTHAI_TEST_TOF_STEREO_NETWORK_MODEL  path of the network (.dlc) on the host
//   DEPTHAI_TEST_TOF_STEREO_NETWORK_UDO    registration library of its custom operators on the device (optional)
#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "depthai/beta/node/ToFStereoFusion.hpp"
#include "depthai/depthai.hpp"

using namespace dai;
using Fusion = beta::node::ToFStereoFusion;

namespace {

constexpr auto FIRST_FRAME_TIMEOUT = std::chrono::seconds(30);  // network load included
constexpr auto RUN_TIME = std::chrono::seconds(10);
constexpr size_t MIN_FRAMES = 60;  // of ~300 at 30 fps; the CPU backend reaches ~24 fps at 1280x800

bool deviceHasTofSensor(const std::shared_ptr<Device>& device) {
    for(const auto& features : device->getConnectedCameraFeatures()) {
        for(const auto type : features.supportedTypes) {
            if(type == CameraSensorType::TOF) return true;
        }
    }
    return false;
}

struct PipelineStopGuard {
    Pipeline& pipeline;
    explicit PipelineStopGuard(Pipeline& p) : pipeline(p) {}
    ~PipelineStopGuard() {
        if(pipeline.isRunning()) pipeline.stop();
    }
    PipelineStopGuard(const PipelineStopGuard&) = delete;
    PipelineStopGuard& operator=(const PipelineStopGuard&) = delete;
};

// The network shipped with the firmware takes 1024x640.
constexpr uint32_t NETWORK_WIDTH = 1024;
constexpr uint32_t NETWORK_HEIGHT = 640;

/** A user network from the environment; nullopt: the network shipped with the firmware. */
struct UserModel {
    std::string model;
    std::string udo;
};

std::optional<UserModel> modelFromEnvironment() {
    const char* model = std::getenv("DEPTHAI_TEST_TOF_STEREO_NETWORK_MODEL");
    if(!model || !*model) return std::nullopt;
    UserModel setup;
    setup.model = model;
    if(const char* udo = std::getenv("DEPTHAI_TEST_TOF_STEREO_NETWORK_UDO")) setup.udo = udo;
    return setup;
}

/** Device log lines of the fusion node, collected from the device log callback. */
class LogCollector {
   public:
    void add(const LogMessage& message) {
        std::lock_guard<std::mutex> lock(mutex);
        lines.push_back(message.payload);
    }
    bool contains(const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex);
        for(const auto& line : lines)
            if(line.find(text) != std::string::npos) return true;
        return false;
    }

    /** The integer after the first occurrence of text, or -1. */
    int numberAfter(const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex);
        for(const auto& line : lines) {
            const auto at = line.find(text);
            if(at != std::string::npos) return std::atoi(line.c_str() + at + text.size());
        }
        return -1;
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        lines.clear();
    }

   private:
    std::mutex mutex;
    std::vector<std::string> lines;
};

std::shared_ptr<Fusion> buildFusion(Pipeline& pipeline, const std::optional<UserModel>& user) {
    auto left = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<Fusion>()->build(left, right);
    if(user) {
        fusion->neuralNetwork->setModelPath(user->model);
        if(!user->udo.empty()) {
            auto backendProperties = fusion->neuralNetwork->properties.backendProperties;
            backendProperties["udo_package"] = user->udo;
            fusion->neuralNetwork->setBackendProperties(backendProperties);
        }
    }
    return fusion;
}

/** fx of cv::stereoRectify(alpha 0, CALIB_ZERO_DISPARITY) for the left/right pair at the given size. */
float expectedRectifiedFx(const CalibrationHandler& calibration, uint32_t width, uint32_t height) {
    const auto toMat = [](const std::vector<std::vector<float>>& rows) {
        cv::Mat mat(static_cast<int>(rows.size()), static_cast<int>(rows[0].size()), CV_64F);
        for(size_t r = 0; r < rows.size(); ++r)
            for(size_t c = 0; c < rows[r].size(); ++c) mat.at<double>(static_cast<int>(r), static_cast<int>(c)) = rows[r][c];
        return mat;
    };
    const auto distortion = [](const std::vector<float>& values) {
        cv::Mat mat(1, static_cast<int>(values.size()), CV_64F);
        for(size_t i = 0; i < values.size(); ++i) mat.at<double>(0, static_cast<int>(i)) = values[i];
        return mat;
    };
    const cv::Mat k1 = toMat(calibration.getCameraIntrinsics(CameraBoardSocket::CAM_B, static_cast<int>(width), static_cast<int>(height)));
    const cv::Mat k2 = toMat(calibration.getCameraIntrinsics(CameraBoardSocket::CAM_C, static_cast<int>(width), static_cast<int>(height)));
    const cv::Mat d1 = distortion(calibration.getDistortionCoefficients(CameraBoardSocket::CAM_B));
    const cv::Mat d2 = distortion(calibration.getDistortionCoefficients(CameraBoardSocket::CAM_C));
    const cv::Mat extrinsics = toMat(calibration.getCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C));
    const cv::Mat rotation = extrinsics(cv::Rect(0, 0, 3, 3)).clone();
    const cv::Mat translation = extrinsics(cv::Rect(3, 0, 1, 3)).clone();
    const cv::Size size(static_cast<int>(width), static_cast<int>(height));
    cv::Mat r1, r2, p1, p2, q;
    cv::stereoRectify(k1, d1, k2, d2, size, rotation, translation, r1, r2, p1, p2, q, cv::CALIB_ZERO_DISPARITY, 0.0, size);
    return static_cast<float>(p1.at<double>(0, 0));
}

}  // namespace

TEST_CASE("ToFStereoFusion outputs depth from the GPU backend with the trained rectification", "[ToFStereoFusion][behaviour]") {
    const auto user = modelFromEnvironment();
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) SKIP("needs an RVC4 device with a ToF sensor");
    if(!device->hasGPU()) SKIP("needs a device with a GPU");

    LogCollector logs;
    device->setLogLevel(LogLevel::INFO);
    device->addLogCallback([&logs](LogMessage message) { logs.add(message); });

    Pipeline pipeline(device);
    auto fusion = buildFusion(pipeline, user);
    auto depthQueue = fusion->depth.createOutputQueue(8, false);

    PipelineStopGuard guard(pipeline);
    REQUIRE_NOTHROW(pipeline.start());

    bool timedOut = false;
    const auto first = depthQueue->get<ImgFrame>(FIRST_FRAME_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(first != nullptr);
    REQUIRE(first->getType() == ImgFrame::Type::RAW16);
    REQUIRE(first->getWidth() == NETWORK_WIDTH);
    REQUIRE(first->getHeight() == NETWORK_HEIGHT);

    size_t frames = 0, validPixels = 0, pixels = 0;
    std::vector<double> frameValid;
    const auto start = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start < RUN_TIME) {
        const auto depth = depthQueue->get<ImgFrame>(std::chrono::seconds(2), timedOut);
        REQUIRE_FALSE(timedOut);  // no output gap longer than 2 s
        ++frames;
        const auto* data = reinterpret_cast<const uint16_t*>(depth->getData().data());
        const size_t count = static_cast<size_t>(depth->getWidth()) * depth->getHeight();
        size_t frameValidPixels = 0, framePixels = 0;
        for(size_t i = 0; i < count; i += 97) {
            frameValidPixels += data[i] != 0 ? 1 : 0;
            ++framePixels;
        }
        validPixels += frameValidPixels;
        pixels += framePixels;
        frameValid.push_back(static_cast<double>(frameValidPixels) / static_cast<double>(framePixels));
    }
    INFO("depth frames in " << RUN_TIME.count() << " s: " << frames << ", valid " << (100.0 * validPixels / pixels) << " %");
    REQUIRE(frames >= MIN_FRAMES);
    REQUIRE(validPixels * 10 >= pixels);  // at least 10 % of the pixels have depth
    // No partly broken frames (static scene: every frame close to the typical one).
    std::vector<double> sorted = frameValid;
    std::sort(sorted.begin(), sorted.end());
    const double medianValid = sorted[sorted.size() / 2];
    INFO("valid fraction per frame: min " << sorted.front() << ", median " << medianValid);
    REQUIRE(sorted.front() >= 0.7 * medianValid);

    // The GPU backend was selected and did not fall back.
    REQUIRE(logs.contains("STEREO_NETWORK backend GPU"));
    REQUIRE_FALSE(logs.contains("GPU backend unavailable"));
    REQUIRE_FALSE(logs.contains("unexpected network output layout"));
    // The node read the network's inputs.
    REQUIRE(logs.contains("STEREO_NETWORK network inputs: "));
    REQUIRE_FALSE(logs.contains("cannot read the network inputs"));

    // Rectified with alpha 0 (the training geometry), not with the left input intrinsics.
    const auto intrinsics = first->transformation.getIntrinsicMatrix();
    const float expectedFx = expectedRectifiedFx(device->readCalibration(), NETWORK_WIDTH, NETWORK_HEIGHT);
    INFO("depth fx " << intrinsics[0][0] << ", expected (alpha 0) " << expectedFx);
    REQUIRE(std::abs(intrinsics[0][0] - expectedFx) < 0.01f * expectedFx);
}

namespace {

struct ConfidenceRun {
    int threshold = -1;  // the u8 threshold the device logged
    size_t pairs = 0, depthPixels = 0, belowThreshold = 0;
};

/** Runs the firmware network with the given threshold and counts the depth pixels whose published confidence is below the u8
 * threshold the device logged. */
ConfidenceRun runCalibrated(const std::shared_ptr<Device>& device, LogCollector& logs, float threshold) {
    logs.clear();
    Pipeline pipeline(device);
    auto fusion = buildFusion(pipeline, std::nullopt);
    fusion->initialConfig->setConfidenceThreshold(threshold);
    auto depthQueue = fusion->depth.createOutputQueue(8, false);
    auto confidenceQueue = fusion->confidence.createOutputQueue(8, false);

    PipelineStopGuard guard(pipeline);
    pipeline.start();
    bool timedOut = false;
    REQUIRE(depthQueue->get<ImgFrame>(FIRST_FRAME_TIMEOUT, timedOut) != nullptr);
    REQUIRE_FALSE(timedOut);
    ConfidenceRun run;
    run.threshold = logs.numberAfter("calibrated confidence, threshold ");
    std::map<int64_t, std::shared_ptr<ImgFrame>> depths, confidences;
    const auto start = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start < RUN_TIME) {
        for(const auto& frame : depthQueue->tryGetAll<ImgFrame>()) depths[frame->getSequenceNum()] = frame;
        for(const auto& frame : confidenceQueue->tryGetAll<ImgFrame>()) confidences[frame->getSequenceNum()] = frame;
        for(auto it = depths.begin(); it != depths.end();) {
            const auto match = confidences.find(it->first);
            if(match == confidences.end()) {
                ++it;
                continue;
            }
            const auto* depth = reinterpret_cast<const uint16_t*>(it->second->getData().data());
            const auto* confidence = match->second->getData().data();
            const size_t count = static_cast<size_t>(it->second->getWidth()) * it->second->getHeight();
            REQUIRE(match->second->getType() == ImgFrame::Type::RAW8);
            REQUIRE(static_cast<size_t>(match->second->getWidth()) * match->second->getHeight() == count);
            for(size_t i = 0; i < count; ++i) {
                if(depth[i] == 0) continue;
                ++run.depthPixels;
                if(static_cast<int>(confidence[i]) < run.threshold) ++run.belowThreshold;
            }
            ++run.pairs;
            confidences.erase(match);
            it = depths.erase(it);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return run;
}

}  // namespace

TEST_CASE("ToFStereoFusion publishes the calibrated confidence of the network shipped with the firmware", "[ToFStereoFusion][behaviour]") {
    if(modelFromEnvironment()) SKIP("a user network has no confidence calibration on the device");
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) SKIP("needs an RVC4 device with a ToF sensor");

    LogCollector logs;
    device->setLogLevel(LogLevel::INFO);
    device->addLogCallback([&logs](LogMessage message) { logs.add(message); });

    // The default threshold maps to the calibrated default of the network.
    const auto standard = runCalibrated(device, logs, beta::ToFStereoFusionConfig{}.confidenceThreshold);
    INFO("threshold " << standard.threshold << ", pairs " << standard.pairs << ", depth pixels " << standard.depthPixels << ", below "
                      << standard.belowThreshold);
    REQUIRE_FALSE(logs.contains("no valid confidence calibration"));
    REQUIRE(standard.threshold > 0);
    REQUIRE(standard.pairs >= MIN_FRAMES / 2);
    REQUIRE(standard.depthPixels > 0);
    // Every pixel with depth passed the threshold; near-range ToF pixels publish at least the threshold.
    REQUIRE(standard.belowThreshold == 0);

    // Other thresholds map proportionally.
    const auto half = runCalibrated(device, logs, 0.25f);
    INFO("threshold at 0.25: " << half.threshold);
    REQUIRE(half.threshold == static_cast<int>(std::lround(standard.threshold * 0.5)));
    REQUIRE(half.belowThreshold == 0);
    REQUIRE(half.depthPixels >= standard.depthPixels / 2);
}
