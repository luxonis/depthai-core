#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <numeric>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

constexpr double TOLERANCE = 0.010;
constexpr double RATE_ERROR = 0.02;
constexpr double WARMUP = 3.0;
constexpr double DURATION = 15.0;

double seconds(std::chrono::steady_clock::time_point timestamp) {
    return std::chrono::duration<double>(timestamp.time_since_epoch()).count();
}

struct Report {
    double header;
    double accel;
    double mag;
};

struct Image {
    int64_t sequence;
    double timestamp;
};

struct Group {
    Image image;
    Report report;
};

struct Capture {
    std::vector<Image> images;
    std::vector<Report> reports;
    std::vector<Group> groups;
};

Report readReport(const std::shared_ptr<dai::IMUData>& data) {
    REQUIRE(data != nullptr);
    REQUIRE(data->packets.size() == 1);
    const auto& packet = data->packets.front();
    return {seconds(data->getTimestampDevice()), seconds(packet.acceleroMeter.getTimestampDevice()), seconds(packet.magneticField.getTimestampDevice())};
}

Image readImage(const std::shared_ptr<dai::ImgFrame>& image) {
    REQUIRE(image != nullptr);
    CHECK(image->getWidth() == 320);
    CHECK(image->getHeight() == 200);
    CHECK(image->getType() == dai::ImgFrame::Type::GRAY8);
    CHECK(image->getTimestampDevice(dai::CameraExposureOffset::MIDDLE) <= image->getTimestampDevice(dai::CameraExposureOffset::END));
    return {image->getSequenceNum(), seconds(image->getTimestampDevice(dai::CameraExposureOffset::MIDDLE))};
}

Capture capture(dai::Pipeline& pipeline, int fps, bool oversampled) {
    const auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, static_cast<float>(fps));
    camera->initialControl.setManualExposure(1000, 100);
    auto* output = camera->requestOutput({320, 200}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, static_cast<float>(fps));
    const auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, oversampled ? 200 : fps);
    imu->enableIMUSensor(dai::IMUSensor::MAGNETOMETER_RAW, oversampled ? 100 : fps);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(1);
    const auto sync = pipeline.create<dai::node::Sync>();
    sync->setTimestampSource(dai::node::Sync::TimestampSource::DEVICE);
    sync->setSyncThreshold(std::chrono::milliseconds(10));
    sync->setSyncAttempts(-1);
    output->link(sync->inputs["image"]);
    imu->out.link(sync->inputs["imu"]);
    const auto images = output->createOutputQueue(4000, false);
    const auto reports = imu->out.createOutputQueue(4000, false);
    const auto groups = sync->out.createOutputQueue(4000, false);
    Capture result;
    pipeline.start();
    const auto startupDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    auto deadline = startupDeadline;
    bool started = false;
    while(std::chrono::steady_clock::now() < deadline) {
        for(int drained = 0; drained < 4000; ++drained) {
            const auto image = images->tryGet<dai::ImgFrame>();
            if(!image) break;
            result.images.push_back(readImage(image));
        }
        for(int drained = 0; drained < 4000; ++drained) {
            const auto report = reports->tryGet<dai::IMUData>();
            if(!report) break;
            result.reports.push_back(readReport(report));
        }
        for(int drained = 0; drained < 4000; ++drained) {
            const auto group = groups->tryGet<dai::MessageGroup>();
            if(!group) break;
            result.groups.push_back({readImage(group->get<dai::ImgFrame>("image")), readReport(group->get<dai::IMUData>("imu"))});
        }
        if(!started && !result.images.empty() && !result.reports.empty()) {
            started = true;
            deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(18500);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    pipeline.stop();
    REQUIRE(started);
    return result;
}

struct Rate {
    double mean = 0;
    double minWindow = 0;
    double maxWindow = 0;
    bool valid = false;
};

// Use elapsed timestamp intervals, not counts in a fixed bucket: bucket-edge
// quantization alone can exceed 2% at 5 Hz. Every sample starts a ~2s window.
Rate rate(const std::vector<double>& timestamps) {
    if(timestamps.size() < 2) return {};
    bool advancing = std::isfinite(timestamps.front()) && timestamps.front() > 0;
    for(std::size_t i = 1; i < timestamps.size(); ++i) {
        advancing = advancing && std::isfinite(timestamps[i]) && timestamps[i] > timestamps[i - 1];
    }
    Rate result;
    if(timestamps.back() > timestamps.front()) result.mean = static_cast<double>(timestamps.size() - 1) / (timestamps.back() - timestamps.front());
    if(!advancing) return result;
    result.minWindow = std::numeric_limits<double>::infinity();
    for(std::size_t begin = 0; begin + 1 < timestamps.size(); ++begin) {
        const auto end = std::lower_bound(timestamps.begin() + begin + 1, timestamps.end(), timestamps[begin] + 2.0);
        if(end == timestamps.end()) break;
        const auto window = static_cast<double>(end - timestamps.begin() - begin) / (*end - timestamps[begin]);
        result.minWindow = std::min(result.minWindow, window);
        result.maxWindow = std::max(result.maxWindow, window);
    }
    result.valid = std::isfinite(result.minWindow);
    if(!result.valid) result.minWindow = 0;
    return result;
}

nlohmann::json rateMetrics(const Rate& actual) {
    return {{"mean_hz", actual.mean}, {"min_window_hz", actual.minWindow}, {"max_window_hz", actual.maxWindow}, {"valid", actual.valid}};
}

void checkRate(const Rate& actual, double expected, const char* stream) {
    INFO("stream=" << stream << ", requested=" << expected);
    CHECK(actual.valid);
    CHECK(actual.mean == Catch::Approx(expected).epsilon(RATE_ERROR));
    CHECK(actual.minWindow == Catch::Approx(expected).epsilon(RATE_ERROR));
    CHECK(actual.maxWindow == Catch::Approx(expected).epsilon(RATE_ERROR));
}

void verify(const Capture& capture, int fps, bool oversampled, const std::string& deviceId) {
    REQUIRE_FALSE(capture.images.empty());
    REQUIRE_FALSE(capture.reports.empty());
    const auto lower = std::max(capture.images.front().timestamp, capture.reports.front().header) + WARMUP;
    const auto upper = lower + DURATION;
    std::vector<double> images, headers, accel, mag;
    std::set<int64_t> eligible;
    for(const auto& image : capture.images) {
        if(image.timestamp < lower || image.timestamp > upper) continue;
        images.push_back(image.timestamp);
        if(!eligible.empty()) CHECK(image.sequence == *eligible.rbegin() + 1);
        CHECK(eligible.insert(image.sequence).second);
    }
    double maxHeaderError = 0;
    double maxSensorSpan = 0;
    std::vector<double> signedSkew;
    for(const auto& report : capture.reports) {
        if(report.header < lower || report.header > upper) continue;
        headers.push_back(report.header);
        accel.push_back(report.accel);
        mag.push_back(report.mag);
        maxHeaderError = std::max(maxHeaderError, std::abs(report.header - std::max(report.accel, report.mag)));
        maxSensorSpan = std::max(maxSensorSpan, std::abs(report.accel - report.mag));
        signedSkew.push_back((report.accel - report.mag) * 1000);
    }
    const auto imageRate = rate(images);
    const auto headerRate = rate(headers);
    const auto accelRate = rate(accel);
    const auto magRate = rate(mag);
    std::vector<double> syncedImages, syncedAccel, syncedMag;
    std::set<int64_t> matched;
    double maxJointSpan = 0;
    for(const auto& group : capture.groups) {
        if(eligible.count(group.image.sequence) == 0) continue;
        CHECK(matched.insert(group.image.sequence).second);
        syncedImages.push_back(group.image.timestamp);
        syncedAccel.push_back(group.report.accel);
        syncedMag.push_back(group.report.mag);
        maxJointSpan = std::max(
            maxJointSpan,
            std::max({group.image.timestamp, group.report.accel, group.report.mag}) - std::min({group.image.timestamp, group.report.accel, group.report.mag}));
    }
    const auto coverage = eligible.empty() ? 0.0 : static_cast<double>(matched.size()) / static_cast<double>(eligible.size());
    auto absoluteSkew = signedSkew;
    for(auto& value : absoluteSkew) value = std::abs(value);
    std::sort(absoluteSkew.begin(), absoluteSkew.end());
    const nlohmann::json metrics = {
        {"device_id", deviceId},
        {"captured_images", capture.images.size()},
        {"captured_reports", capture.reports.size()},
        {"captured_groups", capture.groups.size()},
        {"measurement_start", lower},
        {"measurement_end", upper},
        {"fps", fps},
        {"oversampled", oversampled},
        {"image", rateMetrics(imageRate)},
        {"imu", rateMetrics(headerRate)},
        {"accel", rateMetrics(accelRate)},
        {"mag", rateMetrics(magRate)},
        {"synced_image", rateMetrics(rate(syncedImages))},
        {"synced_accel", rateMetrics(rate(syncedAccel))},
        {"synced_mag", rateMetrics(rate(syncedMag))},
        {"eligible_images", eligible.size()},
        {"matched_images", matched.size()},
        {"group_count", syncedImages.size()},
        {"coverage", coverage},
        {"max_sensor_span_ms", maxSensorSpan * 1000},
        {"max_joint_span_ms", maxJointSpan * 1000},
        {"max_header_error_ms", maxHeaderError * 1000},
        {"p99_sensor_span_ms", absoluteSkew.empty() ? 0 : absoluteSkew[static_cast<std::size_t>(std::ceil(0.99 * absoluteSkew.size())) - 1]},
        {"mean_signed_sensor_skew_ms", signedSkew.empty() ? 0 : std::accumulate(signedSkew.begin(), signedSkew.end(), 0.0) / signedSkew.size()},
        {"signed_sensor_skew_drift_ms", signedSkew.empty() ? 0 : signedSkew.back() - signedSkew.front()}};
    std::cout << "IMU_HIL_METRICS " << metrics.dump() << '\n';
    checkRate(imageRate, fps, "image");
    // IMUData exposes fresh paired reports at the bottleneck rate, not every
    // accelerometer sample captured at 200Hz in the oversampled profile.
    const auto expectedImuRate = oversampled ? 100 : fps;
    checkRate(headerRate, expectedImuRate, "IMU packet");
    checkRate(accelRate, expectedImuRate, "accelerometer");
    checkRate(magRate, expectedImuRate, "magnetometer");
    CHECK(maxHeaderError <= 1e-9);
    CHECK(maxSensorSpan <= TOLERANCE + 1e-9);
    CHECK(coverage >= 0.98);
    CHECK(maxJointSpan <= TOLERANCE + 1e-9);
    checkRate(rate(syncedImages), fps, "synced image");
    checkRate(rate(syncedAccel), fps, "synced accelerometer");
    checkRate(rate(syncedMag), fps, "synced magnetometer");
    REQUIRE(images.size() >= 2);
    REQUIRE(headers.size() >= 2);
    CHECK(images.front() <= lower + 2.0 / fps);
    CHECK(images.back() >= upper - 2.0 / fps);
    CHECK(headers.front() <= lower + 2.0 / expectedImuRate);
    CHECK(headers.back() >= upper - 2.0 / expectedImuRate);
}

}  // namespace

TEST_CASE("RVC4 requested IMU rates and individual report image alignment", "[imu][alignment][rvc4]") {
    const auto fps = GENERATE(5, 10, 20, 30, 40, 45, 50);
    const auto oversampled = GENERATE(false, true);
    // The campaign runner may select one case, while plain CTest runs all.
    if(const auto* selected = std::getenv("DEPTHAI_IMU_TEST_FPS")) {
        REQUIRE(std::string(selected) == std::to_string(std::stoi(selected)));
        const std::array<int, 7> supported{5, 10, 20, 30, 40, 45, 50};
        REQUIRE(std::find(supported.begin(), supported.end(), std::stoi(selected)) != supported.end());
        if(std::stoi(selected) != fps) return;
    }
    if(const auto* selected = std::getenv("DEPTHAI_IMU_TEST_PROFILE")) {
        const std::string profile(selected);
        REQUIRE((profile == "same-rate" || profile == "oversampled"));
        if((profile == "oversampled") != oversampled) return;
    }
    CAPTURE(fps, oversampled);
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(const auto* expected = std::getenv("DEPTHAI_IMU_EXPECTED_DEVICE_ID")) REQUIRE(device->getDeviceId() == expected);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("RVC4-only IMU regression");
    const auto features = device->getConnectedCameraFeatures();
    if(std::none_of(features.begin(), features.end(), [](const auto& feature) { return feature.socket == dai::CameraBoardSocket::CAM_B; })) {
        SKIP("Requires CAM_B for the RVC4 camera/IMU fixture");
    }
    if(device->getConnectedIMU().empty() || device->getConnectedIMU() == "BMI270") SKIP("Requires accelerometer and magnetometer");
    verify(capture(pipeline, fps, oversampled), fps, oversampled, device->getDeviceId());
}
