#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"

namespace {
using namespace std::chrono_literals;

void setReport(dai::IMUReport& report, int ms) {
    report.tsDevice = {ms / 1000, (ms % 1000) * 1000000};
    report.timestamp = report.tsDevice;
    report.tsSystem = report.tsDevice;
}

std::shared_ptr<dai::IMUData> imu(int accelMs, int magMs, int sequence) {
    auto data = std::make_shared<dai::IMUData>();
    data->packets.resize(1);
    setReport(data->packets[0].acceleroMeter, accelMs);
    setReport(data->packets[0].magneticField, magMs);
    data->setSequenceNum(sequence);
    const auto newest = std::chrono::milliseconds(std::max(accelMs, magMs));
    data->setTimestamp(std::chrono::steady_clock::time_point(newest));
    data->setTimestampDevice(std::chrono::steady_clock::time_point(newest));
    data->setTimestampSystem(std::chrono::system_clock::time_point(newest));
    return data;
}

std::shared_ptr<dai::ImgFrame> image(int ms) {
    auto data = std::make_shared<dai::ImgFrame>();
    data->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::milliseconds(ms)));
    data->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::milliseconds(ms)));
    data->setTimestampSystem(std::chrono::system_clock::time_point(std::chrono::milliseconds(ms)));
    return data;
}
}  // namespace

TEST_CASE("Sync properties roundtrip through the pipeline schema") {
    dai::Pipeline pipeline(false);
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    REQUIRE_FALSE(sync->getSyncOnIndividualReports());
    const auto reportAware = GENERATE(false, true);
    sync->setSyncOnIndividualReports(reportAware);
    sync->setSyncThreshold(17ms);
    sync->setSyncAttempts(3);
    sync->setTimestampSource(dai::node::Sync::TimestampSource::SYSTEM);
    REQUIRE(sync->getSyncOnIndividualReports() == reportAware);
    dai::SyncProperties decoded;
    dai::utility::deserialize(pipeline.getPipelineSchema().nodes.at(sync->id).properties, decoded);
    REQUIRE(decoded.syncOnIndividualReports == reportAware);
    REQUIRE(decoded.syncThresholdNs == 17000000);
    REQUIRE(decoded.syncAttempts == 3);
    REQUIRE(decoded.timestampSource == dai::SyncProperties::TimestampSource::SYSTEM);
    const nlohmann::json json = decoded;
    const auto fromJson = json.get<dai::SyncProperties>();
    REQUIRE(fromJson.syncOnIndividualReports == reportAware);
    REQUIRE(fromJson.syncThresholdNs == decoded.syncThresholdNs);
    REQUIRE(fromJson.syncAttempts == decoded.syncAttempts);
    REQUIRE(fromJson.processor == decoded.processor);
    REQUIRE(fromJson.timestampSource == decoded.timestampSource);
}

TEST_CASE("Report-aware Sync rejects an IMU packet whose header hides an old accelerometer report") {
    dai::Pipeline pipeline(false);
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    const auto source = GENERATE(dai::node::Sync::TimestampSource::DEVICE, dai::node::Sync::TimestampSource::HOST, dai::node::Sync::TimestampSource::SYSTEM);
    sync->setTimestampSource(source);
    sync->setSyncOnIndividualReports(true);
    sync->setSyncThreshold(10ms);
    auto images = sync->inputs["image"].createInputQueue();
    auto reports = sync->inputs["imu"].createInputQueue();
    auto output = sync->out.createOutputQueue();
    pipeline.start();
    images->send(image(100));
    auto oldPacket = imu(81, 99, 1);
    auto suitablePacket = imu(98, 102, 2);
    for(const auto& packet : {oldPacket, suitablePacket}) {
        for(dai::IMUReport* report :
            {static_cast<dai::IMUReport*>(&packet->packets[0].acceleroMeter), static_cast<dai::IMUReport*>(&packet->packets[0].magneticField)}) {
            if(source != dai::node::Sync::TimestampSource::HOST) report->timestamp = {1, 0};
            if(source != dai::node::Sync::TimestampSource::SYSTEM) report->tsSystem = dai::Timestamp{2, 0};
            if(source != dai::node::Sync::TimestampSource::DEVICE) report->tsDevice = {3, 0};
        }
    }
    reports->send(oldPacket);
    reports->send(suitablePacket);
    bool timedOut = false;
    auto group = output->get<dai::MessageGroup>(1s, timedOut);
    pipeline.stop();
    REQUIRE_FALSE(timedOut);
    REQUIRE(group != nullptr);
    REQUIRE(group->get<dai::IMUData>("imu")->getSequenceNum() == 2);
}

TEST_CASE("Default Sync retains header-based matching") {
    dai::Pipeline pipeline(false);
    auto sync = pipeline.create<dai::node::Sync>();
    REQUIRE_FALSE(sync->getSyncOnIndividualReports());
    sync->setRunOnHost(true);
    sync->setSyncThreshold(10ms);
    auto images = sync->inputs["image"].createInputQueue();
    auto reports = sync->inputs["imu"].createInputQueue();
    auto output = sync->out.createOutputQueue();
    pipeline.start();
    images->send(image(100));
    reports->send(imu(81, 99, 1));
    bool timedOut = false;
    auto group = output->get<dai::MessageGroup>(1s, timedOut);
    pipeline.stop();
    REQUIRE_FALSE(timedOut);
    REQUIRE(group != nullptr);
    REQUIRE(group->get<dai::IMUData>("imu")->getSequenceNum() == 1);
}

TEST_CASE("Report-aware Sync uses image exposure middle in each clock domain") {
    dai::Pipeline pipeline(false);
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setTimestampSource(
        GENERATE(dai::node::Sync::TimestampSource::DEVICE, dai::node::Sync::TimestampSource::HOST, dai::node::Sync::TimestampSource::SYSTEM));
    sync->setSyncOnIndividualReports(true);
    sync->setSyncThreshold(10ms);
    auto images = sync->inputs["image"].createInputQueue();
    auto reports = sync->inputs["imu"].createInputQueue();
    auto output = sync->out.createOutputQueue();
    auto frame = image(120);
    frame->cam.exposureTimeUs = 40000;
    auto packet = imu(98, 102, 2);
    pipeline.start();
    images->send(frame);
    reports->send(packet);
    bool timedOut = false;
    auto group = output->get<dai::MessageGroup>(1s, timedOut);
    pipeline.stop();
    REQUIRE_FALSE(timedOut);
    REQUIRE(group != nullptr);
    REQUIRE(group->get<dai::ImgFrame>("image") == frame);
    REQUIRE(group->get<dai::IMUData>("imu") == packet);
    REQUIRE(frame->getTimestampDevice() == std::chrono::steady_clock::time_point(120ms));
    REQUIRE(packet->packets[0].acceleroMeter.getTimestampDevice() == std::chrono::steady_clock::time_point(98ms));
    REQUIRE(group->getTimestampDevice() == std::chrono::steady_clock::time_point(120ms));
}

TEST_CASE("Report-aware Sync rejects old members across every batched packet") {
    dai::Pipeline pipeline(false);
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setTimestampSource(dai::node::Sync::TimestampSource::DEVICE);
    sync->setSyncOnIndividualReports(true);
    auto images = sync->inputs["image"].createInputQueue();
    auto reports = sync->inputs["imu"].createInputQueue();
    auto output = sync->out.createOutputQueue();
    auto batch = imu(98, 102, 1);
    auto older = imu(98, 102, 0)->packets.front();
    const auto member = GENERATE(0, 1, 2, 3);
    if(member == 0) setReport(older.acceleroMeter, 70);
    if(member == 1) setReport(older.gyroscope, 70);
    if(member == 2) setReport(older.magneticField, 70);
    if(member == 3) setReport(older.rotationVector, 70);
    batch->packets.push_back(older);
    pipeline.start();
    images->send(image(100));
    reports->send(batch);
    reports->send(imu(98, 102, 2));
    bool timedOut = false;
    auto group = output->get<dai::MessageGroup>(1s, timedOut);
    pipeline.stop();
    REQUIRE_FALSE(timedOut);
    REQUIRE(group != nullptr);
    REQUIRE(group->get<dai::IMUData>("imu")->getSequenceNum() == 2);
}

TEST_CASE("Report-aware Sync stops with malformed IMU reports and remains cancellable") {
    dai::Pipeline pipeline(false);
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setTimestampSource(dai::node::Sync::TimestampSource::SYSTEM);
    sync->setSyncOnIndividualReports(true);
    auto images = sync->inputs["image"].createInputQueue();
    auto reports = sync->inputs["imu"].createInputQueue();
    auto output = sync->out.createOutputQueue();
    auto data = imu(98, 102, 1);
    const auto scenario = GENERATE(0, 1, 2, 3);
    if(scenario == 0) data->packets.clear();
    if(scenario == 1) data->packets = {dai::IMUPacket{}};
    if(scenario == 2) data->packets[0].acceleroMeter.tsSystem.reset();
    pipeline.start();
    images->send(image(100));
    if(scenario != 3) reports->send(data);
    if(scenario != 3) {
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        while(pipeline.isRunning() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(5ms);
        REQUIRE_FALSE(pipeline.isRunning());
        REQUIRE_THROWS_AS(output->get<dai::MessageGroup>(), dai::MessageQueue::QueueException);
    }
    const auto start = std::chrono::steady_clock::now();
    pipeline.stop();
    REQUIRE(std::chrono::steady_clock::now() - start < 1s);
}
