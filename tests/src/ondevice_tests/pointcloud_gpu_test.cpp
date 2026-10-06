#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"

// The PointCloud node runs on an RVC4 device, once on the CPU and once on the GPU (OpenCL). Both get the same
// synthetic depth (and color) frames through input queues; the GPU cloud has to equal the CPU cloud. On a device
// without a GPU the GPU node falls back to the CPU and the comparison still has to hold.

namespace {

constexpr unsigned W = 640, H = 400;
constexpr float FX = 500.f, FY = 500.f, CX = 320.f, CY = 200.f;
constexpr auto OUTPUT_TIMEOUT = std::chrono::seconds(10);

std::shared_ptr<dai::ImgFrame> makeDepthFrame(const dai::ImgTransformation& transformation, std::chrono::steady_clock::time_point timestamp, unsigned seed) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(W);
    frame->setHeight(H);
    frame->setType(dai::ImgFrame::Type::RAW16);
    std::vector<uint16_t> depth(W * H);
    for(size_t i = 0; i < depth.size(); ++i) {
        // Mix of invalid pixels and a smooth depth ramp
        depth[i] = (i % 7 == 0) ? 0 : static_cast<uint16_t>(600 + ((i * 13 + seed * 101) % 2400));
    }
    std::vector<uint8_t> bytes(depth.size() * sizeof(uint16_t));
    std::memcpy(bytes.data(), depth.data(), bytes.size());
    frame->setData(std::move(bytes));
    frame->setTransformation(transformation);
    frame->setTimestamp(timestamp);
    frame->setSequenceNum(seed);
    return frame;
}

std::shared_ptr<dai::ImgFrame> makeColorFrame(const dai::ImgTransformation& transformation, std::chrono::steady_clock::time_point timestamp, unsigned seed) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(W);
    frame->setHeight(H);
    frame->setType(dai::ImgFrame::Type::RGB888i);
    std::vector<uint8_t> data(W * H * 3);
    for(size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>((i * 31 + seed) % 251);
    frame->setData(std::move(data));
    frame->setTransformation(transformation);
    frame->setTimestamp(timestamp);
    frame->setSequenceNum(seed);
    return frame;
}

dai::ImgTransformation makeTransformation() {
    const std::array<std::array<float, 3>, 3> intrinsics = {{{FX, 0.f, CX}, {0.f, FY, CY}, {0.f, 0.f, 1.f}}};
    // Rotation around Z plus a translation: the extrinsics have to be applied exactly once, on the GPU or the CPU
    dai::Extrinsics extrinsics({{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}, {15.f, -25.f, 35.f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    // Mild radial distortion exercised through the ray table
    return dai::ImgTransformation(W, H, intrinsics, dai::CameraModel::Perspective, {0.05f, -0.01f}, extrinsics);
}

template <typename PointT>
void requireSamePoints(const std::vector<PointT>& gpu, const std::vector<PointT>& cpu) {
    REQUIRE(gpu.size() == cpu.size());
    REQUIRE_FALSE(gpu.empty());
    float worst = 0.f;
    for(size_t i = 0; i < cpu.size(); ++i) {
        worst = std::max({worst, std::abs(gpu[i].x - cpu[i].x), std::abs(gpu[i].y - cpu[i].y), std::abs(gpu[i].z - cpu[i].z)});
        if constexpr(std::is_same_v<PointT, dai::Point3fRGBA>) {
            REQUIRE(gpu[i].r == cpu[i].r);
            REQUIRE(gpu[i].g == cpu[i].g);
            REQUIRE(gpu[i].b == cpu[i].b);
        }
    }
    INFO("largest coordinate difference GPU vs CPU: " << worst << " mm");
    REQUIRE(worst < 0.01f);  // float rounding only (values are up to 3000 mm)
}

/// Collects device log lines so that the test can tell whether the OpenCL backend was really used
struct DeviceLogCapture {
    std::mutex mutex;
    std::vector<std::string> lines;

    explicit DeviceLogCapture(dai::Device& device, dai::LogLevel level = dai::LogLevel::INFO) {
        device.setLogLevel(level);
        device.addLogCallback([this](dai::LogMessage message) {
            std::lock_guard<std::mutex> lock(mutex);
            lines.push_back(message.payload);
        });
    }

    bool contains(const std::string& needle) {
        std::lock_guard<std::mutex> lock(mutex);
        return std::any_of(lines.begin(), lines.end(), [&](const std::string& line) { return line.find(needle) != std::string::npos; });
    }

    /// Average of the numbers following `prefix` in the captured lines, e.g. the per-group compute time the node reports
    std::optional<double> averageAfter(const std::string& prefix) {
        std::lock_guard<std::mutex> lock(mutex);
        double sum = 0.0;
        int count = 0;
        for(const auto& line : lines) {
            const auto pos = line.find(prefix);
            if(pos == std::string::npos) continue;
            try {
                sum += std::stod(line.substr(pos + prefix.size()));
                ++count;
            } catch(const std::exception&) {
            }
        }
        if(count == 0) return std::nullopt;
        return sum / count;
    }
};

constexpr const char* COMPUTE_TIME_LOG = "PointCloud: compute ";

constexpr const char* GPU_READY_LOG = "PointCloud GPU backend ready";
constexpr const char* GPU_FALLBACK_LOG = "falling back to CPU";

/// On a device with a GPU the OpenCL backend has to be the one computing; without one the fallback has to be reported
void requireGpuPathAsExpected(dai::Device& device, DeviceLogCapture& logs) {
    // Device logs arrive asynchronously, give them a moment
    for(int i = 0; i < 50 && !logs.contains(GPU_READY_LOG) && !logs.contains(GPU_FALLBACK_LOG); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if(device.hasGPU()) {
        REQUIRE(logs.contains(GPU_READY_LOG));
        REQUIRE_FALSE(logs.contains(GPU_FALLBACK_LOG));
    } else {
        REQUIRE(logs.contains(GPU_FALLBACK_LOG));
    }
}

struct TwoNodes {
    dai::Pipeline pipeline;
    std::shared_ptr<dai::node::PointCloud> cpu;
    std::shared_ptr<dai::node::PointCloud> gpu;
    std::shared_ptr<dai::InputQueue> cpuDepth, gpuDepth, cpuColor, gpuColor;
    std::shared_ptr<dai::MessageQueue> cpuOut, gpuOut;
    std::unique_ptr<DeviceLogCapture> logs;

    explicit TwoNodes(bool organized, bool color) {
        if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC4) {
            logs = std::make_unique<DeviceLogCapture>(*pipeline.getDefaultDevice());
        }
        auto make = [&](bool useGpu) {
            auto node = pipeline.create<dai::node::PointCloud>();
            node->setRunOnHost(false);
            node->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
            node->initialConfig->setOrganized(organized);
            if(useGpu) {
                node->useGPU();
            } else {
                node->useCPU();
            }
            return node;
        };
        cpu = make(false);
        gpu = make(true);
        cpuDepth = cpu->inputDepth.createInputQueue();
        gpuDepth = gpu->inputDepth.createInputQueue();
        if(color) {
            cpuColor = cpu->getColorInput().createInputQueue();
            gpuColor = gpu->getColorInput().createInputQueue();
        }
        cpuOut = cpu->outputPointCloud.createOutputQueue(4, false);
        gpuOut = gpu->outputPointCloud.createOutputQueue(4, false);
    }

    std::pair<std::shared_ptr<dai::PointCloudData>, std::shared_ptr<dai::PointCloudData>> sendAndReceive(unsigned seed, bool color) {
        const auto transformation = makeTransformation();
        const auto now = std::chrono::steady_clock::now();
        auto depth = makeDepthFrame(transformation, now, seed);
        cpuDepth->send(depth);
        gpuDepth->send(depth);
        if(color) {
            auto colorFrame = makeColorFrame(transformation, now, seed);
            cpuColor->send(colorFrame);
            gpuColor->send(colorFrame);
        }
        bool timedOut = false;
        auto cpuCloud = cpuOut->get<dai::PointCloudData>(OUTPUT_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        auto gpuCloud = gpuOut->get<dai::PointCloudData>(OUTPUT_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(cpuCloud != nullptr);
        REQUIRE(gpuCloud != nullptr);
        return {gpuCloud, cpuCloud};
    }
};

bool skipUnlessRvc4(dai::Pipeline& pipeline) {
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) {
        WARN("Skipping: the on-device PointCloud GPU path exists on RVC4 only");
        return true;
    }
    if(!pipeline.getDefaultDevice()->hasGPU()) {
        WARN("Device reports no GPU: the GPU node has to fall back to the CPU, the comparison still holds");
    }
    return false;
}

}  // namespace

TEST_CASE("On-device GPU point cloud equals the CPU point cloud", "[PointCloud][GPU]") {
    const bool organized = GENERATE(false, true);
    TwoNodes nodes(organized, false);
    if(skipUnlessRvc4(nodes.pipeline)) return;
    nodes.pipeline.start();

    for(unsigned seed = 0; seed < 5; ++seed) {
        auto [gpuCloud, cpuCloud] = nodes.sendAndReceive(seed, false);
        REQUIRE(gpuCloud->isOrganized() == organized);
        REQUIRE(gpuCloud->getWidth() == cpuCloud->getWidth());
        REQUIRE(gpuCloud->getHeight() == cpuCloud->getHeight());
        if(organized) {
            REQUIRE(cpuCloud->getPoints().size() == W * H);
        } else {
            REQUIRE(cpuCloud->getPoints().size() == W * H - (W * H + 6) / 7);
        }
        requireSamePoints(gpuCloud->getPoints(), cpuCloud->getPoints());
    }
    requireGpuPathAsExpected(*nodes.pipeline.getDefaultDevice(), *nodes.logs);
    nodes.pipeline.stop();
}

TEST_CASE("On-device GPU colored point cloud equals the CPU point cloud", "[PointCloud][GPU][Colored]") {
    TwoNodes nodes(false, true);
    if(skipUnlessRvc4(nodes.pipeline)) return;
    nodes.pipeline.start();

    for(unsigned seed = 0; seed < 3; ++seed) {
        auto [gpuCloud, cpuCloud] = nodes.sendAndReceive(seed, true);
        REQUIRE(cpuCloud->isColor());
        REQUIRE(gpuCloud->isColor());
        requireSamePoints(gpuCloud->getPointsRGB(), cpuCloud->getPointsRGB());
    }
    requireGpuPathAsExpected(*nodes.pipeline.getDefaultDevice(), *nodes.logs);
    nodes.pipeline.stop();
}

TEST_CASE("On-device point cloud compute rate, GPU vs CPU, synthetic 1280x800 depth", "[PointCloud][GPU][Throughput]") {
    // The node is fed from the host as fast as it accepts frames (blocking input queue), so the output rate is bound by the
    // node's compute or by the link (about 2 MB per frame), not by a camera. The numbers are reported, basic sanity asserted.
    constexpr unsigned BW = 1280, BH = 800;
    constexpr int FRAMES = 60;
    auto measure = [&](bool useGpu) -> double {
        dai::Pipeline pipeline;
        if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) return -1.0;
        // The node reports its compute time per group at debug level every 30 groups: that is the on-device number,
        // independent of the link (a 1280x800 cloud is 12 MB, the download alone takes ~100 ms on gigabit ethernet)
        DeviceLogCapture logs(*pipeline.getDefaultDevice(), dai::LogLevel::DEBUG);
        auto node = pipeline.create<dai::node::PointCloud>();
        node->setRunOnHost(false);
        node->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
        if(useGpu) {
            node->useGPU();
        } else {
            node->useCPU();
        }
        auto inQ = node->inputDepth.createInputQueue(4, true);
        auto outQ = node->outputPointCloud.createOutputQueue(8, false);
        pipeline.start();

        const std::array<std::array<float, 3>, 3> intrinsics = {{{800.f, 0.f, BW / 2.f}, {0.f, 800.f, BH / 2.f}, {0.f, 0.f, 1.f}}};
        dai::Extrinsics extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {10.f, 0.f, 0.f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        dai::ImgTransformation transformation(BW, BH, intrinsics, dai::CameraModel::Perspective, {0.02f}, extrinsics);
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setWidth(BW);
        frame->setHeight(BH);
        frame->setType(dai::ImgFrame::Type::RAW16);
        std::vector<uint16_t> depth(BW * BH);
        for(size_t i = 0; i < depth.size(); ++i) depth[i] = (i % 9 == 0) ? 0 : static_cast<uint16_t>(500 + (i % 3000));
        std::vector<uint8_t> bytes(depth.size() * sizeof(uint16_t));
        std::memcpy(bytes.data(), depth.data(), bytes.size());
        frame->setData(std::move(bytes));
        frame->setTransformation(transformation);

        // Warm up (first frame initializes intrinsics, ray table and GPU buffers)
        frame->setTimestamp(std::chrono::steady_clock::now());
        inQ->send(frame);
        bool timedOut = false;
        REQUIRE(outQ->get<dai::PointCloudData>(OUTPUT_TIMEOUT, timedOut) != nullptr);
        REQUIRE_FALSE(timedOut);

        const auto start = std::chrono::steady_clock::now();
        std::atomic<bool> stopSending{false};
        std::thread sender([&] {
            for(int i = 0; i < FRAMES && !stopSending; ++i) {
                frame->setTimestamp(std::chrono::steady_clock::now());
                frame->setSequenceNum(i + 1);
                inQ->send(frame);
            }
        });
        // A failed assertion below must not leave the thread joinable (std::terminate); the pipeline stop unblocks the sender
        struct Joiner {
            std::thread& thread;
            std::atomic<bool>& stop;
            dai::Pipeline& pipeline;
            ~Joiner() {
                stop = true;
                if(std::uncaught_exceptions() > 0) pipeline.stop();
                if(thread.joinable()) thread.join();
            }
        } joiner{sender, stopSending, pipeline};
        int received = 0;
        while(received < FRAMES) {
            auto cloud = outQ->get<dai::PointCloudData>(OUTPUT_TIMEOUT, timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(cloud != nullptr);
            ++received;
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        sender.join();
        std::optional<double> computeMs;
        for(int i = 0; i < 50 && !(computeMs = logs.averageAfter(COMPUTE_TIME_LOG)); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(useGpu) requireGpuPathAsExpected(*pipeline.getDefaultDevice(), logs);
        pipeline.stop();
        REQUIRE(computeMs.has_value());
        WARN((useGpu ? "GPU" : "CPU") << " PointCloud on device, synthetic 1280x800 depth: " << *computeMs << " ms compute per cloud on the device, "
                                      << FRAMES / seconds << " clouds/s end to end (12 MB clouds over the link)");
        return *computeMs;
    };
    const double cpuMs = measure(false);
    if(cpuMs < 0) {
        WARN("Skipping: RVC4 only");
        return;
    }
    const double gpuMs = measure(true);
    WARN("On-device compute time per 1280x800 cloud, CPU / GPU: " << cpuMs / gpuMs);
}

TEST_CASE("On-device point cloud throughput, GPU vs CPU", "[PointCloud][GPU][Throughput]") {
    // Depth from the stereo pair at 1280x800 -> PointCloud on the device. The two configurations run one after the other
    // so that they do not compete for the device CPU; the numbers are reported, only basic sanity is asserted.
    constexpr auto MEASURE = std::chrono::seconds(6);
    auto measure = [&](bool useGpu) -> double {
        dai::Pipeline pipeline;
        if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) return -1.0;
        auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
        auto stereo = pipeline.create<dai::node::StereoDepth>();
        auto pointCloud = pipeline.create<dai::node::PointCloud>();
        pointCloud->setRunOnHost(false);
        pointCloud->initialConfig->setLengthUnit(dai::LengthUnit::METER);
        if(useGpu) {
            pointCloud->useGPU();
        } else {
            pointCloud->useCPU();
        }
        left->requestOutput(std::make_pair(1280, 800), std::nullopt, dai::ImgResizeMode::CROP, 30)->link(stereo->left);
        right->requestOutput(std::make_pair(1280, 800), std::nullopt, dai::ImgResizeMode::CROP, 30)->link(stereo->right);
        stereo->depth.link(pointCloud->inputDepth);
        auto depthQ = stereo->depth.createOutputQueue(4, false);
        auto outQ = pointCloud->outputPointCloud.createOutputQueue(4, false);
        pipeline.start();
        // Warm up
        bool timedOut = false;
        REQUIRE(outQ->get<dai::PointCloudData>(OUTPUT_TIMEOUT, timedOut) != nullptr);
        REQUIRE_FALSE(timedOut);
        const auto start = std::chrono::steady_clock::now();
        size_t clouds = 0, depthFrames = 0;
        while(std::chrono::steady_clock::now() - start < MEASURE) {
            clouds += outQ->tryGetAll<dai::PointCloudData>().size();
            depthFrames += depthQ->tryGetAll<dai::ImgFrame>().size();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        pipeline.stop();
        const double seconds = std::chrono::duration<double>(MEASURE).count();
        WARN((useGpu ? "GPU" : "CPU") << " PointCloud on device at 1280x800: " << clouds / seconds << " clouds/s (" << depthFrames / seconds
                                      << " depth frames/s)");
        REQUIRE(clouds > 0);
        return clouds / seconds;
    };
    const double cpuFps = measure(false);
    if(cpuFps < 0) {
        WARN("Skipping: RVC4 only");
        return;
    }
    const double gpuFps = measure(true);
    WARN("Throughput GPU / CPU: " << gpuFps / cpuFps);
}
