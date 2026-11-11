#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "depthai/depthai.hpp"

// Short tests miss load-dependent failures, including crashes with dot and flood enabled together.
TEST_CASE("Device sustains camera and processing load for five minutes", "[stress]") {
    using namespace std::chrono_literals;

    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    device->setMaxReconnectionAttempts(0);

    std::map<dai::CameraBoardSocket, std::shared_ptr<dai::node::Camera>> cameras;
    std::map<std::string, dai::Node::Output*> outputs;
    std::shared_ptr<dai::node::Camera> colorCamera;
    dai::Node::Output* colorOutput = nullptr;
    for(const auto& feature : device->getConnectedCameraFeatures()) {
        const auto& types = feature.supportedTypes;
        const bool isColor = std::find(types.begin(), types.end(), dai::CameraSensorType::COLOR) != types.end();
        const bool isMono = std::find(types.begin(), types.end(), dai::CameraSensorType::MONO) != types.end();
        if(!isColor && !isMono) continue;

        auto camera = pipeline.create<dai::node::Camera>()->build(feature.socket);
        // At 30 FPS, 10 ms exposure exercises the full IR duty cycle even in a bright scene.
        camera->initialControl.setManualExposure(10000, 800);
        cameras.emplace(feature.socket, camera);
        // Bound capture to 1080p and align dimensions for H.264 encoding.
        const auto size = std::make_pair(std::min(1920u, camera->getMaxWidth()) / 32 * 32, std::min(1080u, camera->getMaxHeight()) / 2 * 2);
        auto* output = camera->requestOutput(size, isColor ? dai::ImgFrame::Type::NV12 : dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.0f);
        outputs.emplace("camera " + std::to_string(static_cast<int>(feature.socket)), output);
        if(isColor && colorCamera == nullptr) {
            colorCamera = camera;
            colorOutput = output;
        }
    }
    REQUIRE_FALSE(cameras.empty());

    if(colorCamera != nullptr) {
        REQUIRE(colorOutput != nullptr);
        auto encoder = pipeline.create<dai::node::VideoEncoder>()->build(*colorOutput);
        encoder->setDefaultProfilePreset(30.0f, dai::VideoEncoderProperties::Profile::H264_MAIN);
        outputs.emplace("encoder", &encoder->out);

        auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(colorCamera, dai::NNModelDescription{"yolov6-nano"}, 30.0f);
        if(device->getPlatform() == dai::Platform::RVC2) {
            // Camera and stereo processing leave seven CMX slices available for inference.
            neuralNetwork->setNNArchive(neuralNetwork->getNNArchive().value().get(), 6);
            neuralNetwork->setNumInferenceThreads(1);
        }
        neuralNetwork->input.setBlocking(false);
        neuralNetwork->input.setMaxSize(1);
        outputs.emplace("neural network", &neuralNetwork->out);
    }

    const auto stereoPairs = device->getStereoPairs();
    if(!stereoPairs.empty()) {
        const auto& pair = stereoPairs.front();
        auto* left = cameras.at(pair.left)->requestOutput({640, 400}, std::nullopt, dai::ImgResizeMode::CROP, 30.0f);
        auto* right = cameras.at(pair.right)->requestOutput({640, 400}, std::nullopt, dai::ImgResizeMode::CROP, 30.0f);
        REQUIRE(left != nullptr);
        REQUIRE(right != nullptr);
        auto stereo = pipeline.create<dai::node::StereoDepth>()->build(*left, *right);
        stereo->setLeftRightCheck(true);
        outputs.emplace("stereo depth", &stereo->depth);
    }

    // Consume frames on the device so link bandwidth does not throttle the processing load.
    std::map<std::string, std::shared_ptr<dai::MessageQueue>> reports;
    for(const auto& [name, output] : outputs) {
        INFO(name);
        REQUIRE(output != nullptr);
        auto benchmark = pipeline.create<dai::node::BenchmarkIn>();
        benchmark->setRunOnHost(false);
        benchmark->sendReportEveryNMessages(10);
        benchmark->logReportsAsWarnings(false);
        output->link(benchmark->input);
        reports.emplace(name, benchmark->report.createOutputQueue(1, false));
    }

    pipeline.start();
    if(!device->getIrDrivers().empty()) {
        const bool dotEnabled = device->setIrLaserDotProjectorIntensity(1.0f);
        const bool floodEnabled = device->setIrFloodLightIntensity(1.0f);
        if(!dotEnabled) WARN("IR dot projector did not accept the stress intensity");
        if(!floodEnabled) WARN("IR flood light did not accept the stress intensity");
        REQUIRE((dotEnabled || floodEnabled));
    }

    const auto deadline = std::chrono::steady_clock::now() + 5min;
    while(std::chrono::steady_clock::now() < deadline) {
        REQUIRE(pipeline.isRunning());
        REQUIRE_FALSE(device->hasCrashed());
        for(const auto& [name, queue] : reports) {
            INFO(name);
            bool timedOut = false;
            const auto report = queue->get<dai::BenchmarkReport>(15s, timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(report != nullptr);
            REQUIRE(report->fps > 0.0f);
        }
    }
    REQUIRE(pipeline.isRunning());
    REQUIRE_FALSE(device->hasCrashed());
    pipeline.stop();
    pipeline.wait();
}
