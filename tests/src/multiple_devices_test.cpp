#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include <atomic>
#include <iostream>
#include <tuple>
#include <vector>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

size_t maxFoundDevices = 0;
constexpr auto DEVICE_SEARCH_TIMEOUT = 10s;

TEST_CASE("Multiple devices with 50 messages each") {
    constexpr auto NUM_MESSAGES = 50;
    constexpr auto TEST_TIMEOUT = 20s;
    mutex mtx;
    vector<thread> threads;
    vector<tuple<shared_ptr<dai::Device>, int>> devices;
    int deviceCounter = 0;

    // Wait to acquire more than 1 device. Otherwise perform the test with 1 device
    // Could also fail instead - but requires test groups before
    auto t1 = steady_clock::now();
    vector<dai::DeviceInfo> availableDevices;
    do {
        availableDevices = dai::Device::getAllAvailableDevices();
        this_thread::sleep_for(500ms);
    } while((availableDevices.size() < 2 || availableDevices.size() < maxFoundDevices) && (steady_clock::now() - t1 <= DEVICE_SEARCH_TIMEOUT));
    REQUIRE(!availableDevices.empty());
    REQUIRE(availableDevices.size() >= maxFoundDevices);
    maxFoundDevices = availableDevices.size();

    for(const auto& dev : availableDevices) {
        threads.emplace_back([&mtx, &devices, dev, deviceCounter]() {
            // Create pipeline
            dai::Pipeline pipeline;

            // Define source and output
            auto camRgb = pipeline.create<dai::node::ColorCamera>();
            auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

            xoutRgb->setStreamName("rgb");

            // Properties
            camRgb->setPreviewSize(300, 300);
            camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
            camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
            camRgb->setInterleaved(false);
            camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

            // Linking
            camRgb->preview.link(xoutRgb->input);

            // Optional delay between device connection
            // if(deviceCounter) this_thread::sleep_for(1s);
            std::ignore = deviceCounter;

            auto device = make_shared<dai::Device>(pipeline, dev, dai::UsbSpeed::SUPER);

            cout << "MXID: " << device->getMxId() << endl;
            cout << "Connected cameras: ";
            for(const auto& cam : device->getConnectedCameras()) {
                cout << cam << " ";
            }
            cout << endl;

            unique_lock<std::mutex> l(mtx);
            devices.emplace_back(device, 0);
        });

        deviceCounter++;
    }

    // Join device threads
    for(auto& thread : threads) {
        thread.join();
    }

    bool finished = devices.empty();
    t1 = steady_clock::now();
    while (!finished && steady_clock::now() - t1 < TEST_TIMEOUT) {
        finished = true;
        for(auto& devCounter : devices) {
            auto& dev = get<0>(devCounter);
            auto& counter = get<1>(devCounter);
            if(dev->getOutputQueue("rgb")->tryGet<dai::ImgFrame>()) {
                cout << "Device " << dev->getMxId() << " message arrived (" << ++counter << "/" << NUM_MESSAGES << ")" << endl;
            }

            if(counter < NUM_MESSAGES) {
                finished = false;
            }
        }
        std::this_thread::sleep_for(1ms);
    }

    REQUIRE(finished);
}

TEST_CASE("Multiple devices created and destroyed in parallel", "[.][multi_test_devices]") {
    constexpr auto TEST_TIMEOUT = 30s;
    constexpr auto COLOR_FPS = 30;
    constexpr auto MONO_FPS = 30;
    constexpr auto COLOR_THRESHOLD = 0;
    constexpr auto DEPTH_THRESHOLD = 0;
    vector<thread> threads;

    // Wait to acquire devices. Require 2+ devices.
    const auto t1 = steady_clock::now();
    vector<dai::DeviceInfo> availableDevices;
    do {
        availableDevices = dai::Device::getAllAvailableDevices();
        this_thread::sleep_for(500ms);
    } while((availableDevices.size() < 2 || availableDevices.size() < maxFoundDevices) && (steady_clock::now() - t1 <= DEVICE_SEARCH_TIMEOUT));
    REQUIRE(availableDevices.size() >= 2);
    REQUIRE(availableDevices.size() >= maxFoundDevices);
    maxFoundDevices = availableDevices.size();
    cout << "Found device count: " << availableDevices.size() << endl;

    // preallocate to avoid reallocation/move
    threads.reserve(availableDevices.size());

    // create a separate thread for every hardware device that configures color and depth nodes,
    // starts the device, and creates a simple callback for these two data streams
    for(const auto& selectedDevice : availableDevices) {
        threads.emplace_back([=, &selectedDevice]() mutable {
            while(steady_clock::now() - t1 < TEST_TIMEOUT) {
                // Create pipeline
                dai::Pipeline pipeline;

                // Define color source and output
                auto camColor = pipeline.create<dai::node::ColorCamera>();
                auto xoutColor = pipeline.create<dai::node::XLinkOut>();
                xoutColor->setStreamName("color");
                camColor->video.link(xoutColor->input);

                // Color properties
                camColor->setBoardSocket(dai::CameraBoardSocket::RGB);
                camColor->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
                camColor->setFps(static_cast<float>(COLOR_FPS));

                // Define stereo depth source and output
                auto monoLeft = pipeline.create<dai::node::MonoCamera>();
                auto monoRight = pipeline.create<dai::node::MonoCamera>();
                auto stereo = pipeline.create<dai::node::StereoDepth>();
                auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
                monoLeft->out.link(stereo->left);
                monoRight->out.link(stereo->right);
                xoutDepth->setStreamName("depth");
                stereo->depth.link(xoutDepth->input);

                // Depth properties
                monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
                monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
                monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
                monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
                monoLeft->setFps(static_cast<float>(MONO_FPS));
                monoRight->setFps(static_cast<float>(MONO_FPS));
                stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);

                // Create device
                std::shared_ptr<dai::Device> device;
                try {
                    device = make_shared<dai::Device>(pipeline, selectedDevice, dai::UsbSpeed::SUPER);
                }
                catch(const std::exception& e) {
                    FAIL_CHECK("Fail construct Device() with " << selectedDevice.toString() << " " << e.what());
                    continue;
                }

                // Call RPC
                try {
                    cout << "Pipeline running on MXID: " << device->getMxId() << endl;
                }
                catch(const std::exception& e) {
                    FAIL_CHECK("Fail Device::getMxId() with " << selectedDevice.toString() << " " << e.what());
                    continue;
                }

                // Create queue callbacks for color and depth
                if(COLOR_THRESHOLD > 0) {
                    auto colorQueue = device->getOutputQueue("color", 0, false);
                    colorQueue->addCallback(
                        [colorQueue, passedThreshold = false, COLOR_THRESHOLD](std::shared_ptr<dai::ADatatype> data) mutable {
                            auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
                            auto sequenceNum = frame->getSequenceNum();
                            if(sequenceNum < COLOR_THRESHOLD) {
                                if (sequenceNum % (COLOR_THRESHOLD / 10) == 0)
                                    cout << '.';
                            } else if(!passedThreshold) {
                                passedThreshold = true;
                            }
                        });
                } else {
                }
                if(DEPTH_THRESHOLD > 0) {
                    auto depthQueue = device->getOutputQueue("depth", 0, false);
                    depthQueue->addCallback(
                        [depthQueue, passedThreshold = false, DEPTH_THRESHOLD](std::shared_ptr<dai::ADatatype> data) mutable {
                            auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
                            auto sequenceNum = frame->getSequenceNum();
                            if(sequenceNum < DEPTH_THRESHOLD) {
                                if (sequenceNum % (DEPTH_THRESHOLD / 10) == 0)
                                    cout << '.';
                            } else if(!passedThreshold) {
                                passedThreshold = true;
                            }
                        });
                } else {
                }
                SUCCEED("Successful Device with " << selectedDevice.toString());
            }
        });
    }

    // Join device threads
    for(auto& thread : threads) {
        thread.join();
    }
}

TEST_CASE("Device APIs after Device::close()") {
    constexpr auto TEST_TIMEOUT = 20s;

    // Wait to acquire a device.
    auto t1 = steady_clock::now();
    vector<dai::DeviceInfo> availableDevices;
    do {
        availableDevices = dai::Device::getAllAvailableDevices();
        this_thread::sleep_for(500ms);
    } while((availableDevices.empty() || availableDevices.size() < maxFoundDevices) && (steady_clock::now() - t1 <= DEVICE_SEARCH_TIMEOUT));
    REQUIRE(!availableDevices.empty());
    REQUIRE(availableDevices.size() >= maxFoundDevices);
    maxFoundDevices = availableDevices.size();
    cout << "Found device count: " << availableDevices.size() << endl;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define color source and output
    auto camColor = pipeline.create<dai::node::ColorCamera>();
    auto xoutColor = pipeline.create<dai::node::XLinkOut>();
    xoutColor->setStreamName("color");
    camColor->video.link(xoutColor->input);

    // Color properties
    camColor->setBoardSocket(dai::CameraBoardSocket::RGB);
    camColor->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camColor->setFps(30.0f);

    // Create device
    auto device = make_shared<dai::Device>(pipeline, availableDevices[0], dai::UsbSpeed::SUPER);
    cout << "MXID: " << device->getMxId() << "\nConnected cameras: ";
    for(const auto& cam : device->getConnectedCameras()) {
        cout << cam << " ";
    }
    cout << endl;

    // Get color queue and wait for one frame
    bool ignore{};
    if(device->getOutputQueue("color")->get<dai::ImgFrame>(TEST_TIMEOUT, ignore)) {
        cout << "Device " << device->getMxId() << " message arrived" << endl;
    }

    // Close device
    device->close();

    // Validate Device API behaviors
    CHECK_THROWS_AS(std::ignore = device->getMxId(), std::system_error);
    CHECK_THROWS_AS(device->setXLinkChunkSize(1024), std::system_error);
    CHECK_THROWS_AS(std::ignore = device->readCalibration(), std::system_error);
    CHECK_NOTHROW(std::ignore = device->getDeviceInfo().name);
    CHECK_NOTHROW(device->setLogOutputLevel(dai::LogLevel::WARN));
}
