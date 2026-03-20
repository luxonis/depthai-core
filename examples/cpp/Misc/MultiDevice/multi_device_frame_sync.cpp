#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
class FPSCounter {
   public:
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> frameTimes;

    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push_back(now);
        if(frameTimes.size() > 100) {
            frameTimes.erase(frameTimes.begin(), frameTimes.begin() + (frameTimes.size() - 100));
        }
    }

    float getFps() {
        if(frameTimes.size() <= 1) {
            return 0.0f;
        }
        // Calculate the FPS
        return float(frameTimes.size() - 1) * 1e6 / std::chrono::duration_cast<std::chrono::microseconds>(frameTimes.back() - frameTimes.front()).count();
    }
};

enum class SyncType {
    EXTERNAL,
    PTP,
};

// ---------------------------------------------------------------------------
// Create camera outputs
// ---------------------------------------------------------------------------
dai::Node::Output* createCameraOutputs(std::shared_ptr<dai::Pipeline> pipeline,
                                       dai::CameraBoardSocket socket,
                                       float sensorFps,
                                       SyncType syncType,
                                       std::optional<dai::ExternalFrameSyncRole> role) {
    std::shared_ptr<dai::node::Camera> cam;
    // Only specify FPS if camera is master
    if(syncType == SyncType::PTP || role == dai::ExternalFrameSyncRole::MASTER) {
        cam = pipeline->create<dai::node::Camera>()->build(socket, std::nullopt, sensorFps);
    } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
        // Slave cameras will lock to the master's FPS
        cam = pipeline->create<dai::node::Camera>()->build(socket, std::nullopt);
    } else {
        throw std::runtime_error("Don't know how to handle role");
    }

    if(syncType == SyncType::PTP) {
        cam->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::TIME_PTP);
        std::cout << "Setting PTP for " << dai::toString(socket) << std::endl;
    }

    auto output = cam->requestOutput(std::make_pair(640, 480), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH);
    return output;
}

// ---------------------------------------------------------------------------
// Create synchronization node
// ---------------------------------------------------------------------------
std::shared_ptr<dai::node::Sync> createSyncNode(std::shared_ptr<dai::Pipeline>& masterPipeline,
                                                std::map<std::string, dai::Node::Output*>& masterNode,
                                                const std::string& masterName,
                                                std::chrono::nanoseconds syncThreshold,
                                                std::vector<std::string>& outputNames,
                                                std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                                                std::map<std::string, std::shared_ptr<dai::InputQueue>>& inputQueues) {
    auto sync = masterPipeline->create<dai::node::Sync>();

    // Sync node will run on the host, since it needs to sync multiple devices
    sync->setRunOnHost(true);
    sync->setSyncThreshold(syncThreshold);

    // Link master camera outputs to the sync node
    for(const auto& p : masterNode) {
        auto name = std::string("master_") + masterName + "_" + p.first;
        p.second->link(sync->inputs[name]);
        outputNames.push_back(name);
    }

    // For slaves, we must create an input queue for each output
    // We will then manually forward the frames from each input queue to the output queue
    // This is because slave devices have separate pipelines from the master
    for(auto& p : slaveQueues) {
        for(auto& q : p.second) {
            auto name = std::string("slave_") + p.first + "_" + q.first;
            outputNames.push_back(name);
            auto input_queue = sync->inputs[name].createInputQueue();
            inputQueues.emplace(name, input_queue);
        }
    }

    return sync;
}

// ---------------------------------------------------------------------------
// Set up for individual camera sockets
// ---------------------------------------------------------------------------
void setUpCameraSocket(std::shared_ptr<dai::Pipeline>& pipeline,
                       dai::CameraBoardSocket socket,
                       std::string& name,
                       float targetFps,
                       SyncType syncType,
                       std::optional<dai::ExternalFrameSyncRole> role,
                       std::optional<std::map<std::string, dai::Node::Output*>>& masterNode,
                       std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                       std::vector<std::string>& camSockets) {
    auto outNode = createCameraOutputs(pipeline, socket, targetFps, syncType, role);

    if(syncType == SyncType::EXTERNAL) {
        // Master cameras will be linked to the sync node directly
        if(role == dai::ExternalFrameSyncRole::MASTER) {
            if(!masterNode.has_value()) {
                masterNode.emplace();
            }
            masterNode.value().emplace(dai::toString(socket), outNode);

            // Gather all slave camera outputs
        } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
            if(slaveQueues.find(name) == slaveQueues.end()) {
                slaveQueues.emplace(name, std::map<std::string, std::shared_ptr<dai::MessageQueue>>());
            }
            slaveQueues[name].emplace(dai::toString(socket), outNode->createOutputQueue());

        } else {
            throw std::runtime_error("Don't know how to handle role");
        }
    } else if(syncType == SyncType::PTP) {
        // For PTP just put the first camera in master.
        // Actual PTP master might be different, but it doesn't matter for this example.
        if(!masterNode.has_value()) {
            masterNode.emplace();
            masterNode.value().emplace(dai::toString(socket), outNode);
        } else {
            if(slaveQueues.find(name) == slaveQueues.end()) {
                slaveQueues.emplace(name, std::map<std::string, std::shared_ptr<dai::MessageQueue>>());
            }
            slaveQueues[name].emplace(dai::toString(socket), outNode->createOutputQueue());
        }
    }

    // Keep track of all camera socket names
    if(std::find(camSockets.begin(), camSockets.end(), dai::toString(socket)) == camSockets.end()) {
        camSockets.emplace_back(dai::toString(socket));
    }
}

std::string getDeviceName(const std::shared_ptr<dai::Device>& device) {
    auto info = device->getDeviceInfo();
    auto name = info.deviceId;
    if(!info.name.empty() || info.name == "") {
        name += "[" + info.name + "]";
    }
    return name;
}

void setupDevice(dai::DeviceInfo& deviceInfo,
                 std::shared_ptr<dai::Pipeline>& masterPipeline,
                 std::optional<std::map<std::string, dai::Node::Output*>>& masterNode,
                 std::optional<std::string>& masterName,
                 std::map<std::string, std::shared_ptr<dai::Pipeline>>& slavePipelines,
                 std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                 std::vector<std::string>& camSockets,
                 float targetFps,
                 SyncType syncType) {
    // Create pipeline for device
    auto pipeline = std::make_shared<dai::Pipeline>(std::make_shared<dai::Device>(deviceInfo));
    auto device = pipeline->getDefaultDevice();

    if(device->getPlatform() != dai::Platform::RVC4) {
        throw std::runtime_error("This example supports only RVC4 platform!");
    }

    auto name = getDeviceName(device);
    std::optional<dai::ExternalFrameSyncRole> role = std::nullopt;
    if(syncType == SyncType::EXTERNAL) {
        role = device->getExternalFrameSyncRole();
    }

    std::cout << "=== Connected to " << deviceInfo.getDeviceId() << std::endl;
    std::cout << "    Device ID: " << device->getDeviceId() << std::endl;
    std::cout << "    Num of cameras: " << device->getConnectedCameras().size() << std::endl;

    for(auto socket : device->getConnectedCameras()) {
        setUpCameraSocket(pipeline, socket, name, targetFps, syncType, role, masterNode, slaveQueues, camSockets);
    }

    if(syncType == SyncType::EXTERNAL) {
        if(role == dai::ExternalFrameSyncRole::MASTER) {
            device->setExternalStrobeEnable(true);
            std::cout << device->getDeviceId() << " is master" << std::endl;

            if(masterPipeline != nullptr) {
                throw std::runtime_error("Only one master pipeline is supported");
            }
            masterPipeline = pipeline;
            masterName = name;
        } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
            slavePipelines[name] = pipeline;
            std::cout << device->getDeviceId() << " is slave" << std::endl;
        } else {
            throw std::runtime_error("Don't know how to handle role");
        }
    } else if(syncType == SyncType::PTP) {
        // For PTP just put the first camera in master.
        // Actual PTP master might be different, but it doesn't matter for this example.
        if(masterPipeline == nullptr) {
            masterPipeline = pipeline;
            masterName = name;
        } else {
            slavePipelines[name] = pipeline;
        }
    }
}

namespace {
struct ParsedArgs {
    float targetFps = 30.0f;
    int recvAllTimeoutSec = 10;
    float syncThresholdSec = 1e-3f;
    int initialSyncTimeoutSec = 4;
    bool externalSync = false;
    bool ptpSync = false;
    std::vector<std::string> deviceArgs;
};

std::atomic_bool running{true};

std::optional<ParsedArgs> parseArguments(int argc, char** argv) {
    ParsedArgs parsed;

    auto isFlag = [](const std::string& arg) {
        static const std::vector<std::string> flags = {
            "-f",
            "--fps",
            "-d",
            "--devices",
            "-t1",
            "--recv-all-timeout-sec",
            "-t2",
            "--sync-threshold-sec",
            "-t3",
            "--initial-sync-timeout-sec",
            "--external-sync",
            "--ptp-sync",
        };
        return std::find(flags.begin(), flags.end(), arg) != flags.end();
    };
    auto printUsage = [argv]() {
        std::cout << "Usage: " << argv[0]
                  << " [-f|--fps <target_fps>] [-d|--devices <device_1> [device_2 ...]]"
                     " [-t1|--recv-all-timeout-sec <sec>] [-t2|--sync-threshold-sec <sec>]"
                     " [-t3|--initial-sync-timeout-sec <sec>] (--external-sync|--ptp-sync)"
                  << std::endl;
    };

    try {
        for(int i = 1; i < argc;) {
            const std::string arg = argv[i];

            auto parseOneValue = [&](const std::string& optName) -> std::string {
                if(i + 1 >= argc) {
                    throw std::runtime_error("Missing value for option " + optName);
                }
                i++;
                return argv[i++];
            };

            if(arg == "-f" || arg == "--fps") {
                parsed.targetFps = std::stof(parseOneValue(arg));
            } else if(arg == "-t1" || arg == "--recv-all-timeout-sec") {
                parsed.recvAllTimeoutSec = std::stoi(parseOneValue(arg));
            } else if(arg == "-t2" || arg == "--sync-threshold-sec") {
                parsed.syncThresholdSec = std::stof(parseOneValue(arg));
            } else if(arg == "-t3" || arg == "--initial-sync-timeout-sec") {
                parsed.initialSyncTimeoutSec = std::stoi(parseOneValue(arg));
            } else if(arg == "-d" || arg == "--devices") {
                i++;
                if(i >= argc || isFlag(argv[i])) {
                    throw std::runtime_error("Option " + arg + " requires at least one device");
                }
                while(i < argc && !isFlag(argv[i])) {
                    parsed.deviceArgs.emplace_back(argv[i++]);
                }
            } else if(arg == "--external-sync") {
                parsed.externalSync = true;
                i++;
            } else if(arg == "--ptp-sync") {
                parsed.ptpSync = true;
                i++;
            } else {
                throw std::runtime_error("Unknown option: " + arg);
            }
        }
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        printUsage();
        return std::nullopt;
    }

    if(parsed.externalSync == parsed.ptpSync) {
        std::cerr << "Must specify exactly one of --external-sync or --ptp-sync" << std::endl;
        printUsage();
        return std::nullopt;
    }

    return parsed;
}
}

void interruptHandler(int sig) {
    if(running.exchange(false)) {
        std::cout << "Interrupted! Exiting..." << std::endl;
    } else {
        std::cout << "Exiting now!" << std::endl;
        exit(0);
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    signal(SIGINT, interruptHandler);

    auto parsedArgs = parseArguments(argc, argv);
    if(!parsedArgs.has_value()) {
        return 1;
    }
    const auto targetFps = parsedArgs->targetFps;
    const auto recvAllTimeoutSec = parsedArgs->recvAllTimeoutSec;
    const auto syncThresholdSec = parsedArgs->syncThresholdSec;
    const auto initialSyncTimeoutSec = parsedArgs->initialSyncTimeoutSec;
    const auto& deviceArgs = parsedArgs->deviceArgs;
    SyncType syncType;
    if(parsedArgs->externalSync) {
        syncType = SyncType::EXTERNAL;
    } else if(parsedArgs->ptpSync) {
        syncType = SyncType::PTP;
        std::cout << "Master camera does not match PTP master, instead it signifies the sync pipeline" << std::endl;
    } else {
        throw std::runtime_error("Must specify sync type");
    }

    std::vector<dai::DeviceInfo> deviceInfos;

    // if user did not specify device IPs, use all available devices
    if(deviceArgs.empty()) {
        deviceInfos = dai::Device::getAllAvailableDevices();
    } else {
        for(const auto& deviceArg : deviceArgs) {
            deviceInfos.emplace_back(deviceArg);
        }
    }

    if(deviceInfos.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        std::exit(0);
    }

    // Variables to keep track of master and slave pipelines and outputs
    std::shared_ptr<dai::Pipeline> masterPipeline;
    std::optional<std::map<std::string, dai::Node::Output*>> masterNode;
    std::optional<std::string> masterName;

    std::map<std::string, std::shared_ptr<dai::Pipeline>> slavePipelines;
    std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>> slaveQueues;

    // keep track of sync node inputs for slaves
    std::map<std::string, std::shared_ptr<dai::InputQueue>> inputQueues;

    // keep track of all sync node output names
    std::vector<std::string> outputNames;

    // keep track of all camera socket names
    std::vector<std::string> camSockets;

    for(auto deviceInfo : deviceInfos) {
        setupDevice(deviceInfo, masterPipeline, masterNode, masterName, slavePipelines, slaveQueues, camSockets, targetFps, syncType);
    }

    if(masterPipeline == nullptr || !masterNode.has_value() || !masterName.has_value()) {
        throw std::runtime_error("No master detected!");
    }
    if(slavePipelines.size() < 1) {
        throw std::runtime_error("No slaves detected!");
    }

    // Create sync node
    // Sync node groups the frames so that all synced frames are timestamped to within one frame time
    auto sync =
        createSyncNode(masterPipeline, *masterNode, *masterName, std::chrono::nanoseconds(long(round(1e9 * 0.5f / targetFps))), outputNames, slaveQueues, inputQueues);
    auto queue = sync->out.createOutputQueue();

    masterPipeline->start();
    for(auto p : slavePipelines) {
        p.second->start();
    }

    FPSCounter fpsCounter;

    std::optional<std::shared_ptr<dai::MessageGroup>> latestFrameGroup;
    bool firstReceived = false;
    auto startTime = std::chrono::steady_clock::now();
    auto prevReceived = std::chrono::steady_clock::now();

    std::optional<std::chrono::time_point<std::chrono::steady_clock>> initialSyncTime;

    bool waitingForInitialSync = true;

    auto dataCollector = [&](const std::string& deviceName, const std::string& socketName) {
        // Send frames from slave output queues to sync node input queues
        const auto queueName = std::string("slave_") + deviceName + "_" + socketName;
        auto camOutputQueue = slaveQueues.at(deviceName).at(socketName);
        auto inputQueue = inputQueues.at(queueName);
        while(running.load()) {
            if(camOutputQueue->has()) {
                inputQueue->send(camOutputQueue->get());
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    };

    std::vector<std::thread> threads;
    for(const auto& slaveEntry : slaveQueues) {
        for(const auto& socketEntry : slaveEntry.second) {
            threads.emplace_back(dataCollector, slaveEntry.first, socketEntry.first);
        }
    }

    while(running.load()) {
        // Get frames from sync node output queue
        while(queue->has()) {
            auto syncData = queue->get();
            latestFrameGroup = std::dynamic_pointer_cast<dai::MessageGroup>(syncData);
            if(!firstReceived) {
                firstReceived = true;
                initialSyncTime = std::chrono::steady_clock::now();
            }
            prevReceived = std::chrono::steady_clock::now();
            fpsCounter.tick();
        }

        // Timeout if we dont receive any frames at the beginning
        if(!firstReceived) {
            auto endTime = std::chrono::steady_clock::now();
            auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
            if(elapsedSec >= recvAllTimeoutSec) {
                std::cout << "Timeout: Didn't receive all frames in time: " << elapsedSec << std::endl;
                running.store(false);
            }
        }

        // -------------------------------------------------------------------
        // Synchronise: we need at least one frame from every camera and their
        // timestamps must align within syncThresholdSec.
        // -------------------------------------------------------------------
        if(latestFrameGroup.has_value() && size_t(latestFrameGroup.value()->getNumMessages()) == outputNames.size()) {
            using ts_type = std::chrono::time_point<std::chrono::steady_clock>;
            std::map<std::string, ts_type> tsValues;
            for(auto name : outputNames) {
                auto frame = latestFrameGroup.value()->get<dai::ImgFrame>(name);
                tsValues.emplace(name, frame->getTimestamp(dai::CameraExposureOffset::END));
            }

            // Build individual image arrays for each camera socket, displayed side-by-side
            std::vector<cv::Mat> imgs;
            std::vector<bool> firstFrameDone;
            for(auto name : camSockets) {
                imgs.emplace_back();
                firstFrameDone.emplace_back(false);
            }

            for(auto outputName : outputNames) {
                // Find out which camera socket this output belongs to
                int32_t idx = -1;
                for(uint32_t i = 0; i < camSockets.size(); i++) {
                    if(outputName.find(camSockets[i]) != std::string::npos) {
                        idx = i;
                        break;
                    }
                }
                if(idx == -1) {
                    throw std::runtime_error(std::string("Could not find camera socket for ") + outputName);
                }

                // Get frame for this output
                auto msg = latestFrameGroup.value()->get<dai::ImgFrame>(outputName);
                auto fps = fpsCounter.getFps();
                auto frame = msg->getCvFrame();

                // Add output name to frame
                cv::putText(frame, outputName, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 127, 255}, 2, cv::LINE_AA);

                // Add timestamp and FPS to frame
                cv::putText(frame,
                            "Timestamp: "
                                + std::to_string(1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(tsValues[outputName].time_since_epoch()).count())
                                + " | FPS: " + std::to_string(fps).substr(0, 5),
                            {20, 80},
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            {255, 0, 50},
                            2,
                            cv::LINE_AA);

                if(!firstFrameDone[idx]) {
                    imgs[idx] = frame;
                    firstFrameDone[idx] = true;
                } else {
                    cv::hconcat(frame, imgs[idx], imgs[idx]);
                }
            }

            // calculate the greatest time difference between all frames
            auto compFunct = [](const std::pair<std::string, ts_type>& p1, const std::pair<std::string, ts_type>& p2) -> bool { return p1.second < p2.second; };

            auto maxElement = std::max_element(tsValues.begin(), tsValues.end(), compFunct);
            auto minElement = std::min_element(tsValues.begin(), tsValues.end(), compFunct);

            auto delta = maxElement->second - minElement->second;

            bool syncStatus = std::chrono::duration_cast<std::chrono::microseconds>(delta).count() < syncThresholdSec * 1e6;
            std::string syncStatusStr = (syncStatus) ? "in sync" : "out of sync";

            cv::Scalar color = (syncStatus) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            // Timeout if frames don't get synced in time
            if(!syncStatus && waitingForInitialSync) {
                auto endTime = std::chrono::steady_clock::now();
                auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - initialSyncTime.value()).count();
                if(elapsedSec >= initialSyncTimeoutSec) {
                    std::cout << "Timeout: Didn't sync frames in time" << std::endl;
                    running.store(false);
                }
            }

            if(syncStatus && waitingForInitialSync) {
                std::cout << "Frame synced" << std::endl;
                waitingForInitialSync = false;
            }

            if(!syncStatus && !waitingForInitialSync) {
                std::cout << "Sync error: Sync lost, threshold exceeded " << std::chrono::duration_cast<std::chrono::microseconds>(delta).count() << " us" << std::endl;
                continue;
            }

            // Add absolute maximum time difference between all frames
            for(auto i = 0; i < imgs.size(); i++) {
                cv::putText(imgs[i],
                            syncStatusStr + " | delta = "
                                + std::to_string(1e-3 * float(std::chrono::duration_cast<std::chrono::microseconds>(delta).count())).substr(0, 5) + " ms",
                            {20, 120},
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            color,
                            2,
                            cv::LINE_AA);
            }

            // Show the frame
            for(auto i = 0; i < imgs.size(); i++) {
                cv::imshow("synced_view_" + camSockets[i], imgs[i]);
            }

            // Wait for next batch
            latestFrameGroup.reset();
        }

        if(cv::waitKey(1) == 'q') {
            running.store(false);
            break;
        }
    }

    for(auto& thread : threads) {
        thread.join();
    }
    cv::destroyAllWindows();

    return 0;
}
