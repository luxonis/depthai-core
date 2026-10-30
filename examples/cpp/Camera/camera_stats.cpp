#include <array>
#include <chrono>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    if(argc < 3 || argc > 4) {
        std::cerr << "Usage: camera_stats CAM_A CAM_D [seconds=30]\n";
        return 1;
    }
    std::array<dai::CameraBoardSocket, 2> sockets{};
    for(size_t i = 0; i < sockets.size(); ++i) {
        const std::string name = argv[i + 1];
        if(name.size() != 5 || name.substr(0, 4) != "CAM_" || name[4] < 'A' || name[4] > 'J') {
            std::cerr << "Specify physical sockets CAM_A through CAM_J\n";
            return 1;
        }
        sockets[i] = static_cast<dai::CameraBoardSocket>(name[4] - 'A');
    }
    if(sockets[0] == sockets[1]) return 1;
    const auto duration = std::chrono::seconds(argc == 4 ? std::stoi(argv[3]) : 30);
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    for(const auto socket : sockets) {
        auto camera = pipeline.create<dai::node::Camera>()->build(socket, std::nullopt, 15.0f);
        auto* output = camera->requestOutput({320, 240}, std::nullopt, dai::ImgResizeMode::CROP, 15.0f);
        if(output == nullptr) throw std::runtime_error("Sensor does not support the requested output");
        queues.push_back(output->createOutputQueue(1, false));
    }

    using Counter = std::optional<uint64_t> dai::CameraStats::*;
    const std::vector<std::pair<const char*, Counter>> counters{
        {"completedCaptures", &dai::CameraStats::completedCaptures},
        {"halDeviceErrors", &dai::CameraStats::halDeviceErrors},
        {"halRequestErrors", &dai::CameraStats::halRequestErrors},
        {"halResultErrors", &dai::CameraStats::halResultErrors},
        {"halBufferErrors", &dai::CameraStats::halBufferErrors},
        {"cslErrorEvents", &dai::CameraStats::cslErrorEvents},
        {"linkStalledEvents", &dai::CameraStats::linkStalledEvents},
        {"csidFatalEvents", &dai::CameraStats::csidFatalEvents},
        {"csidFifoOverflowEvents", &dai::CameraStats::csidFifoOverflowEvents},
        {"csidRecoveryOverflowEvents", &dai::CameraStats::csidRecoveryOverflowEvents},
        {"csidLaneFifoOverflowEvents", &dai::CameraStats::csidLaneFifoOverflowEvents},
        {"csidPixelCountMismatchEvents", &dai::CameraStats::csidPixelCountMismatchEvents},
        {"csidPacketHeaderCorruptionEvents", &dai::CameraStats::csidPacketHeaderCorruptionEvents},
        {"csidMissingPacketHeaderEvents", &dai::CameraStats::csidMissingPacketHeaderEvents},
        {"csidSensorSwitchingEvents", &dai::CameraStats::csidSensorSwitchingEvents},
        {"csidUnboundedFrameEvents", &dai::CameraStats::csidUnboundedFrameEvents},
        {"icpNoMemoryEvents", &dai::CameraStats::icpNoMemoryEvents},
        {"icpSystemFailureEvents", &dai::CameraStats::icpSystemFailureEvents},
        {"csidMissingEotEvents", &dai::CameraStats::csidMissingEotEvents},
        {"csidPayloadCorruptionEvents", &dai::CameraStats::csidPayloadCorruptionEvents},
        {"sensorStreamOffFailureEvents", &dai::CameraStats::sensorStreamOffFailureEvents},
    };
    std::map<dai::CameraBoardSocket, dai::CameraStats> previous;
    const std::array<const char*, 4> statuses{"AVAILABLE", "INACTIVE", "UNSUPPORTED", "UNAVAILABLE"};
    pipeline.start();
    const auto end = std::chrono::steady_clock::now() + duration;
    auto nextPoll = std::chrono::steady_clock::now();
    while(pipeline.isRunning() && std::chrono::steady_clock::now() < end) {
        // Small nonblocking host queues prevent this polling example from backing up capture.
        for(auto& queue : queues) queue->tryGet<dai::ImgFrame>();
        if(std::chrono::steady_clock::now() >= nextPoll) {
            for(const auto& current : device->getCameraStats()) {
                if(current.socket == dai::CameraBoardSocket::AUTO) {
                    std::cout << "cameraId=" << current.cameraId << ": statistics unavailable; physical socket mapping is unverified\n\n";
                    continue;
                }
                if(current.socket != sockets[0] && current.socket != sockets[1]) continue;
                const auto old = previous.find(current.socket);
                // Counter presence determines validity, including HAL errors in UNAVAILABLE snapshots.
                const bool sameSession = old != previous.end() && !current.sessionId.empty() && old->second.sessionId == current.sessionId
                                         && current.timestampNs > old->second.timestampNs;
                std::cout << current.socket << " status=" << statuses.at(static_cast<size_t>(current.status)) << " session=" << current.sessionId
                          << " timestampNs=" << current.timestampNs << '\n';
                for(const auto& [name, member] : counters) {
                    const auto value = current.*member;
                    std::cout << "  " << name << "=";
                    if(!value) {
                        std::cout << "unavailable\n";
                        continue;
                    }
                    std::cout << *value;
                    const bool csl = member != &dai::CameraStats::completedCaptures && member != &dai::CameraStats::halDeviceErrors
                                     && member != &dai::CameraStats::halRequestErrors && member != &dai::CameraStats::halResultErrors
                                     && member != &dai::CameraStats::halBufferErrors;
                    if(sameSession && old->second.*member
                       && (!csl || (current.cslTimestampNs && old->second.cslTimestampNs && *current.cslTimestampNs > *old->second.cslTimestampNs))) {
                        // uint64_t subtraction implements the documented modulo-2^64 delta.
                        std::cout << " delta=" << (*value - *(old->second.*member));
                    }
                    std::cout << '\n';
                }
                std::cout << '\n';
                previous[current.socket] = current;
            }
            nextPoll += std::chrono::seconds(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
