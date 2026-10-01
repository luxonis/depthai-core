// Multi-device calibration with explicit constraints.
//
// multi_device_calibration.cpp only needs a factory stereo pair per device. This
// example covers the optional MultiDeviceCalibration configuration:
//
//   * setKnownDistance     metric scale from a measured camera-to-camera distance,
//                          so a single camera per device is enough
//   * setInitialGuess      an approximate rig layout as the solver's starting point
//   * setDeviceCalibration calibration loaded from a file instead of the device EEPROM
//   * getSampleCount       progress reporting
//
// Example: two devices 80 cm apart, both looking at the same wall, the second one
// rotated 15 degrees to the left (about the vertical axis):
//
//   multi_device_calibration_constraints -d <A> <B> --known-distance <A> <B> 80 --initial-guess <A> <B> -80 0 0 15 0 0
//
// Device tokens are the IDs printed by the program, or whatever was passed to -d.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

struct KnownDistanceArg {
    std::string from;
    std::string to;
    float centimeters = 0.0f;
};

struct InitialGuessArg {
    std::string from;
    std::string to;
    float x = 0.0f, y = 0.0f, z = 0.0f;           // cm
    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;  // degrees
};

struct ParsedArgs {
    std::vector<std::string> deviceArgs;
    dai::CameraBoardSocket socket = dai::CameraBoardSocket::CAM_A;
    std::size_t sampleCount = 10;
    std::filesystem::path output = "multi_device_calibration_constraints.json";
    std::vector<KnownDistanceArg> knownDistances;
    std::vector<InitialGuessArg> initialGuesses;
    std::map<std::string, std::filesystem::path> calibrationFiles;
};

dai::CameraBoardSocket parseSocket(const std::string& name) {
    for(auto socket : {dai::CameraBoardSocket::CAM_A,
                       dai::CameraBoardSocket::CAM_B,
                       dai::CameraBoardSocket::CAM_C,
                       dai::CameraBoardSocket::CAM_D,
                       dai::CameraBoardSocket::CAM_E,
                       dai::CameraBoardSocket::CAM_F,
                       dai::CameraBoardSocket::CAM_G,
                       dai::CameraBoardSocket::CAM_H}) {
        if(name == std::string(dai::toString(socket))) return socket;
    }
    throw std::runtime_error("Unknown camera socket: " + name);
}

std::optional<ParsedArgs> parseArguments(int argc, char** argv) {
    ParsedArgs parsed;
    auto printUsage = [argv]() {
        std::cout << "Usage: " << argv[0]
                  << " [-d|--devices <device_1> <device_2> ...] [-s|--socket CAM_A] [-n|--sample-count <count>] [-o|--output <path>]\n"
                     "       [--known-distance FROM TO CM]... [--initial-guess FROM TO X Y Z YAW PITCH ROLL]... [--calibration DEVICE PATH]..."
                  << std::endl;
    };
    auto need = [&](int i, int count, const std::string& arg) {
        if(i + count >= argc) throw std::runtime_error("Option " + arg + " needs " + std::to_string(count) + " values");
    };
    try {
        for(int i = 1; i < argc;) {
            const std::string arg = argv[i];
            if(arg == "-d" || arg == "--devices") {
                i++;
                while(i < argc && argv[i][0] != '-') parsed.deviceArgs.emplace_back(argv[i++]);
                if(parsed.deviceArgs.size() < 2) throw std::runtime_error("Option " + arg + " requires at least two devices");
            } else if(arg == "-s" || arg == "--socket") {
                need(i, 1, arg);
                parsed.socket = parseSocket(argv[i + 1]);
                i += 2;
            } else if(arg == "-n" || arg == "--sample-count") {
                need(i, 1, arg);
                parsed.sampleCount = static_cast<std::size_t>(std::stoul(argv[i + 1]));
                i += 2;
            } else if(arg == "-o" || arg == "--output") {
                need(i, 1, arg);
                parsed.output = argv[i + 1];
                i += 2;
            } else if(arg == "--known-distance") {
                need(i, 3, arg);
                parsed.knownDistances.push_back({argv[i + 1], argv[i + 2], std::stof(argv[i + 3])});
                i += 4;
            } else if(arg == "--initial-guess") {
                need(i, 8, arg);
                parsed.initialGuesses.push_back({argv[i + 1],
                                                 argv[i + 2],
                                                 std::stof(argv[i + 3]),
                                                 std::stof(argv[i + 4]),
                                                 std::stof(argv[i + 5]),
                                                 std::stof(argv[i + 6]),
                                                 std::stof(argv[i + 7]),
                                                 std::stof(argv[i + 8])});
                i += 9;
            } else if(arg == "--calibration") {
                need(i, 2, arg);
                parsed.calibrationFiles[argv[i + 1]] = argv[i + 2];
                i += 3;
            } else if(arg == "-h" || arg == "--help") {
                printUsage();
                return std::nullopt;
            } else {
                throw std::runtime_error("Unknown option: " + arg);
            }
        }
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        printUsage();
        return std::nullopt;
    }
    return parsed;
}

// R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in degrees
std::vector<std::vector<float>> rotationMatrix(float yawDeg, float pitchDeg, float rollDeg) {
    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
    const float cy = std::cos(yawDeg * kDegToRad), sy = std::sin(yawDeg * kDegToRad);
    const float cp = std::cos(pitchDeg * kDegToRad), sp = std::sin(pitchDeg * kDegToRad);
    const float cr = std::cos(rollDeg * kDegToRad), sr = std::sin(rollDeg * kDegToRad);
    return {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}, {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}, {-sp, cp * sr, cp * cr}};
}

// MultiDeviceCalibration expresses initial guesses and results between the devices'
// local calibration origins, not between the sockets that were registered.
dai::CameraBoardSocket localOriginSocket(const dai::CalibrationHandler& calibration, dai::CameraBoardSocket socket) {
    auto origin = dai::CameraBoardSocket::AUTO;
    calibration.getExtrinsicsToOrigin(socket, false, origin);
    return origin;
}

}  // namespace

int main(int argc, char** argv) {
    auto parsed = parseArguments(argc, argv);
    if(!parsed.has_value()) return 1;

    std::vector<dai::DeviceInfo> deviceInfos;
    std::vector<std::string> tokens;  // what the user called each device, empty when discovered
    if(parsed->deviceArgs.empty()) {
        deviceInfos = dai::Device::getAllAvailableDevices();
        if(deviceInfos.size() > 2) deviceInfos.resize(2);
        if(deviceInfos.size() < 2) {
            std::cout << "At least two devices are required for this example." << std::endl;
            return 0;
        }
        tokens.assign(deviceInfos.size(), "");
    } else {
        for(const auto& arg : parsed->deviceArgs) {
            deviceInfos.emplace_back(arg);
            tokens.push_back(arg);
        }
    }

    const auto socket = parsed->socket;
    constexpr float fps = 5.0f;

    dai::Pipeline pipeline(false);
    auto calibration = pipeline.create<dai::beta::node::MultiDeviceCalibration>();
    calibration->setSampleCount(parsed->sampleCount);
    calibration->sync->setSyncThreshold(std::chrono::seconds(5));

    std::map<std::string, std::string> deviceIdByToken;           // user token or device ID -> device ID
    std::map<std::string, dai::CalibrationHandler> calibrations;  // device ID -> calibration the node will use

    for(std::size_t index = 0; index < deviceInfos.size(); ++index) {
        auto device = pipeline.addDevice(deviceInfos[index]);
        const auto deviceId = device->getDeviceId();
        deviceIdByToken[deviceId] = deviceId;
        if(!tokens[index].empty()) deviceIdByToken[tokens[index]] = deviceId;

        // setDeviceCalibration: override the live calibration with one from a file.
        // Without an override the node reads the device calibration itself.
        auto file = parsed->calibrationFiles.find(tokens[index]);
        if(file == parsed->calibrationFiles.end()) file = parsed->calibrationFiles.find(deviceId);
        if(file != parsed->calibrationFiles.end()) {
            dai::CalibrationHandler handler(file->second);
            calibration->setDeviceCalibration(deviceId, handler);
            calibrations.emplace(deviceId, handler);
            std::cout << "Using device " << deviceId << " with calibration from " << file->second.string() << std::endl;
        } else {
            calibrations.emplace(deviceId, device->readCalibration());
            std::cout << "Using device " << deviceId << std::endl;
        }

        auto camera = pipeline.create<dai::node::Camera>(device)->build(socket, std::nullopt, fps);
        calibration->addCamera(deviceId, socket, *camera->requestFullResolutionOutput(std::nullopt, fps));
    }

    const auto resolve = [&](const std::string& token) {
        const auto it = deviceIdByToken.find(token);
        if(it == deviceIdByToken.end()) throw std::runtime_error("Unknown device " + token);
        return it->second;
    };

    // setKnownDistance: a tape-measured distance between two registered cameras on
    // different devices. With one camera per device this is the only source of scale.
    for(const auto& known : parsed->knownDistances) {
        calibration->setKnownDistance(resolve(known.from), socket, resolve(known.to), socket, known.centimeters, dai::LengthUnit::CENTIMETER);
    }
    if(parsed->knownDistances.empty()) {
        std::cout << "Warning: no --known-distance given; with a single camera per device the solver has no metric scale." << std::endl;
    }

    // setInitialGuess: an approximate pose between two devices' local calibration
    // origins (X_to = R * X_from + t). Helps the solver when the devices are rotated
    // strongly relative to each other.
    for(const auto& initial : parsed->initialGuesses) {
        const auto fromId = resolve(initial.from);
        const auto toId = resolve(initial.to);
        const auto fromOrigin = localOriginSocket(calibrations.at(fromId), socket);
        const auto toOrigin = localOriginSocket(calibrations.at(toId), socket);
        dai::Extrinsics guess(
            rotationMatrix(initial.yaw, initial.pitch, initial.roll), dai::Point3f(initial.x, initial.y, initial.z), toOrigin, dai::LengthUnit::CENTIMETER);
        guess.toDeviceId = toId;
        calibration->setInitialGuess(fromId, fromOrigin, toId, toOrigin, guess);
        std::cout << "Initial guess " << fromId << "/" << dai::toString(fromOrigin) << " -> " << toId << "/" << dai::toString(toOrigin) << std::endl;
    }

    auto controlQueue = calibration->inputControl.createInputQueue();
    auto resultQueue = calibration->calibrationOutput.createOutputQueue();

    std::cout << "Point all devices at the same textured scene and keep them still." << std::endl;
    pipeline.start();
    controlQueue->send(dai::beta::MultiDeviceCalibrationControl::start());
    std::cout << "Collecting " << calibration->getSampleCount() << " synchronized samples..." << std::endl;

    bool hasTimedOut = false;
    auto result = resultQueue->get<dai::beta::MultiDeviceCalibrationResult>(std::chrono::minutes(3), hasTimedOut);
    if(result == nullptr) throw std::runtime_error("Calibration timed out");
    if(!result->passed || !result->graph.has_value()) throw std::runtime_error(result->info);

    const auto handler = result->getHandler();
    if(!handler.has_value() || !handler->toJsonFile(parsed->output)) {
        throw std::runtime_error("Failed to save calibration to " + parsed->output.string());
    }

    std::cout << "Calibration saved to " << parsed->output.string() << std::endl;
    std::cout << std::fixed << std::setprecision(3) << "Confidence: " << result->dataConfidence << std::endl;
    std::cout << std::defaultfloat << std::setprecision(6) << "Sampson error: " << result->sampsonError << std::endl;
    for(const auto& edge : *result->graph) {  // meter-normalized edges towards the reference device
        const auto& t = edge.extrinsics.translation;
        const double distanceCm = 100.0 * std::sqrt(static_cast<double>(t.x * t.x + t.y * t.y + t.z * t.z));
        std::cout << std::fixed << std::setprecision(1) << edge.fromDeviceId << "/" << dai::toString(edge.fromSocket) << " -> " << edge.extrinsics.toDeviceId
                  << "/" << dai::toString(edge.extrinsics.toCameraSocket) << ": " << distanceCm << " cm between origins" << std::endl;
    }

    pipeline.stop();
    return 0;
}
