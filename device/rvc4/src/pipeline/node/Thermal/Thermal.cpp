#include "Thermal.hpp"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <optional>
#include <stdexcept>

#include "Timesync.hpp"
#include "camera/SeekEagle.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "utilities/CameraDiscovery.hpp"
#include "utilities/ClockOffset.hpp"
#include "utilities/OSUtils.hpp"

extern std::string getSerialNumber();

namespace dai {
namespace gate {

namespace {
bool isThermal(const dai::CameraFeatures& features) {
    return std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::THERMAL) != features.supportedTypes.end();
}
}  // namespace

void Thermal::buildStage2() {
    socket = properties.boardSocket;
    if(socket == CameraBoardSocket::AUTO) {
        for(const auto& features : CameraDiscovery::getInstance().getConnectedCameraFeatures()) {
            if(isThermal(features)) {
                socket = features.socket;
                break;
            }
        }
        if(socket == CameraBoardSocket::AUTO) throw std::invalid_argument("No thermal camera connected");
    }
    const auto cam = CameraDiscovery::getInstance().getCameraDescription("", socket);
    const auto features = CameraDiscovery::getInstance().getCameraFeatures("", socket);
    if(cam.id == -1 || !isThermal(features) || features.configs.empty()) {
        throw std::invalid_argument(fmt::format("No thermal camera on socket {}", dai::toString(socket)));
    }
    const bool temperatureOut = !temperature.getConnections().empty();
    const bool colorOut = !color.getConnections().empty();
    const bool rawOut = !raw.getConnections().empty();
    if(!temperatureOut && !colorOut && !rawOut) {
        logger->warn("Thermal node on socket {} has no output linked, the camera is not started", dai::toString(socket));
        return;
    }
    // The Seek Eagle runs at the frame rate of its own clock; the mode index 0 is its imaging mode
    const auto& mode = features.configs[0];
    logger->info("Thermal node on socket {}: {}, {}x{} wire frames, the core's own frame rate (fps {} not used)",
                 dai::toString(socket),
                 features.sensorName,
                 mode.width,
                 mode.height,
                 properties.fps);
    dai::CameraProperties cameraProperties;
    cameraProperties.boardSocket = socket;
    cameraSensor = std::make_unique<CameraSensor>(
        cam,
        0,
        mode.maxFps,
        LUX_SC_CAMERA_FSYNC_NONE,
        CameraImageOrientation::AUTO,
        cameraProperties,
        getParentPipeline().getEepromData(),
        [this](CameraSensor::FrameCallbackArgs args) { sendFrame(std::move(args)); },
        logger);
    cameraSensor->setupRawOutput();
    seek_eagle::setup(socket, temperatureOut, colorOut, rawOut, properties.initialConfig);
}

void Thermal::buildStage3() {
    if(cameraSensor) cameraSensor->build(dai::CameraSensorType::THERMAL);
}

void Thermal::sendFrame(CameraSensor::FrameCallbackArgs args) {
    using namespace std::chrono;
    if(transformationSize != std::make_pair(args.width, args.height)) {
        transformation = ImgTransformation(args.width, args.height);
        auto pipeline = getParentPipeline();
        if(pipeline.isCalibrationDataAvailable()) {
            const auto calib = pipeline.getCalibrationData();
            if(calib.getEepromData().cameraData.count(socket) > 0) {
                const auto intrinsics = calib.getCameraIntrinsics(socket, args.width, args.height);
                std::array<std::array<float, 3>, 3> matrix{};
                for(size_t i = 0; i < 3; i++)
                    for(size_t j = 0; j < 3; j++) matrix[i][j] = intrinsics[i][j];
                transformation =
                    ImgTransformation(args.width, args.height, matrix, calib.getDistortionModel(socket), calib.getDistortionCoefficients(socket));
                // Extrinsics to the camera with the lowest socket, as in the Camera node: Align needs them
                const auto lowestSocket = calib.getCameraWithLowestId();
                try {
                    Extrinsics extrinsics(calib.getCameraExtrinsics(socket, lowestSocket, false, LengthUnit::CENTIMETER), lowestSocket, LengthUnit::CENTIMETER);
                    try {
                        const auto specTranslation = calib.getCameraTranslationVector(socket, lowestSocket, true, LengthUnit::CENTIMETER);
                        extrinsics.specTranslation = {specTranslation[0], specTranslation[1], specTranslation[2]};
                    } catch(const std::exception& e) {
                        logger->debug(
                            "Thermal node on socket {}: no spec translation to socket {}: {}", dai::toString(socket), dai::toString(lowestSocket), e.what());
                    }
                    transformation.setExtrinsics(extrinsics);
                } catch(const std::exception& e) {
                    logger->warn("Thermal node on socket {}: no extrinsics to socket {} in the calibration, so Align can't align its frames: {}",
                                 dai::toString(socket),
                                 dai::toString(lowestSocket),
                                 e.what());
                }
            }
        }
        if(!deviceId.empty()) {
            auto extrinsics = transformation.getExtrinsics();
            extrinsics.toDeviceId = deviceId;
            transformation.setExtrinsics(extrinsics);
        }
        transformationSize = {args.width, args.height};
    }

    // Timestamps as in the Camera node: QTimer -> device steady clock, synchronized host clock and system clock; once per
    // SDK frame, so that both of its outputs get the same
    if(args.sequenceNumber != timestamps.seq) {
        const nanoseconds qtimer = seconds(args.ts.sec) + nanoseconds(args.ts.nsec);
        timestamps.seq = args.sequenceNumber;
        timestamps.system.reset();
        if(getOSVersion() >= dai::Version(1, 21, 0)) {
            timestamps.device = qtimer + SteadyToQTimerOffset::instance().offset();
            timestamps.system = qtimer + SystemToQTimerOffset::instance().offset();
        } else {
            timestamps.device = qtimer + SteadyToRawOffset::instance().offset();
        }
        const auto offset = Timesync::getDefaultSynchronizedOffset();
        timestamps.host = timestamps.device + seconds(offset.tv_sec) + nanoseconds(offset.tv_nsec);
    }

    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setType(args.format == "GRAYF16" ? dai::ImgFrame::Type::GRAYF16
                   : args.format == "RAW16"   ? dai::ImgFrame::Type::RAW16
                                              : dai::ImgFrame::Type::YUV422i);
    frame->setWidth(args.width);
    frame->setHeight(args.height);
    frame->setSourceSize(args.width, args.height);
    frame->setInstanceNum(static_cast<unsigned int>(socket));
    frame->cam = args.cam;
    frame->fb.stride = args.wstride;
    frame->setSequenceNum(args.sequenceNumber);
    frame->setTimestampDevice(time_point<steady_clock, steady_clock::duration>{duration_cast<steady_clock::duration>(timestamps.device)});
    frame->setTimestamp(time_point<steady_clock, steady_clock::duration>{duration_cast<steady_clock::duration>(timestamps.host)});
    if(timestamps.system) {
        frame->setTimestampSystem(time_point<system_clock, system_clock::duration>{duration_cast<system_clock::duration>(*timestamps.system)});
    }
    frame->transformation = transformation;
    frame->data = args.data;

    auto& out = args.output == "temperature" ? temperature : args.output == "raw" ? raw : color;
    if(!out.trySend(frame)) logger->trace("Thermal: {} output full, frame {} dropped", args.output, args.sequenceNumber);
}

void Thermal::run() {
    const auto serialNumber = getSerialNumber();
    if(!serialNumber.empty() && serialNumber != "xlink-server") deviceId = serialNumber;
    if(cameraSensor) cameraSensor->run();
    while(isRunning()) {
        bool timedOut = false;
        auto config = inputConfig.get<ThermalConfig>(std::chrono::milliseconds(200), timedOut);
        if(!timedOut && config && cameraSensor) seek_eagle::configure(socket, *config);
    }
}

void Thermal::stop() {
    ThreadedNode::stop();
    if(cameraSensor) cameraSensor->stop();
}

}  // namespace gate
}  // namespace dai
