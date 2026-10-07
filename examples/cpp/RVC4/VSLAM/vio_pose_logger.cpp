#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/utility/Clock.hpp"

// Launch on the PC. Only poses are transferred from the RVC4 firmware.
int main(int argc, char** argv) {
    if(argc > 2 || (argc == 2 && std::string(argv[1]) == "--help")) {
        std::cout << "Usage: vio_pose_logger [poses.csv] (output is overwritten)\n";
        return argc > 2 ? 1 : 0;
    }
    try {
        dai::Pipeline pipeline;
        if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) {
            throw std::runtime_error("This example requires RVC4 with the updated VIO firmware.");
        }
        std::cout << "OAK OS: " << pipeline.getDefaultDevice()->getOSVersion() << '\n';
        const auto calibration = pipeline.getCalibrationData();
        try {
            for(const auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
                calibration.getCameraToImuExtrinsics(socket, true, dai::LengthUnit::METER);
            }
        } catch(const std::runtime_error& error) {
            throw std::runtime_error(std::string("VIO calibration check failed before starting cameras. Rebuild/select the RVC4 firmware with the IMU defaults "
                                                 "fix, or supply valid camera-to-IMU calibration for this board. Details: ")
                                     + error.what());
        }
        const auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 60);
        const auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 60);
        const auto imu = pipeline.create<dai::node::IMU>();
        const auto sync = pipeline.create<dai::node::Sync>();
        sync->setRunOnHost(false);
        sync->setTimestampSource(dai::node::Sync::TimestampSource::DEVICE);
        sync->setSyncThreshold(std::chrono::milliseconds(5));
        const auto vio = pipeline.create<dai::node::VIO>();
        vio->setImuUpdateRate(200);
        imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
        imu->setBatchReportThreshold(1);
        imu->setMaxBatchReports(10);
        left->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, std::nullopt, false)->link(sync->inputs["left"]);
        right->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, std::nullopt, false)->link(sync->inputs["right"]);
        sync->out.link(vio->stereo);
        imu->out.link(vio->imu);
        const auto poses = vio->transform.createOutputQueue(8, false);

        const std::string path = argc == 2 ? argv[1] : "poses.csv";
        std::ofstream output;
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output.open(path);
        output.imbue(std::locale::classic());
        output << std::fixed << std::setprecision(9);
        output << "sequence,sample_time_s,device_time_s,system_time_s,age_s,x_m,y_m,z_m,qx,qy,qz,qw\n" << std::flush;
        pipeline.start();
        std::cout << "VIO runs on the RVC4 CPU; logging poses to " << path << ". Ctrl+C to stop.\n";
        std::cout << "Waiting for stereo and IMU startup; the first pose can take several seconds.\n";
        auto lastReceive = dai::Clock::now();
        auto lastDisplay = lastReceive;
        auto previousSample = std::chrono::steady_clock::time_point{};
        while(pipeline.isRunning()) {
            const auto pose = poses->tryGet<dai::TransformData>();
            const auto now = dai::Clock::now();
            if(!pose) {
                const auto timeout = previousSample == std::chrono::steady_clock::time_point{} ? std::chrono::seconds(30) : std::chrono::seconds(2);
                if(now - lastReceive > timeout) throw std::runtime_error("No VIO poses received; check firmware logs, calibration, and IMU.");
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            const auto sampleTime = pose->getTimestamp();
            const auto deviceTime = pose->getTimestampDevice();
            const auto age = std::chrono::duration<double>(now - sampleTime).count();
            if(deviceTime <= previousSample || age < -0.05 || age > 2.0) throw std::runtime_error("Stale or invalid VIO timestamp.");
            const auto translation = pose->getTranslation();
            const auto rotation = pose->getQuaternion();
            const std::array<double, 7> values{translation.x, translation.y, translation.z, rotation.qx, rotation.qy, rotation.qz, rotation.qw};
            if(!std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) throw std::runtime_error("Non-finite VIO pose.");
            output << pose->getSequenceNum() << ',' << std::chrono::duration<double>(sampleTime.time_since_epoch()).count() << ','
                   << std::chrono::duration<double>(deviceTime.time_since_epoch()).count() << ',';
            if(const auto systemTime = pose->getTimestampSystem()) output << std::chrono::duration<double>(systemTime->time_since_epoch()).count();
            output << ',' << age;
            for(const auto value : values) output << ',' << value;
            output << '\n' << std::flush;
            if(now - lastDisplay >= std::chrono::milliseconds(200)) {
                std::cout << "xyz [m]: " << translation.x << ' ' << translation.y << ' ' << translation.z << "  q [xyzw]: " << rotation.qx << ' ' << rotation.qy
                          << ' ' << rotation.qz << ' ' << rotation.qw << "  age: " << age * 1000 << " ms\n";
                lastDisplay = now;
            }
            previousSample = deviceTime;
            lastReceive = now;
        }
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
