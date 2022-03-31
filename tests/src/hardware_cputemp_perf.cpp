#define CATCH_CONFIG_MAIN
#include <atomic>
#include <catch2/catch.hpp>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <tuple>

#include "depthai/depthai.hpp"

using namespace std::string_view_literals;

constexpr auto COLOR_OFF = static_cast<dai::ColorCameraProperties::SensorResolution>(-99999);
constexpr auto DEPTH_OFF = static_cast<dai::MonoCameraProperties::SensorResolution>(-99999);

// implement APIs for device search by protocol
std::tuple<bool, dai::DeviceInfo> getFirstDevice(XLinkDeviceState_t state, const XLinkProtocol_t protocol) {
    auto devices = dai::XLinkConnection::getAllConnectedDevices(state, true);
    for(const auto& d : devices) {
        if ((d.state == state || state == X_LINK_ANY_STATE) && (d.desc.protocol == protocol)) return {true, d};
    }
    return {false, dai::DeviceInfo()};
}

/*
std::tuple<bool, dai::DeviceInfo> getFirstAvailableDevice(const XLinkProtocol_t protocol) {
    bool found;
    dai::DeviceInfo dev;
    std::tie(found, dev) = getFirstDevice(X_LINK_UNBOOTED, protocol);
    if(!found) {
        std::tie(found, dev) = getFirstDevice(X_LINK_BOOTLOADER, protocol);
    }
    if(!found) {
        std::tie(found, dev) = getFirstDevice(X_LINK_FLASH_BOOTED, protocol);
    }
    return {found, dev};
}
*/

template <typename Rep, typename Period>
static std::tuple<bool, dai::DeviceInfo> getAvailableDevice(const std::chrono::duration<Rep, Period> timeout, const XLinkProtocol_t protocol) {
    using namespace std::chrono;
    constexpr auto POLL_SLEEP_TIME = milliseconds(100);

    // First looks for UNBOOTED, then BOOTLOADER, for 'timeout' time
    auto searchStartTime = steady_clock::now();
    bool found = false;
    dai::DeviceInfo deviceInfo;
    do {
        for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_FLASH_BOOTED}) {
            std::tie(found, deviceInfo) = getFirstDevice(searchState, protocol);
            if(found) break;
        }
        if(found) break;

        // If 'timeout' < 'POLL_SLEEP_TIME', use 'timeout' as sleep time and then break
        if(timeout < POLL_SLEEP_TIME) {
            // sleep for 'timeout'
            std::this_thread::sleep_for(timeout);
            break;
        } else {
            std::this_thread::sleep_for(POLL_SLEEP_TIME);  // default pool rate
        }
    } while(steady_clock::now() - searchStartTime < timeout);

    // If none were found, try BOOTED
    // if(!found) std::tie(found, deviceInfo) = XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    return {found, deviceInfo};
}

class PerfTestFixture {
   protected:
    inline static const std::string log_header{"epoch,color,depth,mono,imu,subpixel,align,lrcheck,filter_median,fps,protocol\n"};
    inline static const std::filesystem::path log_filename{"hw-cputemp.csv"};

    std::ofstream& getOutput() {
        static std::ofstream output = []() {
            const auto out_path = std::filesystem::current_path() / log_filename;
            if (std::filesystem::exists(out_path)) {
                // file epoch is not guaranteed to be the same as system_clock epoch
                // ok here as this code's intention is only to create a backup name related to the existing file
                const auto file_time = std::filesystem::last_write_time(out_path);
                const auto epoch_s = std::chrono::duration_cast<std::chrono::seconds>(file_time.time_since_epoch()).count();
                auto backup_path = out_path;
                backup_path.replace_filename(out_path.stem().u8string() + "-" + std::to_string(epoch_s) + ".csv");
                std::filesystem::rename(out_path, backup_path);
                std::cout << "samples backup: " << backup_path.u8string() << std::endl;
            }
            std::cout << "samples: " << out_path.u8string() << std::endl;
            std::ofstream output;
            output.exceptions(std::ifstream::failbit | std::ifstream::badbit);
            output.open(out_path, std::ios::out | std::ios::trunc);

            // write header to match in sequence the generators and data
            output << log_header;
            output << std::boolalpha << std::fixed << std::setprecision(1);
            return std::move(output);
        }();
        return output;
    }
};

TEST_CASE_METHOD(PerfTestFixture, "Hardware CPU and Temp") {

    auto color_res = GENERATE(
        std::tuple<dai::ColorCameraProperties::SensorResolution, int, int>(COLOR_OFF, 0, 0),
        std::tuple<dai::ColorCameraProperties::SensorResolution, int, int>(dai::ColorCameraProperties::SensorResolution::THE_1080_P, 832, 480),
        std::tuple<dai::ColorCameraProperties::SensorResolution, int, int>(dai::ColorCameraProperties::SensorResolution::THE_1080_P, 1280, 720),
        std::tuple<dai::ColorCameraProperties::SensorResolution, int, int>(dai::ColorCameraProperties::SensorResolution::THE_1080_P, 1920, 1080),
        std::tuple<dai::ColorCameraProperties::SensorResolution, int, int>(dai::ColorCameraProperties::SensorResolution::THE_4_K, 2560, 1440),
        std::tuple<dai::ColorCameraProperties::SensorResolution, int, int>(dai::ColorCameraProperties::SensorResolution::THE_4_K, 3840, 2160)
    );
    auto depth_res = GENERATE(
        std::tuple<dai::MonoCameraProperties::SensorResolution, int, int>(DEPTH_OFF, 0, 0),
        std::tuple<dai::MonoCameraProperties::SensorResolution, int, int>(dai::MonoCameraProperties::SensorResolution::THE_400_P, 640, 400),
        std::tuple<dai::MonoCameraProperties::SensorResolution, int, int>(dai::MonoCameraProperties::SensorResolution::THE_480_P, 640, 480),
        std::tuple<dai::MonoCameraProperties::SensorResolution, int, int>(dai::MonoCameraProperties::SensorResolution::THE_720_P, 1280, 720),
        std::tuple<dai::MonoCameraProperties::SensorResolution, int, int>(dai::MonoCameraProperties::SensorResolution::THE_800_P, 1280, 800)
    );
    auto mono_output = GENERATE(false, true);
    auto imu_output = GENERATE(false, true);
    // auto nn_pose_output = GENERATE(false);
    auto depth_subpixel = GENERATE(false, true);
    auto depth_align = GENERATE(false, true);
    auto depth_lrcheck = GENERATE(false, true);
    auto depth_filter_median = GENERATE(dai::MedianFilter::KERNEL_5x5);
    auto fps = GENERATE(30.0f);  // (5, 10, 15, 20, 25, 30, 60);
    auto protocol = GENERATE(X_LINK_TCP_IP); // X_LINK_USB_VSC, X_LINK_TCP_IP

    // permutations to skip
    if ((std::get<0>(depth_res) == DEPTH_OFF) && (std::get<0>(color_res) == COLOR_OFF)) {
        SUCCEED("skipping: color or depth output must be enabled for these tests");
        return;
    }
    if (mono_output && (std::get<0>(depth_res) == DEPTH_OFF)) {
        SUCCEED("skipping: mono output requires depth output for these tests");
        return;
    }
    if (depth_align && (std::get<0>(color_res) == COLOR_OFF)) {
        // workaround https://github.com/luxonis/depthai-core/issues/414
        SUCCEED("skipping: depth aligned to color requires color output for these tests");
        return;
    }

    // output current generator permutation
    auto outputPermutation = [&]() -> std::string {
        return  std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()) + "," +
                std::to_string(std::get<2>(color_res)) + "," +
                std::to_string(std::get<2>(depth_res)) + "," +
                std::to_string(mono_output ? std::get<2>(depth_res) : 0) + "," +
                (imu_output ? "true," : "false,") +
                (depth_subpixel ? "true," : "false,") +
                (depth_align ? "true," : "false,") +
                (depth_lrcheck ? "true," : "false,") +
                std::to_string(static_cast<int32_t>(depth_filter_median)) + "," +
                std::to_string(fps) + "," +
                (protocol == X_LINK_USB_VSC ? "usb" : "tcp") + ",";
    };

    // Create pipeline
    INFO(log_header << outputPermutation());
    if (protocol == X_LINK_TCP_IP) {
        // write to console to indicate progress
        std::cout << "device conf: " << outputPermutation() << std::endl;
    }
    dai::Pipeline p;

    // System information
    {
        auto sysInfo = p.create<dai::node::SystemLogger>();
        auto xo_sysInfo = p.create<dai::node::XLinkOut>();
        xo_sysInfo->setStreamName("sysinfo");
        sysInfo->out.link(xo_sysInfo->input);

        sysInfo->setRate(0.333f);
    }

    // color output
    if (std::get<0>(color_res) != COLOR_OFF) {
        auto colorCamera = p.create<dai::node::ColorCamera>();
        auto xo_colorCamera = p.create<dai::node::XLinkOut>();
        xo_colorCamera->setStreamName("color");
        colorCamera->video.link(xo_colorCamera->input);

        colorCamera->setFps(fps);
        colorCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
        colorCamera->initialControl.setAutoExposureLock(false);
        colorCamera->initialControl.setManualExposure(15000, 1600);

        colorCamera->setResolution(std::get<0>(color_res));
        const auto sensor_width = colorCamera->getResolutionWidth();
        switch (std::get<2>(color_res)) {
            case 480:
                colorCamera->setIspScale(4, 9);
                break;
            default:
                colorCamera->setIspScale(std::get<1>(color_res), sensor_width);
        }
        colorCamera->setVideoSize(std::get<1>(color_res), std::get<2>(color_res));
        colorCamera->sensorCenterCrop();

        // ignore color autofocus and how it affects depth->color aligning
    }

    if (std::get<0>(depth_res) != DEPTH_OFF) {
        auto monoLeft = p.create<dai::node::MonoCamera>();
        auto monoRight = p.create<dai::node::MonoCamera>();
        auto stereoDepth = p.create<dai::node::StereoDepth>();
        auto xo_stereoDepth = p.create<dai::node::XLinkOut>();
        xo_stereoDepth->setStreamName("depth");
        monoLeft->out.link(stereoDepth->left);
        monoRight->out.link(stereoDepth->right);
        stereoDepth->depth.link(xo_stereoDepth->input);
        if (mono_output) {
            auto xo_mono = p.create<dai::node::XLinkOut>();
            xo_mono->setStreamName("mono");
            stereoDepth->rectifiedRight.link(xo_mono->input);
        }

        monoLeft->setFps(fps);
        monoRight->setFps(fps);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        monoLeft->initialControl.setAutoExposureEnable();
        monoRight->initialControl.setAutoExposureEnable();
        // monoLeft->initialControl.setAutoExposureLock(false);
        // monoLeft->initialControl.setManualExposure(15000, 1600);
        // monoRight->initialControl.setAutoExposureLock(false);
        // monoRight->initialControl.setManualExposure(15000, 1600);

        stereoDepth->setSubpixel(depth_subpixel);
        stereoDepth->setExtendedDisparity(false);
        stereoDepth->setRectifyEdgeFillColor(0);
        if (depth_align) {
            // workaround for https://github.com/luxonis/depthai-core/issues/360
            stereoDepth->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::CENTER);
            stereoDepth->setDepthAlign(dai::CameraBoardSocket::RGB);
            stereoDepth->setLeftRightCheck(true); // forced true as required by align
            stereoDepth->setOutputSize(std::get<1>(color_res), std::get<2>(color_res));
            stereoDepth->setOutputKeepAspectRatio(true);
        }
        else {
            stereoDepth->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_RIGHT);
            stereoDepth->setLeftRightCheck(depth_lrcheck);
        }

        stereoDepth->initialConfig.setMedianFilter(depth_filter_median);
    }

    if (imu_output) {
        // TODO check for hw imu support
        auto imu = p.create<dai::node::IMU>();
        auto xo_imu = p.create<dai::node::XLinkOut>();
        xo_imu->setStreamName("imu");
        imu->out.link(xo_imu->input);

        imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER, 60);
        imu->setBatchReportThreshold(1);
        imu->setMaxBatchReports(10);
    }

    /*
    if (nn_pose_output) {
        // TODO
    }
    */

    std::unique_ptr<dai::Device> device;
    REQUIRE_NOTHROW([&]() {
        //INFO(outputPermutation());
        // TODO may improve fps and reduce lag; waiting on Luxonis feedback on pro/cons
        // p.setXLinkChunkSize(0);

        const auto targetDevice = getAvailableDevice(std::chrono::seconds(protocol == X_LINK_TCP_IP ? 60 : 15), protocol);
        REQUIRE(std::get<bool>(targetDevice));
        device = std::make_unique<dai::Device>(p, std::get<dai::DeviceInfo>(targetDevice));
    }());

    std::vector<std::shared_ptr<dai::DataOutputQueue>> queues;
    if (std::get<0>(color_res) != COLOR_OFF) {
        queues.emplace_back(device->getOutputQueue("color", 0, false));
        queues.back()->addCallback([](std::shared_ptr<dai::ADatatype> data) {
            auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
            (void)frame;
        });
    }
    if (std::get<0>(depth_res) != DEPTH_OFF) {
        queues.emplace_back(device->getOutputQueue("depth", 0, false));
        queues.back()->addCallback([](std::shared_ptr<dai::ADatatype> data) {
            auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
            (void)frame;
        });
        if (mono_output) {
            queues.emplace_back(device->getOutputQueue("mono", 0, false));
            queues.back()->addCallback([](std::shared_ptr<dai::ADatatype> data) {
                auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
                (void)frame;
            });
        }
    }
    if (imu_output) {
        queues.emplace_back(device->getOutputQueue("imu", 0, false));
        queues.back()->addCallback([](std::shared_ptr<dai::ADatatype> data) {
            auto imudata = std::dynamic_pointer_cast<dai::IMUData>(data);
            (void)imudata;
        });
    }
    /*
    if (nn_pose_output) {
        // TODO
    }
    */

    // collect and check system information
    std::atomic_flag first_skipped = ATOMIC_FLAG_INIT;
    queues.emplace_back(device->getOutputQueue("sysinfo", 0, false));
    queues.back()->addCallback([&](std::shared_ptr<dai::ADatatype> data) {
        const auto sysInfo = std::dynamic_pointer_cast<dai::SystemInformation>(data);
        if (first_skipped.test_and_set()) {
            getOutput() << outputPermutation();
            getOutput() << (sysInfo->leonCssCpuUsage.average * 100.0f) << ",";
            getOutput() << (sysInfo->leonMssCpuUsage.average * 100.0f) << ",";
            getOutput() << sysInfo->chipTemperature.average << std::endl;
        }
    });

    // keep connection alive to collect device data
    std::this_thread::sleep_for(std::chrono::seconds(10));
}
