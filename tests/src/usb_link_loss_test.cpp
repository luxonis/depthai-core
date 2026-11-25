#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"

#ifdef __linux__
    #include <fcntl.h>
    #include <linux/usbdevice_fs.h>
    #include <sys/ioctl.h>
    #include <unistd.h>

    #include <filesystem>
#endif

using namespace std::chrono;
using namespace std::chrono_literals;

namespace {

#ifdef __linux__
std::string readSysfs(const std::filesystem::path& p) {
    std::ifstream f(p);
    std::string s;
    std::getline(f, s);
    return s;
}

// Reset the USB port of the booted device whose USB serial is 'mxid' (USBDEVFS_RESET).
// The device re-enumerates and the firmware watchdog reboots it: the same link loss a bad
// cable, port or hub produces.
bool resetUsbDevice(const std::string& mxid) {
    for(const auto& entry : std::filesystem::directory_iterator("/sys/bus/usb/devices")) {
        const auto dir = entry.path();
        if(!std::filesystem::exists(dir / "idVendor") || readSysfs(dir / "serial") != mxid) continue;
        char node[64];
        std::snprintf(node, sizeof(node), "/dev/bus/usb/%03d/%03d", std::stoi(readSysfs(dir / "busnum")), std::stoi(readSysfs(dir / "devnum")));
        const int fd = open(node, O_WRONLY);
        if(fd < 0) {
            WARN("Cannot open " << node << " for the USB reset (udev rules?)");
            return false;
        }
        const int rc = ioctl(fd, USBDEVFS_RESET, 0);
        close(fd);
        return rc == 0;
    }
    WARN("No USB device with serial " << mxid << " found in sysfs");
    return false;
}
#endif

}  // namespace

#ifdef __linux__
TEST_CASE("A blocked queue read returns with an error when the USB link is lost", "[usb][linkloss]") {
    // Nothing is ever sent into 'in', so the read on 'out' blocks until the link is lost
    dai::Pipeline pipeline;
    auto xin = pipeline.create<dai::node::XLinkIn>();
    xin->setStreamName("in");
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("out");
    xin->out.link(xout->input);

    dai::Device device(pipeline);
    REQUIRE(device.getDeviceInfo().protocol == X_LINK_USB_VSC);
    const auto mxid = device.getMxId();
    auto queue = device.getOutputQueue("out", 4, true);

    std::atomic<bool> returned{false};
    std::atomic<bool> threw{false};
    std::thread reader([&]() {
        try {
            queue->get();
        } catch(const std::exception&) {
            threw = true;
        }
        returned = true;
    });

    std::this_thread::sleep_for(1s);
    REQUIRE(resetUsbDevice(mxid));

    // The link is gone: the read must not outlive it
    const auto t0 = steady_clock::now();
    while(!returned && steady_clock::now() - t0 < 30s) {
        std::this_thread::sleep_for(100ms);
    }
    if(!returned) {
        // Leave the blocked thread behind instead of hanging the whole test binary
        reader.detach();
        FAIL("queue->get() is still blocked 30 s after the USB link was lost");
    }
    reader.join();
    REQUIRE(threw);

    // Closing after the link loss completes and the device comes back for the next user
    REQUIRE_NOTHROW(device.close());
    REQUIRE(device.isClosed());
    bool found = false;
    const auto t1 = steady_clock::now();
    while(!found && steady_clock::now() - t1 < 30s) {
        std::tie(found, std::ignore) = dai::Device::getDeviceByMxId(mxid);
        if(!found) std::this_thread::sleep_for(500ms);
    }
    REQUIRE(found);
}
#endif
