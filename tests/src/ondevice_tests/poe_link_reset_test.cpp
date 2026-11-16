// Regression test: the TCP connection to a PoE device (RVC2 PoE, or RVC4 over TCP/IP) is
// reset, and the host must close its XLink socket. Before the XLink fix, shutdown() on the
// reset socket failed with ENOTCONN, the host skipped close(), and the process kept one dead
// socket (an open fd without a connection) for each lost link. An application that
// reconnects for a long time then runs out of file descriptors.
//
// The test finds the host socket of the XLink connection and calls connect(AF_UNSPEC) on it.
// Linux then sends RST to the device and puts the host socket in the same state as after an
// RST from the device. After the device is closed, the number of TCP sockets of this process
// that are neither connected nor listening must be the same as before the reset.
// A local proxy cannot do this for RVC2 PoE: after the firmware boot, the host also searches
// for the device with a broadcast and can connect to its real address.
// Needs an RVC2 PoE or RVC4 device over TCP/IP and a Linux host; other devices are skipped.

#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "utility/Environment.hpp"

namespace {

using namespace std::chrono_literals;

constexpr uint16_t XLINK_PORT = 11490;
constexpr int LINK_RESETS = 2;
constexpr auto DEVICE_SEARCH_TIMEOUT = 30s;
// After a lost link, RVC2 reboots and RVC4 ends the firmware session: the device is busy for some time
constexpr auto DEVICE_READY_TIMEOUT = 120s;
// The host sees the reset on its next read or write; the watchdog closes a silent link after 2 x 4 s
constexpr auto LINK_LOSS_TIMEOUT = 30s;
// The XLink dispatcher can close the socket a moment after the device object is gone
constexpr auto SOCKET_CLOSE_TIMEOUT = 10s;

class ScopedEnvVar {
   public:
    ScopedEnvVar(std::string key, std::optional<std::string> value) : key(std::move(key)) {
        if(const char* existing = std::getenv(this->key.c_str())) {
            previous = std::string(existing);
        }

        if(value.has_value()) {
            dai::utility::setEnv(this->key, *value);
        } else {
            dai::utility::unsetEnv(this->key);
        }
    }

    ~ScopedEnvVar() {
        if(previous.has_value()) {
            dai::utility::setEnv(key, *previous);
        } else {
            dai::utility::unsetEnv(key);
        }
    }

   private:
    std::string key;
    std::optional<std::string> previous;
};

// Open fds of this process (the fd of the directory itself is not included)
std::vector<int> openFds() {
    DIR* dir = opendir("/dev/fd");
    if(dir == nullptr) {
        throw std::runtime_error("Cannot list /dev/fd");
    }
    const int dirFd = dirfd(dir);
    std::vector<int> fds;
    while(const dirent* entry = readdir(dir)) {
        char* end = nullptr;
        const long value = std::strtol(entry->d_name, &end, 10);
        if(end == entry->d_name || *end != '\0' || value == dirFd) continue;
        fds.push_back(static_cast<int>(value));
    }
    closedir(dir);
    return fds;
}

bool isTcpStream(int fd) {
    int type = 0;
    socklen_t typeLen = sizeof(type);
    if(getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &typeLen) != 0 || type != SOCK_STREAM) return false;
    sockaddr_storage local{};
    socklen_t localLen = sizeof(local);
    return getsockname(fd, reinterpret_cast<sockaddr*>(&local), &localLen) == 0 && (local.ss_family == AF_INET || local.ss_family == AF_INET6);
}

// Number of TCP sockets in this process that are neither connected nor listening.
// A socket that the peer reset stays like this until the process calls close() on it.
int countDeadTcpSockets() {
    int count = 0;
    for(int fd : openFds()) {
        if(!isTcpStream(fd)) continue;
        int listening = 0;
        socklen_t listeningLen = sizeof(listening);
        if(getsockopt(fd, SOL_SOCKET, SO_ACCEPTCONN, &listening, &listeningLen) == 0 && listening != 0) continue;
        sockaddr_storage peer{};
        socklen_t peerLen = sizeof(peer);
        if(getpeername(fd, reinterpret_cast<sockaddr*>(&peer), &peerLen) != 0) ++count;
    }
    return count;
}

// The lowest count in one second, so that a socket that is connecting at that moment is not counted
int settledDeadTcpSockets() {
    int lowest = countDeadTcpSockets();
    for(int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(100ms);
        lowest = std::min(lowest, countDeadTcpSockets());
    }
    return lowest;
}

int waitForDeadTcpSocketsAtMost(int expected, std::chrono::milliseconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    int count = countDeadTcpSockets();
    while(count > expected && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(100ms);
        count = countDeadTcpSockets();
    }
    return count;
}

// Host sockets that are connected to the XLink port of the device
std::vector<int> xlinkSockets(const std::string& deviceIp) {
    in_addr device{};
    if(inet_pton(AF_INET, deviceIp.c_str(), &device) != 1) {
        throw std::invalid_argument("Invalid IPv4 address: " + deviceIp);
    }
    std::vector<int> sockets;
    for(int fd : openFds()) {
        if(!isTcpStream(fd)) continue;
        sockaddr_in peer{};
        socklen_t peerLen = sizeof(peer);
        if(getpeername(fd, reinterpret_cast<sockaddr*>(&peer), &peerLen) != 0 || peer.sin_family != AF_INET) continue;
        if(peer.sin_addr.s_addr == device.s_addr && ntohs(peer.sin_port) == XLINK_PORT) sockets.push_back(fd);
    }
    return sockets;
}

// Sends RST to the device. The host socket stays open, in the state after an RST from the peer.
bool resetConnection(int fd) {
    sockaddr unspec{};
    unspec.sa_family = AF_UNSPEC;
    return connect(fd, &unspec, sizeof(unspec)) == 0;
}

std::optional<dai::DeviceInfo> findTcpDevice() {
    auto deadline = std::chrono::steady_clock::now() + DEVICE_SEARCH_TIMEOUT;
    do {
        for(const auto& info : dai::Device::getAllAvailableDevices()) {
            if(info.protocol == X_LINK_TCP_IP && (info.platform == X_LINK_MYRIAD_X || info.platform == X_LINK_RVC4)) {
                return info;
            }
        }
        std::this_thread::sleep_for(1s);
    } while(std::chrono::steady_clock::now() < deadline);
    return std::nullopt;
}

// After the previous reset, the device can still reboot (RVC2) or end its session (RVC4)
std::shared_ptr<dai::Device> openDevice(const dai::DeviceInfo& info) {
    std::string lastError;
    auto deadline = std::chrono::steady_clock::now() + DEVICE_READY_TIMEOUT;
    while(std::chrono::steady_clock::now() < deadline) {
        try {
            return std::make_shared<dai::Device>(info);
        } catch(const std::exception& e) {
            lastError = e.what();
        }
        std::this_thread::sleep_for(2s);
    }
    throw std::runtime_error("Cannot open the device " + info.name + ": " + lastError);
}

bool waitUntilClosed(const dai::Device& device, std::chrono::milliseconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while(!device.isClosed()) {
        if(std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(100ms);
    }
    return true;
}

// Crash dump collection is not part of this test; do not save or upload one
class NoCrashdumpFixture {
    ScopedEnvVar noCrashdump{"DEPTHAI_CRASHDUMP", "0"};
    ScopedEnvVar noUpload{"DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1"};
};

}  // namespace

TEST_CASE_METHOD(NoCrashdumpFixture, "PoE: the host closes its XLink socket when the connection is reset") {
    const auto board = findTcpDevice();
    if(!board) {
        SKIP("Needs an RVC2 PoE or RVC4 device over TCP/IP");
    }

    for(int i = 1; i <= LINK_RESETS; ++i) {
        INFO("Link reset " << i << " of " << LINK_RESETS << ", device " << board->name);
        auto device = openDevice(*board);
        // Without this, the device connects again after the reset
        device->setMaxReconnectionAttempts(0);
        REQUIRE_FALSE(device->isClosed());

        const auto sockets = xlinkSockets(board->name);
        REQUIRE(sockets.size() == 1);
        const int deadBefore = settledDeadTcpSockets();
        REQUIRE(resetConnection(sockets.front()));
        REQUIRE(waitUntilClosed(*device, LINK_LOSS_TIMEOUT));
        device.reset();

        const int deadAfter = waitForDeadTcpSocketsAtMost(deadBefore, SOCKET_CLOSE_TIMEOUT);
        INFO("TCP sockets without a connection: " << deadBefore << " before the reset, " << deadAfter << " after the device is closed");
        CHECK(deadAfter <= deadBefore);
    }
}
