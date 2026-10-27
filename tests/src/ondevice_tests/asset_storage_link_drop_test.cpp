// Regression tests: the link to the device fails while DeviceBase::startPipelineImpl()
// sends the asset storage. Before the fix, XLinkWriteError left the writer std::thread
// and called std::terminate (the process aborted). Now pipeline.start() must throw
// std::runtime_error that says the connection was lost during the transfer, in a
// bounded time, and the device must be usable again after it.
//
// A local proxy on 127.0.0.1 forwards the RVC4 ports (gate REST, XLink, discovery) to
// the device. On the XLink connection, it applies one fault after the asset storage
// stream starts: reset, close, blackhole, one-way loss, stall, fragmentation or a
// device that vanishes. Other cases kill the firmware through the gate, or close the
// device from another thread.
// Needs an RVC4 device over TCP/IP; other devices are skipped.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <catch2/catch_all.hpp>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "utility/Environment.hpp"

// Linux sets MSG_NOSIGNAL on each send(); macOS has no MSG_NOSIGNAL and sets SO_NOSIGPIPE on the socket (noSigPipe)
#ifndef MSG_NOSIGNAL
    #define MSG_NOSIGNAL 0
#endif

namespace {

using namespace std::chrono_literals;
using Catch::Matchers::ContainsSubstring;

constexpr uint16_t XLINK_PORT = 11490;
constexpr uint16_t DISCOVERY_PORT = 11491;
constexpr uint16_t GATE_PORT = 11492;
constexpr std::string_view ASSET_STORAGE_STREAM = "__stream_asset_storage";
constexpr std::string_view LINK_LOST_MESSAGE = "was lost during the asset storage transfer";
constexpr std::size_t MiB = 1024 * 1024;
constexpr std::size_t ASSET_SIZE = 64 * MiB;
constexpr auto UPSTREAM_CONNECT_TIMEOUT = 30s;
constexpr auto DEVICE_SEARCH_TIMEOUT = 30s;
constexpr auto DEVICE_IDLE_TIMEOUT = 90s;
constexpr auto FAULT_TIMEOUT = 60s;
// The host watchdog closes a silent link after 2 x 4 s, so each failure must end well before this
constexpr auto START_TIMEOUT = 45s;
// After START_TIMEOUT the proxy resets the held connections; if start() still does not return, the process exits
constexpr auto RESCUE_TIMEOUT = 30s;

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

sockaddr_in makeAddress(const std::string& ip, uint16_t port) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if(inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        throw std::invalid_argument("Invalid IPv4 address: " + ip);
    }
    return addr;
}

// A send() to a socket that the peer reset must not raise SIGPIPE and end the test process
void noSigPipe(int fd) {
#ifdef SO_NOSIGPIPE
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#else
    (void)fd;
#endif
}

int bindLocal(int type, uint16_t port) {
    int fd = socket(AF_INET, type, 0);
    if(fd < 0) {
        throw std::runtime_error("socket() failed");
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    auto addr = makeAddress("127.0.0.1", port);
    if(bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || (type == SOCK_STREAM && listen(fd, 16) != 0)) {
        close(fd);
        throw std::runtime_error("Cannot bind 127.0.0.1:" + std::to_string(port));
    }
    return fd;
}

// Close with SO_LINGER {1, 0}, so that the peer gets RST like on a dropped link
void resetSocket(int fd) {
    linger lin{1, 0};
    setsockopt(fd, SOL_SOCKET, SO_LINGER, &lin, sizeof(lin));
    close(fd);
}

// Close with FIN, like a peer that ends the connection in order
void finSocket(int fd) {
    shutdown(fd, SHUT_RDWR);
    close(fd);
}

bool sendAll(int fd, const char* data, std::size_t size) {
    while(size > 0) {
        auto sent = send(fd, data, size, MSG_NOSIGNAL);
        if(sent < 0 && errno == EINTR) continue;
        if(sent <= 0) return false;
        data += sent;
        size -= static_cast<std::size_t>(sent);
    }
    return true;
}

// Minimal HTTP/1.1 client for the gate REST API (Connection: close, body after the headers)
std::string gateRequest(const std::string& ip, const std::string& method, const std::string& path) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0) {
        throw std::runtime_error("socket() failed");
    }
    noSigPipe(fd);
    timeval timeout{5, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    auto addr = makeAddress(ip, GATE_PORT);
    if(connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close(fd);
        throw std::runtime_error("Cannot connect to the gate at " + ip);
    }
    std::string request = method + " " + path + " HTTP/1.1\r\nHost: " + ip + "\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    std::string response;
    if(sendAll(fd, request.data(), request.size())) {
        char buffer[4096];
        ssize_t len = 0;
        while((len = recv(fd, buffer, sizeof(buffer), 0)) > 0) {
            response.append(buffer, static_cast<std::size_t>(len));
        }
    }
    close(fd);
    // Status line: "HTTP/1.1 200 OK"
    auto statusStart = response.find(' ');
    if(statusStart == std::string::npos || response.compare(statusStart + 1, 1, "2") != 0) {
        throw std::runtime_error("Gate request " + method + " " + path + " failed: " + response.substr(0, response.find("\r\n")));
    }
    auto bodyStart = response.find("\r\n\r\n");
    return bodyStart == std::string::npos ? std::string() : response.substr(bodyStart + 4);
}

struct GateSession {
    std::string id;
    std::string state;
};

std::vector<GateSession> gateSessions(const std::string& ip) {
    std::vector<GateSession> sessions;
    auto body = nlohmann::json::parse(gateRequest(ip, "GET", "/api/v1/sessions"));
    for(const auto& session : body.at("sessions")) {
        const auto& id = session.at("id");
        sessions.push_back({id.is_string() ? id.get<std::string>() : id.dump(), session.at("state").get<std::string>()});
    }
    return sessions;
}

// What the proxy does to the XLink connection after the trigger point
enum class Fault {
    None,        // only signal the trigger (the test does something else)
    Reset,       // RST to both sides
    Close,       // FIN to both sides
    Blackhole,   // forward nothing, keep both sockets open (cable pulled, no RST)
    MuteDevice,  // forward host->device, drop device->host (one-way link loss)
    Stall,       // forward nothing for some time, then go on (short network outage)
    Fragment,    // no trigger; forward everything in small random pieces
};

struct FaultPlan {
    Fault fault = Fault::Reset;
    // Host->device bytes after the asset storage stream name, before the fault
    std::size_t triggerAfter = 4 * MiB;
    std::chrono::milliseconds stall{0};
    // After the fault, refuse new connections and drop discovery (the device is gone)
    bool vanish = false;
};

// Forwards 127.0.0.1:{11490,11492}/tcp and 11491/udp to the device and applies the
// fault plan to the XLink connection that carries the asset storage.
class LinkFaultProxy {
   public:
    LinkFaultProxy(std::string deviceIp, FaultPlan plan) : deviceIp(std::move(deviceIp)), plan(plan) {
        try {
            gateFd = bindLocal(SOCK_STREAM, GATE_PORT);
            xlinkFd = bindLocal(SOCK_STREAM, XLINK_PORT);
            udpLocalFd = bindLocal(SOCK_DGRAM, DISCOVERY_PORT);
            udpRemoteFd = socket(AF_INET, SOCK_DGRAM, 0);
        } catch(...) {
            closeListeners();
            throw;
        }
        threads.emplace_back([this] { acceptLoop(gateFd, GATE_PORT, false); });
        threads.emplace_back([this] { acceptLoop(xlinkFd, XLINK_PORT, true); });
        threads.emplace_back([this] { udpLoop(); });
    }

    LinkFaultProxy(const LinkFaultProxy&) = delete;
    LinkFaultProxy& operator=(const LinkFaultProxy&) = delete;

    ~LinkFaultProxy() {
        stop = true;
        // Accept loops end in one poll period. Join them first, so that no new pump thread starts
        for(auto& t : threads) t.join();
        std::lock_guard<std::mutex> lock(pumpMtx);
        for(auto& t : pumps) t.join();
        closeListeners();
    }

    bool assetStreamSeen() const {
        return streamSeen;
    }

    bool faultApplied() const {
        return fired;
    }

    bool waitForFault(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(firedMtx);
        return firedCv.wait_for(lock, timeout, [this] { return fired.load(); });
    }

    // Ends a held fault (blackhole, mute): resets the connection
    void release() {
        released = true;
    }

   private:
    void closeListeners() {
        for(int fd : {gateFd, xlinkFd, udpLocalFd, udpRemoteFd}) {
            if(fd >= 0) close(fd);
        }
    }

    void fire() {
        {
            std::lock_guard<std::mutex> lock(firedMtx);
            fired = true;
        }
        firedCv.notify_all();
    }

    bool vanished() const {
        return plan.vanish && fired;
    }

    void acceptLoop(int listenFd, uint16_t port, bool watch) {
        pollfd pfd{listenFd, POLLIN, 0};
        while(!stop) {
            if(poll(&pfd, 1, 100) <= 0) continue;
            int client = accept(listenFd, nullptr, nullptr);
            if(client < 0) continue;
            noSigPipe(client);
            if(vanished()) {
                resetSocket(client);
                continue;
            }
            std::lock_guard<std::mutex> lock(pumpMtx);
            pumps.emplace_back([this, client, port, watch] { pump(client, port, watch); });
        }
    }

    // The host connects at once, but the firmware listens on the XLink port some seconds later
    int connectUpstream(uint16_t port) {
        auto addr = makeAddress(deviceIp, port);
        auto deadline = std::chrono::steady_clock::now() + UPSTREAM_CONNECT_TIMEOUT;
        while(!stop && std::chrono::steady_clock::now() < deadline) {
            int fd = socket(AF_INET, SOCK_STREAM, 0);
            if(fd >= 0 && connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
                noSigPipe(fd);
                return fd;
            }
            if(fd >= 0) close(fd);
            std::this_thread::sleep_for(200ms);
        }
        return -1;
    }

    void sleepUnlessStopped(std::chrono::milliseconds duration) {
        auto deadline = std::chrono::steady_clock::now() + duration;
        while(!stop && !released && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(20ms);
        }
    }

    bool forward(int to, const char* data, std::size_t size, std::mt19937& rng) {
        if(plan.fault != Fault::Fragment) {
            return sendAll(to, data, size);
        }
        std::uniform_int_distribution<std::size_t> pieceSize(1, 3000);
        while(size > 0) {
            auto piece = std::min(size, pieceSize(rng));
            if(!sendAll(to, data, piece)) return false;
            data += piece;
            size -= piece;
        }
        return true;
    }

    // Finds the asset storage stream name in the host->device bytes, also across two reads
    bool findStreamName(std::string& tail, const char* data, std::size_t size) {
        tail.append(data, size);
        if(tail.find(ASSET_STORAGE_STREAM) != std::string::npos) {
            streamSeen = true;
            return true;
        }
        if(tail.size() >= ASSET_STORAGE_STREAM.size()) {
            tail.erase(0, tail.size() - (ASSET_STORAGE_STREAM.size() - 1));
        }
        return false;
    }

    void pump(int client, uint16_t port, bool watch) {
        int upstream = connectUpstream(port);
        if(upstream < 0) {
            resetSocket(client);
            return;
        }
        int one = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        setsockopt(upstream, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        std::mt19937 rng(port);
        std::vector<char> buffer(1 << 20);
        std::string tail;
        std::size_t sent = 0;
        std::optional<std::size_t> armedAt;
        bool muted = false;
        pollfd fds[2] = {{client, POLLIN, 0}, {upstream, POLLIN, 0}};
        while(!stop && !(muted && released)) {
            if(poll(fds, 2, 100) <= 0) continue;
            for(int i = 0; i < 2; ++i) {
                if(fds[i].revents == 0) continue;
                auto len = recv(fds[i].fd, buffer.data(), buffer.size(), 0);
                if(len <= 0) {
                    close(client);
                    close(upstream);
                    return;
                }
                const bool fromHost = i == 0;
                const auto size = static_cast<std::size_t>(len);
                if(muted && !fromHost) continue;
                if(watch && fromHost && plan.fault == Fault::Fragment && !streamSeen) {
                    findStreamName(tail, buffer.data(), size);
                } else if(watch && fromHost && plan.fault != Fault::Fragment && !fired) {
                    sent += size;
                    if(!armedAt && findStreamName(tail, buffer.data(), size)) {
                        armedAt = sent;
                    }
                    if(armedAt && sent - *armedAt >= plan.triggerAfter) {
                        fire();
                        switch(plan.fault) {
                            case Fault::Reset:
                                resetSocket(client);
                                resetSocket(upstream);
                                return;
                            case Fault::Close:
                                finSocket(client);
                                finSocket(upstream);
                                return;
                            case Fault::Blackhole:
                                while(!stop && !released) std::this_thread::sleep_for(20ms);
                                resetSocket(client);
                                resetSocket(upstream);
                                return;
                            case Fault::MuteDevice:
                                muted = true;
                                break;
                            case Fault::Stall:
                                sleepUnlessStopped(plan.stall);
                                break;
                            case Fault::None:
                            case Fault::Fragment:
                                break;
                        }
                    }
                }
                if(!forward(fds[1 - i].fd, buffer.data(), size, rng)) {
                    close(client);
                    close(upstream);
                    return;
                }
            }
        }
        resetSocket(client);
        resetSocket(upstream);
    }

    void udpLoop() {
        auto device = makeAddress(deviceIp, DISCOVERY_PORT);
        sockaddr_in client{};
        bool haveClient = false;
        std::vector<char> buffer(65535);
        pollfd fds[2] = {{udpLocalFd, POLLIN, 0}, {udpRemoteFd, POLLIN, 0}};
        while(!stop) {
            if(poll(fds, 2, 100) <= 0) continue;
            if(fds[0].revents != 0) {
                socklen_t clientLen = sizeof(client);
                auto len = recvfrom(udpLocalFd, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr*>(&client), &clientLen);
                if(len > 0 && !vanished()) {
                    haveClient = true;
                    sendto(udpRemoteFd, buffer.data(), static_cast<std::size_t>(len), 0, reinterpret_cast<sockaddr*>(&device), sizeof(device));
                }
            }
            if(fds[1].revents != 0) {
                auto len = recv(udpRemoteFd, buffer.data(), buffer.size(), 0);
                if(len > 0 && haveClient && !vanished()) {
                    sendto(udpLocalFd, buffer.data(), static_cast<std::size_t>(len), 0, reinterpret_cast<sockaddr*>(&client), sizeof(client));
                }
            }
        }
    }

    std::string deviceIp;
    FaultPlan plan;
    int gateFd = -1;
    int xlinkFd = -1;
    int udpLocalFd = -1;
    int udpRemoteFd = -1;
    std::atomic<bool> stop{false};
    std::atomic<bool> released{false};
    std::atomic<bool> streamSeen{false};
    std::atomic<bool> fired{false};
    std::mutex firedMtx;
    std::condition_variable firedCv;
    std::vector<std::thread> threads;
    std::mutex pumpMtx;
    std::vector<std::thread> pumps;
};

const std::optional<dai::DeviceInfo>& rvc4OverTcp() {
    static const std::optional<dai::DeviceInfo> board = [] {
        auto deadline = std::chrono::steady_clock::now() + DEVICE_SEARCH_TIMEOUT;
        do {
            for(const auto& info : dai::Device::getAllAvailableDevices()) {
                if(info.platform == X_LINK_RVC4 && info.protocol == X_LINK_TCP_IP) {
                    return std::optional<dai::DeviceInfo>(info);
                }
            }
            std::this_thread::sleep_for(1s);
        } while(std::chrono::steady_clock::now() < deadline);
        return std::optional<dai::DeviceInfo>();
    }();
    return board;
}

// After a link failure, the gate keeps the session RUNNING until it notices the lost
// host. Wait until no session runs, so that the next case (or test) can use the device.
bool waitUntilDeviceIsIdle(const std::string& ip) {
    auto deadline = std::chrono::steady_clock::now() + DEVICE_IDLE_TIMEOUT;
    while(std::chrono::steady_clock::now() < deadline) {
        try {
            bool running = false;
            for(const auto& session : gateSessions(ip)) {
                running = running || session.state == "RUNNING";
            }
            if(!running) return true;
        } catch(const std::exception&) {
            // The gate can be busy for a moment; try again
        }
        std::this_thread::sleep_for(1s);
    }
    return false;
}

struct ScenarioContext {
    std::string boardIp;
    std::set<std::string> sessionsBefore;
    std::shared_ptr<dai::Device> device;
};

// Runs on the test thread when the proxy applies the fault (while start() runs)
using FaultAction = std::function<void(const ScenarioContext&)>;

struct Outcome {
    bool threw = false;
    bool runtimeError = false;
    std::string message;
    std::chrono::milliseconds duration{0};
    bool assetStreamSeen = false;
    bool faultApplied = false;
    std::string faultActionError;
};

Outcome runScenario(const FaultPlan& plan, std::size_t assetSize, const FaultAction& onFault = {}) {
    const auto& board = rvc4OverTcp();
    if(!board) {
        SKIP("Needs an RVC4 device over TCP/IP");
    }
    REQUIRE(waitUntilDeviceIsIdle(board->name));

    ScenarioContext context;
    context.boardIp = board->name;
    for(const auto& session : gateSessions(board->name)) {
        context.sessionsBefore.insert(session.id);
    }

    std::unique_ptr<LinkFaultProxy> proxy;
    try {
        proxy = std::make_unique<LinkFaultProxy>(board->name, plan);
    } catch(const std::exception& e) {
        SKIP(std::string("Cannot start the local proxy: ") + e.what());
    }

    {
        // The device is reached through the proxy, so a name filter for the real IP must not apply
        ScopedEnvVar noNameFilter("DEPTHAI_DEVICE_NAME_LIST", std::nullopt);
        dai::DeviceInfo viaProxy("127.0.0.1");
        viaProxy.platform = X_LINK_RVC4;
        context.device = std::make_shared<dai::Device>(viaProxy);
    }

    Outcome outcome;
    {
        dai::Pipeline pipeline(context.device);
        auto script = pipeline.create<dai::node::Script>();
        script->setScript("import time\nwhile True:\n    time.sleep(1)\n");
        // A large asset keeps the transfer running when the fault comes
        pipeline.getAssetManager().set("big_asset", std::vector<std::uint8_t>(assetSize, 0xA5));

        // Catch2 assertions are not thread safe: the starter thread only records the result
        std::promise<void> done;
        auto doneFuture = done.get_future();
        std::thread starter([&] {
            auto t0 = std::chrono::steady_clock::now();
            try {
                pipeline.start();
            } catch(const std::runtime_error& e) {
                outcome.threw = true;
                outcome.runtimeError = true;
                outcome.message = e.what();
            } catch(const std::exception& e) {
                outcome.threw = true;
                outcome.message = e.what();
            }
            outcome.duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
            done.set_value();
        });

        if(onFault && proxy->waitForFault(FAULT_TIMEOUT)) {
            // An exception here would leave the starter thread joinable (std::terminate); record it and fail after the join
            try {
                onFault(context);
            } catch(const std::exception& e) {
                outcome.faultActionError = e.what();
            } catch(...) {
                outcome.faultActionError = "unknown exception";
            }
        }
        if(doneFuture.wait_for(START_TIMEOUT) != std::future_status::ready) {
            proxy->release();
            if(doneFuture.wait_for(RESCUE_TIMEOUT) != std::future_status::ready) {
                // The starter thread uses the pipeline on this stack, so it cannot be left behind
                std::cerr << "pipeline.start() did not return " << std::chrono::seconds(START_TIMEOUT + RESCUE_TIMEOUT).count()
                          << " s after it began, even after the proxy reset the link" << std::endl;
                std::_Exit(EXIT_FAILURE);
            }
        }
        starter.join();
    }
    context.device.reset();
    outcome.assetStreamSeen = proxy->assetStreamSeen();
    outcome.faultApplied = proxy->faultApplied();
    proxy.reset();

    UNSCOPED_INFO("start() took " << outcome.duration.count() << " ms; threw: " << outcome.threw << "; message: " << outcome.message);
    INFO("Fault action failed: " << outcome.faultActionError);
    REQUIRE(outcome.faultActionError.empty());
    return outcome;
}

void requireLinkLostError(const Outcome& outcome) {
    REQUIRE(outcome.faultApplied);
    REQUIRE(outcome.threw);
    CHECK(outcome.runtimeError);
    CHECK_THAT(outcome.message, ContainsSubstring(std::string(LINK_LOST_MESSAGE)));
    CHECK(outcome.duration < START_TIMEOUT);
}

void requireStarted(const Outcome& outcome) {
    REQUIRE(outcome.assetStreamSeen);
    CHECK_FALSE(outcome.threw);
}

// Crash dump collection is not part of these tests; do not save or upload one
class NoCrashdumpFixture {
    ScopedEnvVar noCrashdump{"DEPTHAI_CRASHDUMP", "0"};
    ScopedEnvVar noUpload{"DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1"};
};

}  // namespace

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: connection reset in the middle of the transfer") {
    requireLinkLostError(runScenario({Fault::Reset, 4 * MiB}, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: connection closed in order (FIN) in the middle of the transfer") {
    requireLinkLostError(runScenario({Fault::Close, 4 * MiB}, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: connection reset when the stream opens") {
    requireLinkLostError(runScenario({Fault::Reset, 0}, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: connection reset in the last bytes of the transfer") {
    requireLinkLostError(runScenario({Fault::Reset, ASSET_SIZE - 256 * 1024}, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: device vanishes (reset and then no reconnection or discovery)") {
    FaultPlan plan{Fault::Reset, 4 * MiB};
    plan.vanish = true;
    requireLinkLostError(runScenario(plan, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: cable pulled (blackhole with no RST or FIN)") {
    requireLinkLostError(runScenario({Fault::Blackhole, 4 * MiB}, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: one-way link (device to host traffic lost)") {
    requireLinkLostError(runScenario({Fault::MuteDevice, 4 * MiB}, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: outage longer than the watchdog") {
    FaultPlan plan{Fault::Stall, 4 * MiB};
    plan.stall = 15s;
    requireLinkLostError(runScenario(plan, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: short outage that is shorter than the watchdog") {
    FaultPlan plan{Fault::Stall, 4 * MiB};
    plan.stall = 2s;
    requireStarted(runScenario(plan, ASSET_SIZE));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: link splits the stream into small random pieces") {
    requireStarted(runScenario({Fault::Fragment, 0}, 16 * MiB));
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: firmware is killed during the transfer") {
    auto outcome = runScenario({Fault::None, 4 * MiB}, ASSET_SIZE, [](const ScenarioContext& context) {
        std::vector<std::string> mine;
        for(const auto& session : gateSessions(context.boardIp)) {
            if(session.state == "RUNNING" && context.sessionsBefore.count(session.id) == 0) {
                mine.push_back(session.id);
            }
        }
        // Stop only the session of this test, and only if it is the only new one
        if(mine.size() != 1) {
            throw std::runtime_error("Cannot find the session of this test: " + std::to_string(mine.size()) + " new RUNNING sessions");
        }
        gateRequest(context.boardIp, "POST", "/api/v1/sessions/" + mine.front() + "/stop");
    });
    requireLinkLostError(outcome);
}

TEST_CASE_METHOD(NoCrashdumpFixture, "Asset storage: application closes the device from another thread during the transfer") {
    auto outcome = runScenario({Fault::None, 4 * MiB}, ASSET_SIZE, [](const ScenarioContext& context) { context.device->close(); });
    REQUIRE(outcome.faultApplied);
    // The process must not abort or hang; start() fails because the device is closed
    CHECK(outcome.threw);
    CHECK(outcome.duration < START_TIMEOUT);
}
