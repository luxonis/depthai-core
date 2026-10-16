#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace dai {

/**
 * Host side of the RVC4 UDP timesync (firmware Timesync.cpp, "UDP fast path").
 *
 * The device starts each exchange (REQ) and this responder answers in two steps, PTP-style: RESP carries the host
 * receive stamp of the REQ, FUP the host send stamp of the RESP, both on the host's steady clock. On Linux these are
 * kernel software timestamps (SO_TIMESTAMPING), so the responder thread's scheduling does not enter the estimate;
 * elsewhere application stamps. Until the first REQ arrives, HELLO (with the session token the firmware got over
 * RPC) goes out every 2 ms: the device learns the host's address from it, and host firewalls / NAT see the
 * device's packets as replies.
 *
 * The same socket carries small calls either way (RPC_CALL / RPC_REPLY, 32-byte payloads): the caller resends a call
 * until the reply, the callee answers a repeated call from its cache.
 */
class TimesyncUdpResponder {
   public:
    static constexpr int DEVICE_PORT = 11493;
    using RpcPayload = std::array<uint8_t, 32>;
    static constexpr uint16_t RPC_UNKNOWN_METHOD = 0xffff;
    // Answers the device's calls on the responder thread (keep it quick): reply status, out = reply payload
    using RpcHandler = std::function<uint16_t(uint16_t method, const RpcPayload& in, RpcPayload& out)>;
    // End of a call: replied (else the deadline passed), the reply's status and payload
    using RpcDone = std::function<void(bool replied, uint16_t status, const RpcPayload& reply)>;

    // Whether this platform has a responder (POSIX sockets)
    static bool supported();

    // Starts the responder thread for the device at deviceIp (IPv4 literal or host name, optional ":port" suffix);
    // a sync-start group finds it by its owner (the DeviceBase)
    TimesyncUdpResponder(const std::string& deviceIp, uint64_t token, const void* owner = nullptr);
    ~TimesyncUdpResponder();
    TimesyncUdpResponder(const TimesyncUdpResponder&) = delete;
    TimesyncUdpResponder& operator=(const TimesyncUdpResponder&) = delete;

    // Socket set up and thread running
    bool ok() const;
    // Exchanges answered so far
    uint64_t answered() const;

    void setRpcHandler(RpcHandler handler);
    // Calls the device, resending until it replies or the deadline passes; done runs on the responder thread
    void call(uint16_t method, const RpcPayload& payload, std::chrono::milliseconds deadline, RpcDone done);

   private:
    struct Call {
        uint32_t id;
        uint16_t method;
        RpcPayload payload;
        std::chrono::steady_clock::time_point next, deadline;
        std::chrono::milliseconds interval;
        RpcDone done;
    };

    void run();
    void sendCall(const Call& c);
    void receiveRpc(const void* packet);
    std::chrono::steady_clock::time_point resendCalls();

    int fd = -1;
    uint64_t token;
    const void* owner;
    bool kernelStamps = false;
    std::atomic<bool> stopRequested{false};
    std::atomic<uint64_t> answeredCount{0};
    std::mutex rpcMtx;
    RpcHandler rpcHandler;
    std::vector<Call> calls;
    uint32_t nextCallId = 1;
    std::array<std::array<uint8_t, 56>, 16> replies{};  // the last replies sent (call id at offset 16, 0 = empty)
    size_t nextReply = 0;
    std::thread thread;
};

/**
 * Synchronized camera start of a multi-device pipeline (RVC4 with the UDP timesync, firmware CameraSensor.cpp).
 * Before the device pipelines start, every device is armed with how many cameras the pipeline runs on it (ARM). The
 * devices hold their cameras' stream-on until all are configured and report READY; once all did (or 10 s passed), each
 * gets the same start time, a frame boundary ~150 ms ahead on the host clock (START_AT). The devices convert it to
 * their clock only then, so the timesync's drift error covers those 150 ms, not the seconds the setup took.
 */
class SyncStartGroup {
   public:
    // Arms the devices (DeviceBase, cameras on it) that have a UDP timesync for the pipeline's start and coordinates it
    // when there are at least two; replaces an earlier group of the pipeline
    static void begin(const void* pipeline, const std::vector<std::pair<const void*, uint16_t>>& devices);
    // Ends the pipeline's group (before its devices close)
    static void end(const void* pipeline);

    ~SyncStartGroup();

   private:
    struct Member {
        const void* device;  // its responder is looked up for every use: a reconnect or close replaces it
        uint16_t cameras;
        bool ready = false;
        int64_t framePeriodNs = 0;
    };
    void coordinate();

    uint32_t generation = 0;
    std::mutex mtx;
    std::condition_variable cv;
    bool stopping = false;
    std::vector<Member> members;
    std::thread thread;
};

}  // namespace dai
