#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

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
 */
class TimesyncUdpResponder {
   public:
    static constexpr int DEVICE_PORT = 11493;

    // Whether this platform has a responder (POSIX sockets)
    static bool supported();

    // Starts the responder thread for the device at deviceIp (IPv4 literal or host name, optional ":port" suffix)
    TimesyncUdpResponder(const std::string& deviceIp, uint64_t token);
    ~TimesyncUdpResponder();
    TimesyncUdpResponder(const TimesyncUdpResponder&) = delete;
    TimesyncUdpResponder& operator=(const TimesyncUdpResponder&) = delete;

    // Socket set up and thread running
    bool ok() const;
    // Exchanges answered so far
    uint64_t answered() const;

   private:
    void run();

    int fd = -1;
    uint64_t token;
    bool kernelStamps = false;
    std::atomic<bool> stopRequested{false};
    std::atomic<uint64_t> answeredCount{0};
    std::thread thread;
};

}  // namespace dai
