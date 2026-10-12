#include "device/TimesyncUdp.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "utility/Logging.hpp"

#if defined(__linux__) || defined(__APPLE__)
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <netinet/in.h>
    #include <poll.h>
    #include <sys/socket.h>
    #include <unistd.h>

    #include <cerrno>
    #include <ctime>
    #ifdef __linux__
        #include <linux/errqueue.h>
        #include <linux/net_tstamp.h>
    #endif
    #define DAI_TIMESYNC_UDP_POSIX 1
#endif

namespace dai {

#ifdef DAI_TIMESYNC_UDP_POSIX
namespace {

// Wire format, keep in sync with the firmware (Timesync.cpp)
constexpr uint32_t UDP_MAGIC = 0x55535444;  // "DTSU"
constexpr uint16_t UDP_VERSION = 1;
enum UdpMsgType : uint16_t { UDP_HELLO = 1, UDP_REQ = 2, UDP_RESP = 3, UDP_FUP = 4 };
constexpr uint32_t UDP_FLAG_KERNEL_STAMPS = 1;  // FUP: t2/t3 are kernel stamps (else application stamps)
constexpr uint32_t UDP_FLAG_INVALID = 2;        // FUP: this exchange could not be stamped
    #pragma pack(push, 1)
struct UdpMsg {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint64_t token;
    uint64_t seq;
    int64_t t2;  // RESP
    int64_t t3;  // FUP
    uint32_t flags;
    uint32_t reserved;
};
    #pragma pack(pop)
static_assert(sizeof(UdpMsg) == 48, "wire format");

constexpr auto HELLO_FAST = std::chrono::milliseconds(2);  // until the first REQ, for the first few seconds: the firmware
                                                           // opens its socket after setTimesync and waits for a HELLO
constexpr auto HELLO_FAST_FOR = std::chrono::seconds(3);
constexpr auto HELLO_SLOW = std::chrono::milliseconds(500);  // no REQ yet: the firmware may still pick the UDP path up

int64_t steadyNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

    #ifdef __linux__
int64_t clockNs(clockid_t id) {
    struct timespec ts;
    clock_gettime(id, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// CLOCK_REALTIME (kernel stamps) - steady clock (CLOCK_MONOTONIC), from the tightest of three bracketed reads. Both are
// slewed alike, so this changes only when the wall clock is stepped.
int64_t realtimeMinusSteady() {
    int64_t best = INT64_MAX, off = 0;
    for(int i = 0; i < 3; i++) {
        const int64_t m0 = steadyNs();
        const int64_t r = clockNs(CLOCK_REALTIME);
        const int64_t m1 = steadyNs();
        if(m1 - m0 < best) {
            best = m1 - m0;
            off = r - (m0 + (m1 - m0) / 2);
        }
    }
    return off;
}

int64_t cmsgStamp(struct msghdr* mh) {
    for(struct cmsghdr* c = CMSG_FIRSTHDR(mh); c; c = CMSG_NXTHDR(mh, c)) {
        if(c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPING) {
            struct scm_timestamping st;
            memcpy(&st, CMSG_DATA(c), sizeof(st));
            return static_cast<int64_t>(st.ts[0].tv_sec) * 1000000000LL + st.ts[0].tv_nsec;
        }
    }
    return 0;
}

// TX timestamp with an OPT_ID key >= minKey from the error queue (waits up to timeoutMs, 0 on timeout)
int64_t txStamp(int fd, uint32_t minKey, int timeoutMs, uint32_t& key) {
    const int64_t deadline = steadyNs() + timeoutMs * 1000000LL;
    for(;;) {
        char ctrl[512], data[64];
        struct iovec iov = {data, sizeof(data)};
        struct msghdr mh = {};
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = ctrl;
        mh.msg_controllen = sizeof(ctrl);
        if(recvmsg(fd, &mh, MSG_ERRQUEUE | MSG_DONTWAIT) >= 0) {
            int64_t t = 0;
            bool haveKey = false;
            uint32_t k = 0;
            for(struct cmsghdr* c = CMSG_FIRSTHDR(&mh); c; c = CMSG_NXTHDR(&mh, c)) {
                if(c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPING) {
                    struct scm_timestamping st;
                    memcpy(&st, CMSG_DATA(c), sizeof(st));
                    t = static_cast<int64_t>(st.ts[0].tv_sec) * 1000000000LL + st.ts[0].tv_nsec;
                }
                if(c->cmsg_level == SOL_IP && c->cmsg_type == IP_RECVERR) {
                    struct sock_extended_err e;
                    memcpy(&e, CMSG_DATA(c), sizeof(e));
                    if(e.ee_origin == SO_EE_ORIGIN_TIMESTAMPING) {
                        k = e.ee_data;
                        haveKey = true;
                    }
                }
            }
            if(t && haveKey && static_cast<int32_t>(k - minKey) >= 0) {
                key = k;
                return t;
            }
            continue;
        }
        const int64_t left = (deadline - steadyNs()) / 1000000LL;
        if(left <= 0) return 0;
        struct pollfd p = {fd, POLLPRI, 0};
        poll(&p, 1, static_cast<int>(left));
    }
}
    #endif

bool sendMsg(int fd, const UdpMsg& m, bool stamped) {
    #ifdef __linux__
    if(stamped) {
        char ctrl[CMSG_SPACE(sizeof(uint32_t))] = {};
        struct iovec iov = {const_cast<UdpMsg*>(&m), sizeof(m)};
        struct msghdr mh = {};
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = ctrl;
        mh.msg_controllen = sizeof(ctrl);
        struct cmsghdr* c = CMSG_FIRSTHDR(&mh);
        c->cmsg_level = SOL_SOCKET;
        c->cmsg_type = SO_TIMESTAMPING;
        c->cmsg_len = CMSG_LEN(sizeof(uint32_t));
        const uint32_t flags = SOF_TIMESTAMPING_TX_SOFTWARE;
        memcpy(CMSG_DATA(c), &flags, sizeof(flags));
        return sendmsg(fd, &mh, 0) == static_cast<ssize_t>(sizeof(m));
    }
    #else
    (void)stamped;
    #endif
    return send(fd, &m, sizeof(m), 0) == static_cast<ssize_t>(sizeof(m));
}

// "10.0.0.2", "10.0.0.2:11490" or a host name -> IPv4 address
bool resolve(const std::string& name, struct sockaddr_in& out) {
    std::string host = name;
    const auto colon = host.find(':');
    if(colon != std::string::npos) {
        if(host.find(':', colon + 1) != std::string::npos) return false;  // IPv6: the firmware listens on IPv4 only
        host = host.substr(0, colon);
    }
    out = {};
    out.sin_family = AF_INET;
    out.sin_port = htons(TimesyncUdpResponder::DEVICE_PORT);
    if(inet_pton(AF_INET, host.c_str(), &out.sin_addr) == 1) return true;
    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    struct addrinfo* res = nullptr;
    if(getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0 || !res) return false;
    out.sin_addr = reinterpret_cast<struct sockaddr_in*>(res->ai_addr)->sin_addr;
    freeaddrinfo(res);
    return true;
}

}  // namespace

bool TimesyncUdpResponder::supported() {
    return true;
}

TimesyncUdpResponder::TimesyncUdpResponder(const std::string& deviceIp, uint64_t token) : token(token) {
    struct sockaddr_in dev;
    if(!resolve(deviceIp, dev)) {
        logger::debug("UDP timesync: cannot resolve '{}'", deviceIp);
        return;
    }
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd < 0) return;
    if(connect(fd, reinterpret_cast<struct sockaddr*>(&dev), sizeof(dev)) < 0) {
        close(fd);
        fd = -1;
        return;
    }
    #ifdef __linux__
    const int tsFlags = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_OPT_TSONLY | SOF_TIMESTAMPING_OPT_ID;
    kernelStamps = setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPING, &tsFlags, sizeof(tsFlags)) == 0;
    #endif
    thread = std::thread([this]() { run(); });
}

TimesyncUdpResponder::~TimesyncUdpResponder() {
    stopRequested = true;
    if(thread.joinable()) thread.join();
    if(fd >= 0) close(fd);
}

bool TimesyncUdpResponder::ok() const {
    return fd >= 0 && thread.joinable();
}

uint64_t TimesyncUdpResponder::answered() const {
    return answeredCount.load();
}

void TimesyncUdpResponder::run() {
    using namespace std::chrono;
    const auto start = steady_clock::now();
    auto nextHello = start;
    bool gotReq = false;
    uint32_t txKey = 0;  // SOF_TIMESTAMPING_OPT_ID numbers the packets that request a TX stamp from 0
    UdpMsg hello = {UDP_MAGIC, UDP_VERSION, UDP_HELLO, token, 0, 0, 0, 0, 0};
    while(!stopRequested) {
        auto now = steady_clock::now();
        if(!gotReq && now >= nextHello) {
            // Before the firmware opens its socket, a HELLO comes back as ICMP port unreachable, which the next send on
            // this connected socket reports instead of sending: send that one again
            if(!sendMsg(fd, hello, false) && errno == ECONNREFUSED) sendMsg(fd, hello, false);
            nextHello = now + (now - start < HELLO_FAST_FOR ? duration_cast<steady_clock::duration>(HELLO_FAST) : duration_cast<steady_clock::duration>(HELLO_SLOW));
        }
        int timeoutMs = 100;
        if(!gotReq) timeoutMs = static_cast<int>(std::max<int64_t>(0, duration_cast<milliseconds>(nextHello - now).count()));
        struct pollfd p = {fd, POLLIN, 0};
        if(poll(&p, 1, timeoutMs) <= 0) continue;
    #ifdef __linux__
        if(!(p.revents & POLLIN)) {
            int err = 0;
            socklen_t len = sizeof(err);
            getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);  // clears a pending ICMP error, else poll returns at once
            uint32_t stale = 0;
            if(kernelStamps) txStamp(fd, txKey, 0, stale);  // only a late TX stamp is pending: drop it
            continue;
        }
    #endif
        UdpMsg m;
        char ctrl[512];
        struct iovec iov = {&m, sizeof(m)};
        struct msghdr mh = {};
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = ctrl;
        mh.msg_controllen = sizeof(ctrl);
        const ssize_t r = recvmsg(fd, &mh, MSG_DONTWAIT);
        const int64_t t2app = steadyNs();
        if(r != static_cast<ssize_t>(sizeof(m)) || m.magic != UDP_MAGIC || m.version != UDP_VERSION || m.type != UDP_REQ || m.token != token) continue;
        gotReq = true;

        int64_t t2 = t2app;
        uint32_t flags = 0;
    #ifdef __linux__
        int64_t wall = 0;
        if(kernelStamps) {
            wall = realtimeMinusSteady();
            const int64_t t2k = cmsgStamp(&mh);
            if(t2k) {
                t2 = t2k - wall;
                flags = UDP_FLAG_KERNEL_STAMPS;
            } else {
                flags = UDP_FLAG_INVALID;
            }
        }
    #endif
        UdpMsg resp = m;
        resp.type = UDP_RESP;
        resp.t2 = t2;
        resp.t3 = 0;
        resp.flags = 0;
        const int64_t t3app = steadyNs();
        const bool stamped = (flags & UDP_FLAG_KERNEL_STAMPS) != 0;
        if(!sendMsg(fd, resp, stamped)) continue;
        int64_t t3 = t3app;
    #ifdef __linux__
        if(stamped) {
            uint32_t key = 0;
            const int64_t t3k = txStamp(fd, txKey, 20, key);
            txKey = t3k ? key + 1 : txKey + 1;
            // a wall clock step between the two conversions would corrupt t2 or t3
            if(t3k && std::llabs(realtimeMinusSteady() - wall) < 1000) {
                t3 = t3k - wall;
            } else {
                flags |= UDP_FLAG_INVALID;
            }
        }
    #endif
        UdpMsg fup = m;
        fup.type = UDP_FUP;
        fup.t2 = t2;
        fup.t3 = t3;
        fup.flags = flags;
        sendMsg(fd, fup, false);
        answeredCount++;
    }
}

#else

bool TimesyncUdpResponder::supported() {
    return false;
}
TimesyncUdpResponder::TimesyncUdpResponder(const std::string&, uint64_t token) : token(token) {}
TimesyncUdpResponder::~TimesyncUdpResponder() = default;
bool TimesyncUdpResponder::ok() const {
    return false;
}
uint64_t TimesyncUdpResponder::answered() const {
    return 0;
}
void TimesyncUdpResponder::run() {}

#endif

}  // namespace dai
