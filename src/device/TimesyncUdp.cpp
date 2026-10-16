#include "device/TimesyncUdp.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>

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
constexpr uint16_t UDP_VERSION = 3;
enum UdpMsgType : uint16_t { UDP_HELLO = 1, UDP_REQ = 2, UDP_RESP = 3, UDP_FUP = 4, UDP_RPC_CALL = 5, UDP_RPC_REPLY = 6 };
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
    int64_t reserved2;
};
struct UdpRpc {
    uint32_t magic;
    uint16_t version;
    uint16_t type;  // UDP_RPC_CALL or UDP_RPC_REPLY
    uint64_t token;
    uint32_t callId;
    uint16_t method;
    uint16_t status;  // reply
    uint8_t payload[32];
};
    #pragma pack(pop)
static_assert(sizeof(UdpMsg) == 56, "wire format");
static_assert(sizeof(UdpRpc) == sizeof(UdpMsg), "one packet size on the socket");

// Synchronized start of a multi-device pipeline, keep in sync with the firmware (Timesync.hpp, CameraSensor.cpp)
namespace sync_start {
enum Method : uint16_t { ARM = 1, READY = 2, START_AT = 3 };
enum Status : uint16_t { OK = 0, LATE = 1, STALE = 2 };
    #pragma pack(push, 1)
struct Arm {  // host -> device before its pipeline starts: hold the cameras, the pipeline runs this many here
    uint32_t generation;
    uint16_t cameras;
};
struct Ready {  // device -> host: all cameras configured, stream-on held
    uint32_t generation;
    uint16_t cameras;
    int64_t framePeriodNs;
};
struct StartAt {  // host -> device: stream on at this host steady-clock time; a LATE reply carries int64_t lateNs
    uint32_t generation;
    int64_t startHostNs;
};
    #pragma pack(pop)
}  // namespace sync_start

constexpr auto RPC_RESEND_FIRST = std::chrono::milliseconds(2);  // then doubling, up to RPC_RESEND_MAX
constexpr auto RPC_RESEND_MAX = std::chrono::milliseconds(50);
constexpr auto SYNC_ARM_DEADLINE = std::chrono::milliseconds(300);
constexpr auto SYNC_READY_TIMEOUT = std::chrono::seconds(10);
constexpr auto SYNC_START_MARGIN = std::chrono::milliseconds(150);  // READY of the last device -> start
constexpr auto SYNC_START_DEADLINE = std::chrono::milliseconds(100);

// Responders by owner (DeviceBase), sync-start groups by pipeline
std::mutex registryMtx;
std::map<const void*, TimesyncUdpResponder*> responders;
std::map<const void*, std::shared_ptr<SyncStartGroup>> groups;
std::atomic<uint32_t> lastGeneration{0};

// Runs fn on the device's responder; it cannot stop meanwhile (it unregisters first). False: the device has none.
template <typename F>
bool withResponder(const void* device, F&& fn) {
    std::lock_guard<std::mutex> lock(registryMtx);
    auto it = responders.find(device);
    if(it == responders.end()) return false;
    fn(*it->second);
    return true;
}

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

TimesyncUdpResponder::TimesyncUdpResponder(const std::string& deviceIp, uint64_t token, const void* owner) : token(token), owner(owner) {
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
    if(owner != nullptr) {
        std::lock_guard<std::mutex> lock(registryMtx);
        responders[owner] = this;
    }
}

TimesyncUdpResponder::~TimesyncUdpResponder() {
    if(owner != nullptr) {
        std::lock_guard<std::mutex> lock(registryMtx);
        auto it = responders.find(owner);
        if(it != responders.end() && it->second == this) responders.erase(it);
    }
    stopRequested = true;
    if(thread.joinable()) thread.join();
    if(fd >= 0) close(fd);
    std::vector<Call> pending;
    {
        std::lock_guard<std::mutex> lock(rpcMtx);
        pending.swap(calls);
    }
    for(auto& c : pending) {
        if(c.done) c.done(false, 0, RpcPayload{});
    }
}

void TimesyncUdpResponder::setRpcHandler(RpcHandler handler) {
    std::lock_guard<std::mutex> lock(rpcMtx);
    rpcHandler = std::move(handler);
}

void TimesyncUdpResponder::sendCall(const Call& c) {
    UdpRpc m = {UDP_MAGIC, UDP_VERSION, UDP_RPC_CALL, token, c.id, c.method, 0, {}};
    memcpy(m.payload, c.payload.data(), sizeof(m.payload));
    send(fd, &m, sizeof(m), 0);
}

void TimesyncUdpResponder::call(uint16_t method, const RpcPayload& payload, std::chrono::milliseconds deadline, RpcDone done) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(rpcMtx);
    Call c{nextCallId++, method, payload, now + RPC_RESEND_FIRST, now + deadline, RPC_RESEND_FIRST * 2, std::move(done)};
    if(fd >= 0) sendCall(c);
    calls.push_back(std::move(c));
}

// Resends the calls that are due and ends the expired ones; when the next one is due (max() when none)
std::chrono::steady_clock::time_point TimesyncUdpResponder::resendCalls() {
    const auto now = std::chrono::steady_clock::now();
    auto due = std::chrono::steady_clock::time_point::max();
    std::vector<RpcDone> expired;
    {
        std::lock_guard<std::mutex> lock(rpcMtx);
        for(auto it = calls.begin(); it != calls.end();) {
            if(now >= it->deadline) {
                expired.push_back(std::move(it->done));
                it = calls.erase(it);
                continue;
            }
            if(now >= it->next) {
                sendCall(*it);
                it->next = now + it->interval;
                it->interval = std::min(it->interval * 2, RPC_RESEND_MAX);
            }
            due = std::min(due, it->next);
            ++it;
        }
    }
    for(auto& done : expired) {
        if(done) done(false, 0, RpcPayload{});
    }
    return due;
}

// An RPC packet from the device: a reply ends our call, a call is answered (a repeated one from the cache)
void TimesyncUdpResponder::receiveRpc(const void* packet) {
    UdpRpc m;
    memcpy(&m, packet, sizeof(m));
    RpcPayload payload;
    memcpy(payload.data(), m.payload, payload.size());
    if(m.type == UDP_RPC_REPLY) {
        RpcDone done;
        {
            std::lock_guard<std::mutex> lock(rpcMtx);
            auto it = std::find_if(calls.begin(), calls.end(), [&](const Call& c) { return c.id == m.callId; });
            if(it == calls.end()) return;
            done = std::move(it->done);
            calls.erase(it);
        }
        if(done) done(true, m.status, payload);
        return;
    }
    RpcHandler handler;
    {
        std::lock_guard<std::mutex> lock(rpcMtx);
        for(const auto& r : replies) {
            UdpRpc cached;
            memcpy(&cached, r.data(), sizeof(cached));
            if(cached.callId != 0 && cached.callId == m.callId) {
                send(fd, r.data(), r.size(), 0);
                return;
            }
        }
        handler = rpcHandler;
    }
    RpcPayload out{};
    UdpRpc reply = m;
    reply.type = UDP_RPC_REPLY;
    reply.status = handler ? handler(m.method, payload, out) : RPC_UNKNOWN_METHOD;
    memcpy(reply.payload, out.data(), sizeof(reply.payload));
    {
        std::lock_guard<std::mutex> lock(rpcMtx);
        memcpy(replies[nextReply++ % replies.size()].data(), &reply, sizeof(reply));
    }
    send(fd, &reply, sizeof(reply), 0);
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
    UdpMsg hello = {UDP_MAGIC, UDP_VERSION, UDP_HELLO, token, 0, 0, 0, 0, 0, 0};
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
        const auto callDue = resendCalls();
        if(callDue != steady_clock::time_point::max()) {
            timeoutMs = std::min(timeoutMs, static_cast<int>(std::max<int64_t>(0, duration_cast<milliseconds>(callDue - now).count() + 1)));
        }
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
        if(r != static_cast<ssize_t>(sizeof(m)) || m.magic != UDP_MAGIC || m.version != UDP_VERSION || m.token != token) continue;
        if(m.type == UDP_RPC_CALL || m.type == UDP_RPC_REPLY) {
            receiveRpc(&m);
            continue;
        }
        if(m.type != UDP_REQ) continue;
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

void SyncStartGroup::begin(const void* pipeline, const std::vector<std::pair<const void*, uint16_t>>& devices) {
    end(pipeline);
    std::vector<std::pair<const void*, uint16_t>> candidates;  // with cameras and a UDP timesync
    for(const auto& d : devices) {
        if(d.second > 0 && withResponder(d.first, [](TimesyncUdpResponder&) {})) candidates.push_back(d);
    }
    if(candidates.size() < 2) return;
    auto group = std::make_shared<SyncStartGroup>();
    group->generation = ++lastGeneration;
    // Arm them; only the armed ones are waited for, the others start on their own
    std::mutex armMtx;
    std::condition_variable armCv;
    size_t pending = 0;
    std::vector<uint8_t> armed(candidates.size(), 0);
    for(size_t i = 0; i < candidates.size(); i++) {
        const sync_start::Arm arm{group->generation, candidates[i].second};
        TimesyncUdpResponder::RpcPayload payload{};
        memcpy(payload.data(), &arm, sizeof(arm));
        {
            std::lock_guard<std::mutex> lock(armMtx);
            ++pending;
        }
        const bool sent = withResponder(candidates[i].first, [&, i](TimesyncUdpResponder& responder) {
            responder.call(sync_start::ARM, payload, SYNC_ARM_DEADLINE, [&, i](bool replied, uint16_t status, const TimesyncUdpResponder::RpcPayload&) {
                std::lock_guard<std::mutex> lock(armMtx);
                armed[i] = replied && status == sync_start::OK;
                --pending;
                armCv.notify_all();
            });
        });
        if(!sent) {
            std::lock_guard<std::mutex> lock(armMtx);
            --pending;
        }
    }
    {
        std::unique_lock<std::mutex> lock(armMtx);
        armCv.wait(lock, [&] { return pending == 0; });
    }
    for(size_t i = 0; i < candidates.size(); i++) {
        if(armed[i]) group->members.push_back({candidates[i].first, candidates[i].second});
    }
    if(group->members.size() < candidates.size()) {
        logger::warn("Synchronized start: {} of {} devices armed, the others start on their own", group->members.size(), candidates.size());
    }
    if(group->members.empty()) return;
    const std::weak_ptr<SyncStartGroup> weak = group;
    for(size_t i = 0; i < group->members.size(); i++) {
        withResponder(group->members[i].device, [&weak, i](TimesyncUdpResponder& responder) {
            responder.setRpcHandler([weak, i](uint16_t method, const TimesyncUdpResponder::RpcPayload& in, TimesyncUdpResponder::RpcPayload&) -> uint16_t {
                if(method != sync_start::READY) return TimesyncUdpResponder::RPC_UNKNOWN_METHOD;
                const auto g = weak.lock();
                if(!g) return sync_start::STALE;
                sync_start::Ready ready;
                memcpy(&ready, in.data(), sizeof(ready));
                std::lock_guard<std::mutex> lock(g->mtx);
                if(ready.generation != g->generation) return sync_start::STALE;
                g->members[i].ready = true;
                g->members[i].framePeriodNs = ready.framePeriodNs;
                g->cv.notify_all();
                return sync_start::OK;
            });
        });
    }
    group->thread = std::thread([g = group.get()]() { g->coordinate(); });
    std::lock_guard<std::mutex> lock(registryMtx);
    groups[pipeline] = std::move(group);
}

// Waits for every device's READY (SYNC_READY_TIMEOUT at most) and sends all of them the start time
void SyncStartGroup::coordinate() {
    using namespace std::chrono;
    int64_t startNs = 0;
    std::vector<const void*> targets;
    {
        std::unique_lock<std::mutex> lock(mtx);
        const bool allReady = cv.wait_for(lock, SYNC_READY_TIMEOUT, [&] {
            return stopping || std::all_of(members.begin(), members.end(), [](const Member& m) { return m.ready; });
        });
        if(stopping) return;
        if(!allReady) logger::warn("Synchronized start: not every device reported its cameras ready, starting the others");
        int64_t periodNs = 0;
        bool samePeriod = true;
        for(const auto& m : members) {
            targets.push_back(m.device);
            if(!m.ready || m.framePeriodNs <= 0) continue;
            if(periodNs == 0) {
                periodNs = m.framePeriodNs;
            } else if(m.framePeriodNs != periodNs) {
                samePeriod = false;
            }
        }
        startNs = steadyNs() + duration_cast<nanoseconds>(SYNC_START_MARGIN).count();
        if(periodNs > 0 && samePeriod) {
            startNs = (startNs + periodNs - 1) / periodNs * periodNs;  // a frame boundary of the host clock, like TIME_DAI's grid
        } else if(!samePeriod) {
            logger::warn("Synchronized start: the devices run different frame rates, the start is not on a frame boundary");
        }
    }
    const sync_start::StartAt at{generation, startNs};
    TimesyncUdpResponder::RpcPayload payload{};
    memcpy(payload.data(), &at, sizeof(at));
    const uint32_t gen = generation;
    for(const void* device : targets) {
        withResponder(device, [&](TimesyncUdpResponder& responder) {
            responder.call(sync_start::START_AT, payload, SYNC_START_DEADLINE, [gen](bool replied, uint16_t status, const TimesyncUdpResponder::RpcPayload& reply) {
                if(!replied) {
                    logger::warn("Synchronized start {}: a device did not confirm the start time", gen);
                } else if(status == sync_start::LATE) {
                    int64_t lateNs = 0;
                    memcpy(&lateNs, reply.data(), sizeof(lateNs));
                    logger::warn("Synchronized start {}: a device got the start time {:.1f} ms late", gen, lateNs / 1e6);
                }
            });
        });
    }
    logger::debug("Synchronized start {}: {} devices stream on at host {} ns", gen, targets.size(), startNs);
}

void SyncStartGroup::end(const void* pipeline) {
    std::shared_ptr<SyncStartGroup> group;
    {
        std::lock_guard<std::mutex> lock(registryMtx);
        auto it = groups.find(pipeline);
        if(it == groups.end()) return;
        group = std::move(it->second);
        groups.erase(it);
    }
}

SyncStartGroup::~SyncStartGroup() {
    {
        std::lock_guard<std::mutex> lock(mtx);
        stopping = true;
    }
    cv.notify_all();
    if(thread.joinable()) thread.join();
    for(auto& m : members) withResponder(m.device, [](TimesyncUdpResponder& responder) { responder.setRpcHandler(nullptr); });
}

#else

bool TimesyncUdpResponder::supported() {
    return false;
}
TimesyncUdpResponder::TimesyncUdpResponder(const std::string&, uint64_t token, const void* owner) : token(token), owner(owner) {}
TimesyncUdpResponder::~TimesyncUdpResponder() = default;
bool TimesyncUdpResponder::ok() const {
    return false;
}
uint64_t TimesyncUdpResponder::answered() const {
    return 0;
}
void TimesyncUdpResponder::setRpcHandler(RpcHandler) {}
void TimesyncUdpResponder::call(uint16_t, const RpcPayload&, std::chrono::milliseconds, RpcDone done) {
    if(done) done(false, 0, RpcPayload{});
}
void TimesyncUdpResponder::run() {}
void SyncStartGroup::begin(const void*, const std::vector<std::pair<const void*, uint16_t>>&) {}
void SyncStartGroup::end(const void*) {}
SyncStartGroup::~SyncStartGroup() = default;

#endif

}  // namespace dai
