#include "Timesync.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <vector>

static int MvNanosToTimespec(struct timespec* ts, const int64_t ns) {
    if(!ts) {
        return -1;
    }
    ts->tv_sec = ns / 1000000000;
    ts->tv_nsec = ns - ts->tv_sec * 1000000000;

    return 0;
}
static int MvTimespecToNanos(int64_t* ns, const struct timespec* currentTime) {
    if(currentTime && ns) {
        *ns = (int64_t)currentTime->tv_sec * 1000000000;
        *ns += currentTime->tv_nsec;
        return 0;
    } else {
        return -1;
    }
}
static int MvGetTimestamp(struct timespec* currentTime) {
    using namespace std::chrono;
    auto d = steady_clock::now().time_since_epoch();
    currentTime->tv_sec = duration_cast<seconds>(d).count();
    currentTime->tv_nsec = duration_cast<nanoseconds>(d).count() % 1000000000;
    return 0;
}
#define TOD_NANOSECONDS_PER_SECOND (uint32_t)1000000000
static uint32_t _Timespec_Add_to(struct timespec* time, const struct timespec* add) {
    uint32_t seconds = add->tv_sec;

    time->tv_sec += add->tv_sec;
    time->tv_nsec += add->tv_nsec;

    while(time->tv_nsec >= TOD_NANOSECONDS_PER_SECOND) {
        time->tv_nsec -= TOD_NANOSECONDS_PER_SECOND;
        time->tv_sec++;
        seconds++;
    }

    return seconds;
}
static void _Timespec_Subtract(const struct timespec* start, const struct timespec* end, struct timespec* result) {
    if(end->tv_nsec < start->tv_nsec) {
        result->tv_sec = end->tv_sec - start->tv_sec - 1;
        result->tv_nsec = (TOD_NANOSECONDS_PER_SECOND - start->tv_nsec) + end->tv_nsec;
    } else {
        result->tv_sec = end->tv_sec - start->tv_sec;
        result->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
}
static void _Timespec_Divide_by_integer(const struct timespec* time, uint32_t iterations, struct timespec* result) {
    uint64_t t;

    t = time->tv_sec;
    t *= TOD_NANOSECONDS_PER_SECOND;
    t += time->tv_nsec;

    t /= iterations;

    result->tv_sec = t / TOD_NANOSECONDS_PER_SECOND;
    result->tv_nsec = t % TOD_NANOSECONDS_PER_SECOND;
}

#include <sys/param.h>

#include <cstring>
#include <mutex>

#include "XLink/XLink.h"

static std::mutex defaultSyncOffsetMutex;
static struct timespec timesyncDefaultSynchronizedOffset = {};
static struct timespec& defaultSyncOffset = timesyncDefaultSynchronizedOffset;
static int timesyncInstanceCounter = 0;
static int& instanceCounter = timesyncInstanceCounter;

constexpr bool Timesync::PRINT_DEBUG = false;
std::atomic<int> Timesync::instanceCounter{0};

struct timespec Timesync::getDefaultSynchronizedOffset() {
    struct timespec ret;
    std::lock_guard<std::mutex> lock(defaultSyncOffsetMutex);
    ret.tv_sec = defaultSyncOffset.tv_sec;
    ret.tv_nsec = defaultSyncOffset.tv_nsec;
    return ret;
}
struct timespec Timesync::getDefaultSynchronizedTimestamp() {
    timespec ts;
    MvGetTimestamp(&ts);
    auto offset = getDefaultSynchronizedOffset();
    _Timespec_Add_to(&ts, &offset);
    return ts;
}

Timesync::Timesync(std::string name) : streamName(name) {
    instanceNum = instanceCounter++;

    impl = DEFAULT;
}

Timesync::Timesync(std::string name, Implementation implementation) : streamName(name), impl(implementation) {
    instanceNum = instanceCounter++;
}

void Timesync::updateSyncOffsetNanos(uint64_t offsetNanos) {
    struct timespec offset;
    MvNanosToTimespec(&offset, offsetNanos);
    updateSyncOffset(offset);
}

void Timesync::updateSyncOffset(struct timespec offset) {
    {
        std::lock_guard<std::mutex> lock(synchronizedOffsetMutex);
        synchronizedOffset = offset;
    }

    if(instanceNum == 0) {
        std::lock_guard<std::mutex> lock(defaultSyncOffsetMutex);
        defaultSyncOffset.tv_nsec = offset.tv_nsec;
        defaultSyncOffset.tv_sec = offset.tv_sec;
    }

    if(callback) {
        callback(offset);
    }
}

void Timesync::startSyncWait(std::chrono::milliseconds period, int numSamples, bool random) {
    startSync(period, numSamples, random);
    {
        std::unique_lock<std::mutex> lock(syncMtx);
        syncCv.wait(lock, [this]() { return firstSyncComplete; });
    }
}

void Timesync::startSync(std::chrono::milliseconds period, int numSamples, bool random) {
    if(callback == nullptr) {
        printf("Timesync | Callback not set\n");
    }

    syncPeriod = period;
    syncNumSamples = numSamples;
    syncRandom = random;

    if(thread.joinable()) return;

    thread = std::thread([this]() {
        using namespace std::chrono;
        using namespace std::chrono_literals;

        if(syncRandom) std::srand(std::time(nullptr));

        streamId_t streamId = XLinkOpenStream(0, streamName.c_str(), 128);

        struct timespec* pTDevice1 = (struct timespec*)malloc(sizeof(struct timespec));  // Heap-allocated so it is aligned for XLinkWriteData
        assert(pTDevice1 != nullptr);

        struct timespec tDevice2;  // device times
        struct timespec tHost;     // host timestamp

        streamPacketDesc_t* pPacket = nullptr;

        bool isFirst = true;

        constexpr double NANO_RATIO = 1000000000.0;

        struct TimeSyncSample {
            struct timespec offset;
            struct timespec latency;
        };
        std::vector<TimeSyncSample> samples;

        // TODO(themarpe) - perhaps expose
        time_offset = 0.0f;
        time_skew = 0.0;
        sequence = 0;
        filter_alpha = 0;
        filter_beta = 0;
        high_rtt_count = 0;
        high_deviation_count = 0;

        filter_alpha_initial = 0.05f;
        filter_beta_initial = 0.05f;
        filter_alpha_final = 0.003f;
        filter_beta_final = 0.003f;

        convergence_window = 10;  // original 500

        max_rtt_sample = 10;
        max_deviation_sample = 10;
        max_cons_high_rtt = 10;
        max_cons_high_deviation = 10;

        while(threadRunning) {
            const int NUM_SAMPLES = syncNumSamples.load();
            const auto SLEEP_BETWEEN_SYNC = syncPeriod.load();
            const bool RANDOM = syncRandom.load();
            const auto getSleepBetweenSamples = [RANDOM, this]() {
                if(impl == MAVLINK) {
                    if(RANDOM) return milliseconds(std::rand() % 40 + 10);
                    return milliseconds(50);
                } else {
                    if(RANDOM) return milliseconds(std::rand() % 400 + 100);
                    return milliseconds(500);
                }
            };  // 100-500ms

            if(NUM_SAMPLES > 0) {
                samples.resize(NUM_SAMPLES);

                for(int i = 0; i < NUM_SAMPLES; i++) {
                    auto t1 = steady_clock::now();
                    MvGetTimestamp(pTDevice1);
                    XLinkError_t ret;
                    ret = XLinkWriteData(streamId, reinterpret_cast<uint8_t*>(pTDevice1), sizeof(*pTDevice1));
                    if(ret != X_LINK_SUCCESS) {
                        break;
                    }
                    ret = XLinkReadData(streamId, &pPacket);
                    if(ret != X_LINK_SUCCESS) {
                        break;
                    }
                    MvGetTimestamp(&tDevice2);
                    auto t2 = steady_clock::now();
                    assert(pPacket->length == (sizeof(struct timespec)));
                    memcpy(&tHost, pPacket->data, sizeof(tHost));
                    XLinkReleaseData(streamId);

                    int64_t ts1, now_ns, tc1;
                    MvTimespecToNanos(&ts1, pTDevice1);
                    MvTimespecToNanos(&now_ns, &tDevice2);
                    MvTimespecToNanos(&tc1, &tHost);

                    struct timespec rtt;
                    _Timespec_Subtract(pTDevice1, &tDevice2, &rtt);

                    struct timespec latency;
                    _Timespec_Divide_by_integer(&rtt, 2, &latency);

                    if(impl == DEFAULT) {
                        struct timespec offset = {};
                        _Timespec_Subtract(&tDevice2, &tHost, &offset);
                        _Timespec_Add_to(&offset, &latency);

                        if(isFirst) {
                            double ts = ((int64_t)offset.tv_sec) + offset.tv_nsec / NANO_RATIO;

                            updateSyncOffset(offset);

                            {
                                std::unique_lock<std::mutex> lock(syncMtx);
                                firstSyncComplete = true;
                            }
                            syncCv.notify_all();

                            break;
                        }

                        samples[i].offset = offset;
                        samples[i].latency = latency;

                    } else if(impl == MAVLINK) {
                        struct timespec toffset = {};
                        _Timespec_Subtract(&tDevice2, &tHost, &toffset);
                        _Timespec_Add_to(&toffset, &latency);
                        int64_t offset;
                        MvTimespecToNanos(&offset, &toffset);

                        add_timesync_observation(offset, now_ns - ts1, tc1);
                        printf("Added timesync observation, RTT: %llds %lldns, converged: %d, diff: %lldms\n",
                               static_cast<long long>(rtt.tv_sec),
                               static_cast<long long>(rtt.tv_nsec),
                               sync_converged(),
                               static_cast<long long>(duration_cast<milliseconds>(t2 - t1).count()));

                        if(sync_converged() && isFirst) {
                            isFirst = false;

                            {
                                std::unique_lock<std::mutex> lock(syncMtx);
                                firstSyncComplete = true;
                            }
                            syncCv.notify_all();
                        }
                    }

                    std::this_thread::sleep_for(getSleepBetweenSamples());
                }

                if(impl == DEFAULT) {
                    if(isFirst) {
                        isFirst = false;
                        continue;
                    }

                    auto tsToNs = [](const struct timespec& ts) -> int64_t { return ((int64_t)(ts.tv_sec * (int64_t)NANO_RATIO)) + (int64_t)ts.tv_nsec; };

                    auto stdDeviation = [](auto arr) {
                        double sum = 0.0, standardDeviation = 0.0;

                        for(const auto& el : arr) {
                            sum += el.latency.tv_sec + el.latency.tv_nsec / NANO_RATIO;
                        }

                        double mean = sum / arr.size();

                        for(const auto& el : arr) {
                            double lat = el.latency.tv_sec + el.latency.tv_nsec / NANO_RATIO;
                            standardDeviation += std::pow(lat - mean, 2);
                        }

                        return sqrt(standardDeviation / arr.size());
                    };

                    std::sort(samples.begin(), samples.end(), [tsToNs](const auto& lhs, const auto& rhs) -> bool {
                        return tsToNs(lhs.latency) < tsToNs(rhs.latency);
                    });

                    double medianLatency = samples[samples.size() / 2].latency.tv_sec + samples[samples.size() / 2].latency.tv_nsec / 1000000000.0;
                    double stdDevLatency = stdDeviation(samples);

                    int numAverage = 0;
                    for(const auto& el : samples) {
                        double l = el.latency.tv_sec + el.latency.tv_nsec / NANO_RATIO;
                        if(l < medianLatency + stdDevLatency) {
                            numAverage++;
                        }
                    }

                    struct timespec avgOffset = {};
                    int64_t nsAvg = 0;
                    for(const auto& el : samples) {
                        double l = el.latency.tv_sec + el.latency.tv_nsec / NANO_RATIO;
                        if(l < medianLatency + stdDevLatency) {
                            int64_t ns;
                            MvTimespecToNanos(&ns, &el.offset);
                            nsAvg += ns / numAverage;
                        }
                    }
                    MvNanosToTimespec(&avgOffset, nsAvg);

                    {
                        double ts = avgOffset.tv_sec + avgOffset.tv_nsec / NANO_RATIO;
                        updateSyncOffset(avgOffset);
                    }
                } else if(impl == MEDIAN) {
                    if(isFirst) {
                        isFirst = false;
                        continue;
                    }

                    std::nth_element(
                        samples.begin(), samples.begin() + samples.size() / 2, samples.end(), [](const TimeSyncSample& lhs, const TimeSyncSample& rhs) -> bool {
                            return lhs.offset.tv_sec < rhs.offset.tv_sec || (lhs.offset.tv_sec == rhs.offset.tv_sec && lhs.offset.tv_nsec < rhs.offset.tv_nsec);
                        });

                    updateSyncOffset((samples.begin() + samples.size() / 2)->offset);
                }
            }

            std::this_thread::sleep_for(SLEEP_BETWEEN_SYNC);
        }
    });
}

void Timesync::setSyncCallback(std::function<void(const struct timespec&)> cb) {
    callback = cb;
}

struct timespec Timesync::getSyncOffset() const {
    std::lock_guard<std::mutex> lock(synchronizedOffsetMutex);
    return synchronizedOffset;
}

struct timespec Timesync::getSyncTime() const {
    struct timespec syncTime, offset;
    MvGetTimestamp(&syncTime);
    {
        std::lock_guard<std::mutex> lock(synchronizedOffsetMutex);
        offset = synchronizedOffset;
    }
    _Timespec_Add_to(&syncTime, &offset);
    return syncTime;
}

void Timesync::add_timesync_observation(int64_t offset_ns, uint64_t rtt_ns, uint64_t remote_time_ns) {
    uint64_t deviation = llabs(int64_t(time_offset) - offset_ns);

    if(rtt_ns < max_rtt_sample * 1000000ULL) {  // Only use samples with low RTT
        if(sync_converged() && (deviation > max_deviation_sample * 1000000ULL)) {
            high_deviation_count++;

            if(high_deviation_count > max_cons_high_deviation) {
                printf("TM: Time jump detected. Resetting time synchroniser.");

                reset_filter();
            }
        } else {
            if(!sync_converged()) {
                float progress = static_cast<float>(sequence) / convergence_window;
                float p = 1.0f - expf(0.5f * (1.0f - 1.0f / (1.0f - progress)));
                filter_alpha = p * filter_alpha_final + (1.0f - p) * filter_alpha_initial;
                filter_beta = p * filter_beta_final + (1.0f - p) * filter_beta_initial;
            } else {
                filter_alpha = filter_alpha_final;
                filter_beta = filter_beta_final;
            }

            add_sample(offset_ns);

            updateSyncOffsetNanos(sync_converged() ? time_offset : 0);

            sequence++;

            high_deviation_count = 0;

            high_rtt_count = 0;
        }
    } else {
        high_rtt_count++;

        if(high_rtt_count > max_cons_high_rtt) {
            printf("TM: RTT too high for timesync: %0.2f ms.", rtt_ns / 1000000.0);

            high_rtt_count = 0;
        }
    }

    // TODO(themarpe) - if needed or logging
    // // Publish timesync status
    // auto timesync_status = mavros_msgs::msg::TimesyncStatus();
    // timesync_status.header.stamp = node->now();
    // timesync_status.remote_timestamp_ns = remote_time_ns;
    // timesync_status.observed_offset_ns = offset_ns;
    // timesync_status.estimated_offset_ns = time_offset;
    // timesync_status.round_trip_time_ms = static_cast<float>(rtt_ns / 1000000.0);
}

void Timesync::add_sample(int64_t offset_ns) {  // See https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
    double time_offset_prev = time_offset;

    if(sequence == 0) {  // First offset sample
        time_offset = offset_ns;
    } else {
        time_offset = filter_alpha * offset_ns + (1.0 - filter_alpha) * (time_offset + time_skew);

        time_skew = filter_beta * (time_offset - time_offset_prev) + (1.0 - filter_beta) * time_skew;
    }
}

void Timesync::reset_filter() {
    sequence = 0;
    time_offset = 0.0;
    time_skew = 0.0;
    filter_alpha = filter_alpha_initial;
    filter_beta = filter_beta_initial;
    high_deviation_count = 0;
    high_rtt_count = 0;
}

inline bool Timesync::sync_converged() {
    return sequence >= uint32_t(convergence_window);
}

uint64_t Timesync::get_monotonic_now(void) {
    struct timespec spec;
    MvGetTimestamp(&spec);
    return spec.tv_sec * 1000000000ULL + spec.tv_nsec;
}
