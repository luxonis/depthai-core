#include "Timesync.hpp"

// std
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <vector>

// #include "mvTimestamp.h"
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

    /* Add the basics */
    time->tv_sec += add->tv_sec;
    time->tv_nsec += add->tv_nsec;

    /* Now adjust it so nanoseconds is in range */
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

    /*
     *  For math simplicity just convert the timespec to nanoseconds
     *  in a 64-bit integer.
     */
    t = time->tv_sec;
    t *= TOD_NANOSECONDS_PER_SECOND;
    t += time->tv_nsec;

    /*
     *  Divide to get nanoseconds per iteration
     */

    t /= iterations;

    /*
     *  Put it back in the timespec result
     */

    result->tv_sec = t / TOD_NANOSECONDS_PER_SECOND;
    result->tv_nsec = t % TOD_NANOSECONDS_PER_SECOND;
}

// CACHE_LINE_SIZE
#include <sys/param.h>

// Mdk
#include <cstring>
#include <mutex>

#include "XLink/XLink.h"

// ALGORITHM

// 1. Client stamps current local time on a "time request" packet and sends to server
// 2. Upon receipt by server, server stamps server-time and returns
// 3. Upon receipt by client, client subtracts current time from sent time and divides by two to compute latency. It subtracts current time from server time to
// determine client-server time delta and adds in the half-latency to get the correct clock delta. (So far this algothim is very similar to SNTP)
// 4. The first result should immediately be used to update the clock since it will get the local clock into at least the right ballpark (at least the right
// timezone!)
// 5. The client repeats steps 1 through 3 five or more times, pausing a few seconds each time. Other traffic may be allowed in the interim, but should be
// minimized for best results
// 6. The results of the packet receipts are accumulated and sorted in lowest-latency to highest-latency order. The median latency is determined by picking the
// mid-point sample from this ordered list.
// 7. All samples above approximately 1 standard-deviation from the median are discarded and the remaining samples are averaged using an arithmetic mean.

//                                   T
//                                   host
//
//                         XX-----------------XX
//                        XX                   XX
//                       XX                     XX
//                      XX                       XX
//                     XX                         XX
//                    XX                           XX
//                   XX                             XX
//                  XX                               XX
// +---------------XX                                 XX---------------+
//
//                 T1                                 T2
//

// Default synchronized offset
static std::mutex defaultSyncOffsetMutex;
static struct timespec timesyncDefaultSynchronizedOffset = {};
static struct timespec& defaultSyncOffset = timesyncDefaultSynchronizedOffset;
static int timesyncInstanceCounter = 0;
static int& instanceCounter = timesyncInstanceCounter;

// static
constexpr bool Timesync::PRINT_DEBUG = false;
std::atomic<int> Timesync::instanceCounter{0};

// static
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

    // Default to DEFAULT timesync impl
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
    // Set synchronizedOffset
    {
        std::lock_guard<std::mutex> lock(synchronizedOffsetMutex);
        synchronizedOffset = offset;
    }

    // if instance num == 0, update default sync offset
    if(instanceNum == 0) {
        std::lock_guard<std::mutex> lock(defaultSyncOffsetMutex);
        defaultSyncOffset.tv_nsec = offset.tv_nsec;
        defaultSyncOffset.tv_sec = offset.tv_sec;
    }

    // notify callback that an update occured
    if(callback) {
        callback(offset);
    }
}

void Timesync::startSyncWait(std::chrono::milliseconds period, int numSamples, bool random) {
    // Starts sync if not already and waits till first sync finished
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

    // Modify Timesync service parameters
    syncPeriod = period;
    syncNumSamples = numSamples;
    syncRandom = random;

    if(thread.joinable()) return;

    // Open XLink stream
    thread = std::thread([this]() {
        using namespace std::chrono;
        using namespace std::chrono_literals;

        if(syncRandom) std::srand(std::time(nullptr));

        streamId_t streamId = XLinkOpenStream(0, streamName.c_str(), 128);

        // Allocate device1 aligned, for XLinkWriteData
        struct timespec* pTDevice1 = (struct timespec*)malloc(sizeof(struct timespec));
        assert(pTDevice1 != nullptr);

        struct timespec tDevice2;  // device times
        struct timespec tHost;     // host timestamp

        streamPacketDesc_t* pPacket = nullptr;

        bool isFirst = true;

        constexpr double NANO_RATIO = 1000000000.0;

        // To keep the vector dynamic memory persistant
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

        // Low compared to ref, for faster first convergence, due to synced wait
        convergence_window = 10;  // original 500

        max_rtt_sample = 10;
        max_deviation_sample = 10;
        max_cons_high_rtt = 10;
        max_cons_high_deviation = 10;

        while(threadRunning) {
            // Create copies of atomic vars
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
                // Resize vector to size of samples specified
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
                    // // NOTE - unused at the moment, overriden below
                    // int64_t offset = (ts1 + now_ns - tc1*2) / 2;

                    // Calculate RTT
                    struct timespec rtt;
                    _Timespec_Subtract(pTDevice1, &tDevice2, &rtt);

                    // Calculate 'one way trip' eg latency
                    struct timespec latency;
                    _Timespec_Divide_by_integer(&rtt, 2, &latency);

                    if(impl == DEFAULT) {
                        // offset
                        struct timespec offset = {};
                        _Timespec_Subtract(&tDevice2, &tHost, &offset);
                        _Timespec_Add_to(&offset, &latency);

                        // Update clock with the acquired offset and break to restart the processing loop
                        if(isFirst) {
                            // update clock with offset, then continue
                            double ts = ((int64_t)offset.tv_sec) + offset.tv_nsec / NANO_RATIO;

                            // update the offset
                            updateSyncOffset(offset);

                            // Notify that first sync is complete
                            {
                                std::unique_lock<std::mutex> lock(syncMtx);
                                firstSyncComplete = true;
                            }
                            syncCv.notify_all();

                            // Break
                            break;
                        }

                        samples[i].offset = offset;
                        samples[i].latency = latency;

                    } else if(impl == MAVLINK) {
                        // offset
                        struct timespec toffset = {};
                        _Timespec_Subtract(&tDevice2, &tHost, &toffset);
                        _Timespec_Add_to(&toffset, &latency);
                        int64_t offset;
                        MvTimespecToNanos(&offset, &toffset);

                        // Calculate RTT by tDevice1+tDevice2 / 2 and assume its the same in both ways
                        add_timesync_observation(offset, now_ns - ts1, tc1);
                        printf("Added timesync observation, RTT: %llds %lldns, converged: %d, diff: %dms\n",
                               rtt.tv_sec,
                               rtt.tv_nsec,
                               sync_converged(),
                               duration_cast<milliseconds>(t2 - t1).count());

                        if(sync_converged() && isFirst) {
                            isFirst = false;

                            // Notify that first sync is complete
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
                    // skip if first and continue after that
                    if(isFirst) {
                        isFirst = false;
                        continue;
                    }

                    auto tsToNs = [](const struct timespec& ts) -> int64_t { return ((int64_t)(ts.tv_sec * (int64_t)NANO_RATIO)) + (int64_t)ts.tv_nsec; };

                    // calculate the limit for outliers
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

                    // mvLog(MVLOG_DEBUG, "List of samples: ");

                    // count num average
                    int numAverage = 0;
                    for(const auto& el : samples) {
                        double l = el.latency.tv_sec + el.latency.tv_nsec / NANO_RATIO;
                        if(l < medianLatency + stdDevLatency) {
                            numAverage++;
                            // mvLog(MVLOG_DEBUG, "%f(nsec: %lld), ", l, ((int64_t)(el.offset.tv_sec * (int64_t)NANO_RATIO)) + (int64_t)el.offset.tv_nsec);
                        }
                    }

                    // divide and add
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

                    // mvLog(MVLOG_DEBUG, "\nstdDevLatency: %f, medianLatency: %f\n", stdDevLatency, medianLatency);
                    // mvLog(MVLOG_DEBUG,
                    //       "avg offset: %f(nsec: %lld)\n",
                    //       avgOffset.tv_sec + avgOffset.tv_nsec / NANO_RATIO,
                    //       ((int64_t)(avgOffset.tv_sec * (int64_t)NANO_RATIO)) + (int64_t)avgOffset.tv_nsec);

                    // callback with offset
                    {
                        double ts = avgOffset.tv_sec + avgOffset.tv_nsec / NANO_RATIO;
                        // mvLog(MVLOG_DEBUG,
                        //       "synced, offset: %f(nsec: %lld)\n",
                        //       ts,
                        //       ((int64_t)(avgOffset.tv_sec * (int64_t)NANO_RATIO)) + (int64_t)avgOffset.tv_nsec);
                        updateSyncOffset(avgOffset);
                    }
                } else if(impl == MEDIAN) {
                    // skip if first and continue after that
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

            // Sleep between syncs
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
    // Calculate the difference of this sample from the current estimate
    uint64_t deviation = llabs(int64_t(time_offset) - offset_ns);

    if(rtt_ns < max_rtt_sample * 1000000ULL) {  // Only use samples with low RTT
        if(sync_converged() && (deviation > max_deviation_sample * 1000000ULL)) {
            // Increment the counter if we have a good estimate and are
            // getting samples far from the estimate
            high_deviation_count++;

            // We reset the filter if we received consecutive samples
            // which violate our present estimate.
            // This is most likely due to a time jump on the offboard system.
            if(high_deviation_count > max_cons_high_deviation) {
                printf("TM: Time jump detected. Resetting time synchroniser.");

                // Reset the filter
                reset_filter();

                // // Reset diagnostics
                // dt_diag.clear();
                // dt_diag.set_timestamp(remote_time_ns);
            }
        } else {
            // Filter gain scheduling
            if(!sync_converged()) {
                // Interpolate with a sigmoid function
                float progress = static_cast<float>(sequence) / convergence_window;
                float p = 1.0f - expf(0.5f * (1.0f - 1.0f / (1.0f - progress)));
                filter_alpha = p * filter_alpha_final + (1.0f - p) * filter_alpha_initial;
                filter_beta = p * filter_beta_final + (1.0f - p) * filter_beta_initial;
            } else {
                filter_alpha = filter_alpha_final;
                filter_beta = filter_beta_final;
            }

            // Perform filter update
            add_sample(offset_ns);

            // Save time offset for other components to use
            updateSyncOffsetNanos(sync_converged() ? time_offset : 0);

            // Increment sequence counter after filter update
            sequence++;

            // Reset high deviation count after filter update
            high_deviation_count = 0;

            // Reset high RTT count after filter update
            high_rtt_count = 0;
        }
    } else {
        // Increment counter if round trip time is too high for accurate timesync
        high_rtt_count++;

        if(high_rtt_count > max_cons_high_rtt) {
            // Issue a warning to the user if the RTT is constantly high
            printf("TM: RTT too high for timesync: %0.2f ms.", rtt_ns / 1000000.0);

            // Reset counter
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

    // timesync_status_pub->publish(timesync_status);

    // // Update diagnostics
    // dt_diag.tick(rtt_ns, remote_time_ns, time_offset);
}

void Timesync::add_sample(int64_t offset_ns) {
    /* Online exponential smoothing filter. The derivative of the estimate is also
     * estimated in order to produce an estimate without steady state lag:
     * https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
     */

    double time_offset_prev = time_offset;

    if(sequence == 0) {  // First offset sample
        time_offset = offset_ns;
    } else {
        // Update the clock offset estimate
        time_offset = filter_alpha * offset_ns + (1.0 - filter_alpha) * (time_offset + time_skew);

        // Update the clock skew estimate
        time_skew = filter_beta * (time_offset - time_offset_prev) + (1.0 - filter_beta) * time_skew;
    }
}

void Timesync::reset_filter() {
    // Do a full reset of all statistics and parameters
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

// Create timestamp sync thread
