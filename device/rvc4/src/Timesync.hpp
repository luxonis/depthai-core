#pragma once

#include <chrono>
#include <thread>
#include <string>
#include <functional>
#include <atomic>
#include <condition_variable>

class Timesync {
    static const bool PRINT_DEBUG;
    static std::atomic<int> instanceCounter;

    std::string streamName;
    std::function<void(const struct timespec&)> callback = nullptr;
    std::thread thread;
    std::atomic<bool> threadRunning{true};
    mutable std::mutex synchronizedOffsetMutex;
    struct timespec synchronizedOffset;
    int instanceNum;

    std::mutex syncMtx;
    std::condition_variable syncCv;
    bool firstSyncComplete{false};

    std::atomic<std::chrono::milliseconds> syncPeriod;
    std::atomic<int> syncNumSamples;
    std::atomic<bool> syncRandom;

    void updateSyncOffset(struct timespec);
    void updateSyncOffsetNanos(uint64_t offsetNanos);

    double time_offset;  // Ported from MAVLINK
    double time_skew;

    uint32_t sequence;
    double filter_alpha;
    double filter_beta;

    float filter_alpha_initial;
    float filter_beta_initial;
    float filter_alpha_final;
    float filter_beta_final;
    int convergence_window;

    int max_rtt_sample;
    int max_deviation_sample;
    int max_cons_high_rtt;
    int max_cons_high_deviation;
    int high_rtt_count;
    int high_deviation_count;

    void add_timesync_observation(int64_t offset_ns, uint64_t local_time_ns, uint64_t remote_time_ns);
    void add_sample(int64_t offset_ns);
    void reset_filter();
    inline bool sync_converged();
    uint64_t get_monotonic_now(void);

public:
    enum Implementation {
        DEFAULT = 0,
        MAVLINK = 1,
        MEDIAN = 2
    };
private:
    Implementation impl;
public:

    Timesync(std::string name);
    Timesync(std::string name, Implementation);
    ~Timesync(){ threadRunning = false; if(thread.joinable()) thread.join(); }
    void startSyncWait(std::chrono::milliseconds period, int numSamples, bool random = true);
    void startSync(std::chrono::milliseconds period, int numSamples, bool random = true);
    void setSyncCallback(std::function<void(const struct timespec&)> cb);
    struct timespec getSyncOffset() const;
    struct timespec getSyncTime() const;

    static struct timespec getDefaultSynchronizedOffset();
    static struct timespec getDefaultSynchronizedTimestamp();

};
