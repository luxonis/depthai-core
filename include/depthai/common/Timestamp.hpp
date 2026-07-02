#pragma once

// std
#include <chrono>
#include <cstdint>

#include "depthai/utility/Serialization.hpp"

namespace dai {

/// Timestamp structure
struct Timestamp {
    int64_t sec = 0, nsec = 0;

    template <typename Clock = std::chrono::steady_clock>
    std::chrono::time_point<Clock, typename Clock::duration> get() const {
        using namespace std::chrono;
        using Duration = typename Clock::duration;
        auto total = seconds(sec) + nanoseconds(nsec);
        auto dur = duration_cast<Duration>(total);
        return time_point<Clock, typename Clock::duration>(dur);
    }
};

DEPTHAI_SERIALIZE_EXT(Timestamp, sec, nsec);

}  // namespace dai
