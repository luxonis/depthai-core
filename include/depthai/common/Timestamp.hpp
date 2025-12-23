#pragma once

// std
#include <chrono>
#include <cstdint>

#include "depthai/utility/Serialization.hpp"

namespace dai {

/// Timestamp structure
struct Timestamp {
    int64_t sec = 0, nsec = 0;

    template <typename Clock>
    std::chrono::time_point<Clock, typename Clock::duration> get() const {
        using namespace std::chrono;
        using Duration = typename Clock::duration;
        auto total = seconds(sec) + nanoseconds(nsec);
        auto dur = duration_cast<Duration>(total);
        return time_point<Clock, typename Clock::duration>(dur);
    }
};

template std::chrono::time_point<std::chrono::system_clock, std::chrono::system_clock::duration> Timestamp::get<std::chrono::system_clock>() const;
template std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Timestamp::get<std::chrono::steady_clock>() const;

DEPTHAI_SERIALIZE_EXT(Timestamp, sec, nsec);

}  // namespace dai
