#pragma once

#include <chrono>

namespace dai {

struct ptp_clock {
    using duration = std::chrono::nanoseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<ptp_clock, duration>;

    static constexpr bool is_steady = false;  // PHCs are adjustable/steerable
};

}  // namespace dai
